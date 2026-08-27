#include "safety.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "board_config.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "safety";

typedef enum {
    STARTUP_CHECK_CONNECTIVITY = 0,
    STARTUP_CHECK_CURRENT,
    STARTUP_CHECK_FANS,
    STARTUP_CHECK_HOT,
    STARTUP_CHECK_DONE,
} startup_check_t;

static bool s_initialized;
static safety_state_t s_state = SAFETY_STATE_BOOT_SAFE;
static safety_persistent_state_t s_persisted_state = SAFETY_PERSIST_NORMAL;
static bool s_manual_reset_authorized;
static bool s_trip_latched;
static bool s_dc_in_range;
static bool s_startup_fans_started;
static startup_check_t s_startup_check;
static uint32_t s_startup_check_started_ms;
static uint32_t s_dc_stable_started_ms;
static uint32_t s_last_ina_sequence;
static uint32_t s_last_adc_sequence;
static uint32_t s_last_connect_ina_sequence;
static uint32_t s_last_connect_adc_sequence;
static bool s_connect_ina_pending;
static bool s_connect_adc_pending;
static uint32_t s_last_adc_update_ms;
static uint32_t s_last_fan_window_ms;
static uint32_t s_closed_samples;
static uint32_t s_open_samples;
static uint32_t s_low_10v5_samples;
static uint32_t s_low_9v5_samples;
static uint32_t s_hot_stale_failures;
static uint32_t s_last_ina_valid_ms;
static uint32_t s_ina_reconfigure_attempts;
static bool s_fallback_applied;
static float s_fallback_restore_duty;
static uint32_t s_recovery_trial_started_ms;
static bool s_recovery_output_seen;

static volatile bool s_alt_isr_latched;
static volatile uint32_t s_alt_isr_time_ms;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static safety_status_t s_status = {
    .state = SAFETY_STATE_BOOT_SAFE,
    .last_fault = SAFETY_FAULT_NONE,
    .last_error = ESP_ERR_INVALID_STATE,
};
static safety_fault_snapshot_t s_fault_snapshot;
static safety_persistence_request_t s_persistence_request;

static uint32_t safety_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static bool IRAM_ATTR safety_alt_isr_callback(void *user_ctx)
{
    (void)user_ctx;
    portENTER_CRITICAL_ISR(&s_lock);
    s_alt_isr_latched = true;
    s_alt_isr_time_ms = (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
    portEXIT_CRITICAL_ISR(&s_lock);
    return false;
}

static void safety_set_state(safety_state_t state)
{
    s_state = state;
    s_status.state = state;
    s_status.sequence++;
}

static void safety_capture_fault(safety_fault_reason_t reason, uint32_t timestamp_ms)
{
    safety_fault_snapshot_t snapshot = {0};
    snapshot.reason = reason;
    snapshot.timestamp_ms = timestamp_ms;
    (void)ina226_get_snapshot(&snapshot.ina);
    (void)adc_sensors_get_snapshot(&snapshot.adc);
    (void)fan_control_get_status(FAN_ID_1, &snapshot.fan1);
    (void)fan_control_get_status(FAN_ID_2, &snapshot.fan2);
    (void)tec_pwm_get_status(&snapshot.tec);
    portENTER_CRITICAL(&s_lock);
    s_fault_snapshot = snapshot;
    portEXIT_CRITICAL(&s_lock);
}

static void safety_reset_recovery_trial(void)
{
    s_recovery_trial_started_ms = 0U;
    s_recovery_output_seen = false;
}

static void safety_update_recovery_trial(uint32_t now_ms,
                                         const tec_pwm_status_t *tec)
{
    if (s_persisted_state != SAFETY_PERSIST_RETRY_PENDING ||
        s_persistence_request.pending || s_trip_latched ||
        s_state != SAFETY_STATE_RUNNING || tec == NULL ||
        !s_status.permit || !tec->permit || tec->trip_latched ||
        !tec->output_enabled || tec->physical_duty_percent <= 0.0f) {
        safety_reset_recovery_trial();
        return;
    }

    if (!s_recovery_output_seen) {
        s_recovery_output_seen = true;
        s_recovery_trial_started_ms = now_ms;
        return;
    }

    if ((uint32_t)(now_ms - s_recovery_trial_started_ms) >=
        SAFETY_RECOVERY_NORMAL_RUNTIME_MS) {
        portENTER_CRITICAL(&s_lock);
        if (!s_persistence_request.pending &&
            s_persisted_state == SAFETY_PERSIST_RETRY_PENDING) {
            /* The state changes to NORMAL only after the main task has
             * successfully persisted and acknowledged this request. */
            s_persistence_request.pending = true;
            s_persistence_request.requested_state = SAFETY_PERSIST_NORMAL;
            s_persistence_request.reason = SAFETY_FAULT_NONE;
            s_persistence_request.sequence++;
        }
        portEXIT_CRITICAL(&s_lock);
    }
}

static tec_pwm_trip_reason_t safety_tec_trip_reason(safety_fault_reason_t reason)
{
    return reason == SAFETY_FAULT_PWM_ERROR ? TEC_PWM_TRIP_DRIVER_ERROR
                                             : TEC_PWM_TRIP_SOFTWARE_EMERGENCY;
}

static esp_err_t safety_trip(safety_fault_reason_t reason)
{
    if (s_trip_latched) {
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;
    if (reason != SAFETY_FAULT_ALT) {
        ret = tec_pwm_emergency_shutdown(safety_tec_trip_reason(reason));
    }
    /* FAN1 is the controllable hot-side fan. FAN2 is a hardware-fixed
       full-speed cold-side fan and therefore has no software speed command. */
    (void)fan_control_set_full_speed(FAN_ID_1);

    const uint32_t now = reason == SAFETY_FAULT_ALT && s_alt_isr_time_ms != 0U
                             ? s_alt_isr_time_ms : safety_now_ms();
    safety_capture_fault(reason, now);
    s_trip_latched = true;
    s_status.trip_latched = true;
    s_status.permit = false;
    s_status.last_fault = reason;
    s_status.last_fault_time_ms = now;
    s_status.last_error = ret;
    safety_set_state(SAFETY_STATE_TRIPPED);
    safety_reset_recovery_trial();

    portENTER_CRITICAL(&s_lock);
    s_persistence_request.pending = true;
    s_persistence_request.requested_state =
        s_persisted_state == SAFETY_PERSIST_RETRY_PENDING ? SAFETY_PERSIST_LOCKED
                                                          : SAFETY_PERSIST_RETRY_PENDING;
    s_persistence_request.reason = reason;
    s_persistence_request.sequence++;
    portEXIT_CRITICAL(&s_lock);
    return ret;
}

static void safety_startup_fail(safety_fault_reason_t reason)
{
    (void)tec_pwm_request_stop();
    (void)fan_control_set_full_speed(FAN_ID_1);
    const uint32_t now = safety_now_ms();
    safety_capture_fault(reason, now);
    s_status.permit = false;
    s_status.last_fault = reason;
    s_status.last_fault_time_ms = now;
    s_status.last_error = ESP_ERR_INVALID_STATE;
    safety_reset_recovery_trial();
    safety_set_state(SAFETY_STATE_BOOT_FAILED);

#if CONFIG_ICEBOX_FAULT_LOG_NVS
    /* Startup/self-test failures are diagnostic faults, not a second severe
     * retry attempt.  The persistence request keeps the current safety state
     * unchanged and lets the main task persist the RAM fault snapshot. */
    portENTER_CRITICAL(&s_lock);
    s_persistence_request.pending = true;
    s_persistence_request.requested_state = s_persisted_state;
    s_persistence_request.reason = reason;
    s_persistence_request.sequence++;
    portEXIT_CRITICAL(&s_lock);
#endif
}

static bool safety_ina_is_fresh(const ina226_snapshot_t *snapshot)
{
    if (snapshot == NULL || !snapshot->valid || snapshot->sequence == 0U ||
        snapshot->sequence == s_last_ina_sequence) {
        return false;
    }
    s_last_ina_sequence = snapshot->sequence;
    s_last_ina_valid_ms = snapshot->timestamp_ms;
    return true;
}

static bool safety_adc_is_fresh(const adc_sensor_snapshot_t *snapshot)
{
    if (snapshot == NULL || snapshot->sequence == 0U ||
        snapshot->sequence == s_last_adc_sequence) {
        return false;
    }
    s_last_adc_sequence = snapshot->sequence;
    return true;
}

static void safety_begin_self_test(uint32_t now_ms)
{
    tec_pwm_status_t tec = {0};
    if (tec_pwm_get_status(&tec) != ESP_OK ||
        tec.target_duty_percent != 0.0f ||
        tec.ramped_duty_percent != 0.0f || tec.output_enabled) {
        return;
    }
    if (tec.permit && tec_pwm_revoke_permit() != ESP_OK) {
        return;
    }

    s_startup_check = STARTUP_CHECK_CONNECTIVITY;
    s_startup_check_started_ms = now_ms;
    s_startup_fans_started = false;
    s_closed_samples = 0;
    s_open_samples = 0;
    s_last_connect_ina_sequence = 0;
    s_last_connect_adc_sequence = 0;
    s_connect_ina_pending = false;
    s_connect_adc_pending = false;
    s_status.startup_checks_complete = false;
    s_status.permit = false;
    safety_reset_recovery_trial();
    safety_set_state(SAFETY_STATE_POWER_SELF_TEST);
}

static bool safety_startup_timeout(uint32_t now_ms)
{
    if (s_startup_check == STARTUP_CHECK_FANS) {
        return false;
    }
    if ((uint32_t)(now_ms - s_startup_check_started_ms) >=
        SAFETY_STARTUP_CHECK_TIMEOUT_MS) {
        safety_startup_fail(SAFETY_FAULT_STARTUP_TIMEOUT);
        return true;
    }
    return false;
}

static void safety_advance_startup_check(uint32_t now_ms)
{
    s_startup_check++;
    s_startup_check_started_ms = now_ms;
}

static void safety_run_self_test(uint32_t now_ms,
                                 const ina226_snapshot_t *ina,
                                 const adc_sensor_snapshot_t *adc,
                                 bool ina_fresh,
                                 bool adc_fresh)
{
    if (safety_startup_timeout(now_ms)) {
        return;
    }

    if (s_startup_check == STARTUP_CHECK_CONNECTIVITY) {
        if (ina_fresh) {
            s_connect_ina_pending = true;
        }
        if (adc_fresh) {
            s_connect_adc_pending = true;
        }
        if (s_connect_ina_pending && s_connect_adc_pending && ina->valid &&
            adc->tec_n_valid && ina->sequence != s_last_connect_ina_sequence &&
            adc->sequence != s_last_connect_adc_sequence) {
            s_last_connect_ina_sequence = ina->sequence;
            s_last_connect_adc_sequence = adc->sequence;
            s_connect_ina_pending = false;
            s_connect_adc_pending = false;
            const float vn = adc->tec_n_voltage_v;
            const float vbus = ina->bus_voltage_v;
            if (fabsf(vn - vbus) < SAFETY_TEC_CLOSED_MAX_DIFF_V) {
                s_closed_samples++;
                s_open_samples = 0;
            } else if (fabsf(vn) <= SAFETY_TEC_OPEN_MAX_ABS_V) {
                s_open_samples++;
                s_closed_samples = 0;
            } else {
                s_open_samples++;
                s_closed_samples = 0;
            }
            if (s_closed_samples >= SAFETY_TEC_CONNECT_VALID_SAMPLES) {
                safety_advance_startup_check(now_ms);
            } else if (s_open_samples >= SAFETY_TEC_CONNECT_VALID_SAMPLES) {
                safety_startup_fail(SAFETY_FAULT_TEC_OPEN);
            }
        }
        return;
    }

    if (s_startup_check == STARTUP_CHECK_CURRENT) {
        if (ina_fresh && ina->valid) {
            if (ina->current_a > SAFETY_STARTUP_PWM_OFF_FAULT_CURRENT_A) {
                safety_startup_fail(SAFETY_FAULT_STARTUP_CURRENT);
            } else if (ina->current_a < SAFETY_STARTUP_MAX_CURRENT_A) {
                safety_advance_startup_check(now_ms);
            }
        }
        return;
    }

    if (s_startup_check == STARTUP_CHECK_FANS) {
        if (!s_startup_fans_started) {
            esp_err_t ret = fan_control_set_full_speed(FAN_ID_1);
            if (ret == ESP_OK) {
                ret = fan_control_tach_clear(FAN_ID_1);
            }
            if (ret == ESP_OK) {
                /* FAN2 is the fixed-speed cold-side fan; only start a fresh
                   tach window, never issue a software speed command. */
                ret = fan_control_tach_clear(FAN_ID_2);
            }
            s_startup_fans_started = true;
            if (ret != ESP_OK) {
                safety_startup_fail(SAFETY_FAULT_STARTUP_TIMEOUT);
                return;
            }
        }
        if ((uint32_t)(now_ms - s_startup_check_started_ms) >=
            SAFETY_FAN_STARTUP_WINDOW_MS) {
            fan_tach_sample_t fan1 = {0};
            fan_tach_sample_t fan2 = {0};
            esp_err_t ret1 = fan_control_tach_read(FAN_ID_1, &fan1);
            esp_err_t ret2 = fan_control_tach_read(FAN_ID_2, &fan2);
            if (ret1 != ESP_OK || ret2 != ESP_OK ||
                fan1.pulses < SAFETY_FAN_STARTUP_MIN_PULSES ||
                fan2.pulses < SAFETY_FAN_STARTUP_MIN_PULSES) {
                safety_startup_fail(fan1.pulses < SAFETY_FAN_STARTUP_MIN_PULSES ?
                                    SAFETY_FAULT_FAN1_STALL : SAFETY_FAULT_FAN2_STALL);
            } else {
                safety_advance_startup_check(now_ms);
            }
        }
        return;
    }

    if (s_startup_check == STARTUP_CHECK_HOT) {
        if (adc_fresh) {
            if (!adc->hot_valid) {
                safety_startup_fail(SAFETY_FAULT_HOT_INVALID);
            } else if (!isfinite(adc->hot_temp_c) ||
                       adc->hot_temp_c > SAFETY_HOT_START_MAX_TEMP_C) {
                safety_startup_fail(SAFETY_FAULT_HOT_OVERTEMPERATURE);
            } else {
                safety_advance_startup_check(now_ms);
            }
        }
        return;
    }

    if (s_startup_check == STARTUP_CHECK_DONE) {
        if (tec_pwm_recover() != ESP_OK || tec_pwm_grant_permit() != ESP_OK) {
            safety_startup_fail(SAFETY_FAULT_PWM_ERROR);
            return;
        }
        s_status.startup_checks_complete = true;
        s_status.permit = true;
        s_last_fan_window_ms = now_ms;
        safety_set_state(SAFETY_STATE_READY);
    }
}

static void safety_handle_power(uint32_t now_ms, const ina226_snapshot_t *ina,
                                bool fresh)
{
    const bool active = s_state == SAFETY_STATE_RUNNING || s_state == SAFETY_STATE_READY;
    const uint32_t last_sample_ms = ina->valid ? ina->timestamp_ms : s_last_ina_valid_ms;
    const bool stale = active && (!ina->valid || !fresh) &&
                       (uint32_t)(now_ms - last_sample_ms) >= SAFETY_INA_STALE_TIMEOUT_MS;
    if (stale) {
        if (!s_fallback_applied) {
            tec_pwm_status_t tec = {0};
            if (tec_pwm_get_status(&tec) == ESP_OK) {
                s_fallback_restore_duty = tec.target_duty_percent;
                s_fallback_applied = true;
                (void)safety_set_duty(s_fallback_restore_duty * 0.5f);
            }
        } else if (s_ina_reconfigure_attempts < SAFETY_INA_RECONFIGURE_ATTEMPTS) {
            s_ina_reconfigure_attempts++;
            (void)ina226_reconfigure();
        } else {
            (void)safety_trip(SAFETY_FAULT_INA_COMMUNICATION);
        }
        return;
    }

    if (!ina->valid || !fresh) {
        return;
    }

    if (s_fallback_applied) {
        (void)safety_set_duty(s_fallback_restore_duty);
        s_fallback_applied = false;
        s_fallback_restore_duty = 0.0f;
    }
    s_ina_reconfigure_attempts = 0;
    if (ina->bus_voltage_v > SAFETY_HARD_OVERVOLTAGE_V) {
        (void)safety_trip(SAFETY_FAULT_OVERVOLTAGE);
        return;
    }

    if ((s_state == SAFETY_STATE_RUNNING || s_state == SAFETY_STATE_READY) &&
        ina->bus_voltage_v >= SAFETY_NO_DC_MAX_V) {
        if (ina->bus_voltage_v < 9.5f) {
            s_low_9v5_samples++;
        } else {
            s_low_9v5_samples = 0;
        }
        if (ina->bus_voltage_v < 10.5f) {
            s_low_10v5_samples++;
        } else {
            s_low_10v5_samples = 0;
        }
        if (s_low_9v5_samples >= SAFETY_LOW_VOLTAGE_9V5_SAMPLES ||
            s_low_10v5_samples >= SAFETY_LOW_VOLTAGE_10V5_SAMPLES) {
            (void)safety_trip(SAFETY_FAULT_LOW_VOLTAGE);
            return;
        }
    } else {
        s_low_9v5_samples = 0;
        s_low_10v5_samples = 0;
    }

    if (ina->current_a >= TEC_CURRENT_SAFETY_TRIP_A) {
        (void)safety_trip(SAFETY_FAULT_OVERCURRENT);
        return;
    }

    if (s_state == SAFETY_STATE_RUNNING || s_state == SAFETY_STATE_READY) {
        if (ina->bus_voltage_v < SAFETY_NO_DC_MAX_V) {
            (void)tec_pwm_request_stop();
            s_status.permit = false;
            (void)tec_pwm_revoke_permit();
            s_dc_in_range = false;
            s_status.dc_in_range = false;
            s_status.startup_checks_complete = false;
            safety_set_state(SAFETY_STATE_NO_DC);
        } else if (ina->bus_voltage_v < SAFETY_DC_START_MIN_V) {
            (void)tec_pwm_request_stop();
            s_status.permit = false;
            (void)tec_pwm_revoke_permit();
            s_dc_in_range = false;
            s_status.dc_in_range = false;
            s_status.startup_checks_complete = false;
            safety_set_state(SAFETY_STATE_POWER_INVALID);
        }
    }
}

static void safety_handle_adc(const adc_sensor_snapshot_t *adc, bool fresh, bool poll_due)
{
    if (s_state != SAFETY_STATE_RUNNING && s_state != SAFETY_STATE_READY) {
        return;
    }
    if (fresh) {
        s_hot_stale_failures = 0;
        if (!adc->hot_valid) {
            (void)safety_trip(SAFETY_FAULT_HOT_INVALID);
            return;
        }
        if (!isfinite(adc->hot_temp_c) || adc->hot_temp_c > SAFETY_HOT_START_MAX_TEMP_C) {
            (void)safety_trip(SAFETY_FAULT_HOT_OVERTEMPERATURE);
            return;
        }
        if (!adc->cold_valid) {
            (void)safety_trip(SAFETY_FAULT_COLD_INVALID);
        }
    } else if (poll_due) {
        s_hot_stale_failures++;
        if (s_hot_stale_failures >= SAFETY_HOT_STALE_FAILURE_LIMIT) {
            (void)safety_trip(SAFETY_FAULT_HOT_STALE);
        }
    }
}

static void safety_handle_fans(uint32_t now_ms)
{
    if (s_state != SAFETY_STATE_RUNNING) {
        return;
    }
    if ((uint32_t)(now_ms - s_last_fan_window_ms) < SAFETY_FAN_RUNTIME_WINDOW_MS) {
        return;
    }
    s_last_fan_window_ms = now_ms;
    fan_tach_sample_t fan1 = {0};
    fan_tach_sample_t fan2 = {0};
    esp_err_t ret1 = fan_control_tach_read(FAN_ID_1, &fan1);
    esp_err_t ret2 = fan_control_tach_read(FAN_ID_2, &fan2);
    if (ret1 != ESP_OK || fan1.pulses < SAFETY_FAN_STARTUP_MIN_PULSES) {
        (void)safety_trip(SAFETY_FAULT_FAN1_STALL);
        return;
    }
    if (ret2 != ESP_OK || fan2.pulses < SAFETY_FAN_STARTUP_MIN_PULSES) {
        (void)safety_trip(SAFETY_FAULT_FAN2_STALL);
        return;
    }
    (void)fan_control_tach_clear(FAN_ID_1);
    (void)fan_control_tach_clear(FAN_ID_2);
}

esp_err_t safety_init(const safety_boot_context_t *boot)
{
    if (boot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_fault_snapshot, 0, sizeof(s_fault_snapshot));
    memset(&s_persistence_request, 0, sizeof(s_persistence_request));
    s_persisted_state = boot->persisted_state;
    s_manual_reset_authorized = boot->manual_reset_authorized;
    s_trip_latched = false;
    s_dc_in_range = false;
    s_alt_isr_latched = false;
    s_alt_isr_time_ms = 0;
    s_recovery_trial_started_ms = 0U;
    s_recovery_output_seen = false;
    s_status = (safety_status_t) {
        .state = SAFETY_STATE_BOOT_SAFE,
        .last_fault = SAFETY_FAULT_NONE,
        .last_error = ESP_OK,
    };
    safety_set_state(SAFETY_STATE_BOOT_SAFE);

    esp_err_t ret = tec_pwm_init(safety_alt_isr_callback, NULL);
    if (ret != ESP_OK) {
        safety_set_state(SAFETY_STATE_BOOT_FAILED);
        s_status.last_error = ret;
        return ret;
    }
    s_initialized = true;
    if (s_persisted_state == SAFETY_PERSIST_LOCKED && !s_manual_reset_authorized) {
        s_trip_latched = true;
        s_status.trip_latched = true;
        safety_set_state(SAFETY_STATE_BOOT_FAILED);
    }
    ESP_LOGI(TAG, "initialized");
    return ESP_OK;
}

esp_err_t safety_mark_boot_failed(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    safety_startup_fail(SAFETY_FAULT_BOOT_FAILURE);
    return ESP_OK;
}

esp_err_t safety_apply_persisted_state(safety_persistent_state_t state,
                                       bool manual_reset_authorized)
{
    if (!s_initialized || state < SAFETY_PERSIST_NORMAL || state > SAFETY_PERSIST_LOCKED) {
        return !s_initialized ? ESP_ERR_INVALID_STATE : ESP_ERR_INVALID_ARG;
    }
    if (s_state != SAFETY_STATE_BOOT_SAFE) {
        return ESP_ERR_INVALID_STATE;
    }
    s_persisted_state = state;
    s_manual_reset_authorized = manual_reset_authorized;
    if (state == SAFETY_PERSIST_LOCKED && !manual_reset_authorized) {
        s_trip_latched = true;
        s_status.trip_latched = true;
        s_status.permit = false;
        safety_set_state(SAFETY_STATE_BOOT_FAILED);
    }
    return ESP_OK;
}

esp_err_t safety_set_duty(float duty_percent)
{
    if (!isfinite(duty_percent) || duty_percent < 0.0f || duty_percent > 100.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized || s_trip_latched ||
        (duty_percent > 0.0f && s_state != SAFETY_STATE_READY &&
         s_state != SAFETY_STATE_RUNNING)) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = tec_pwm_set_target(duty_percent);
    if (ret == ESP_OK) {
        if (duty_percent > 0.0f) {
            safety_set_state(SAFETY_STATE_RUNNING);
            s_status.permit = true;
        } else {
            /* A stop request is valid in every safe state.  Do not turn a
             * revoked permit back on merely because the requested duty is
             * zero (for example while in NO_DC). */
            tec_pwm_status_t tec = {0};
            if (tec_pwm_get_status(&tec) == ESP_OK) {
                s_status.permit = tec.permit && !s_trip_latched &&
                                  (s_state == SAFETY_STATE_READY ||
                                   s_state == SAFETY_STATE_RUNNING);
            }
        }
        s_status.last_error = ESP_OK;
    }
    return ret;
}

esp_err_t safety_emergency_shutdown(safety_fault_reason_t reason)
{
    if (reason == SAFETY_FAULT_NONE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return safety_trip(reason);
}

esp_err_t safety_tick_10ms(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint32_t now = safety_now_ms();
    ina226_snapshot_t ina = {0};
    adc_sensor_snapshot_t adc = {0};
    tec_pwm_status_t tec = {0};
    (void)ina226_update();
    (void)ina226_get_snapshot(&ina);
    const bool adc_poll_due = (uint32_t)(now - s_last_adc_update_ms) >=
                              SAFETY_ADC_UPDATE_PERIOD_MS;
    if (adc_poll_due) {
        (void)adc_sensors_update();
        s_last_adc_update_ms = now;
    }
    (void)adc_sensors_get_snapshot(&adc);
    (void)tec_pwm_get_status(&tec);

    bool alt_event;
    uint32_t alt_time_ms;
    portENTER_CRITICAL(&s_lock);
    alt_event = s_alt_isr_latched;
    alt_time_ms = s_alt_isr_time_ms;
    s_alt_isr_latched = false;
    portEXIT_CRITICAL(&s_lock);
    if (alt_event || tec.last_trip_reason == TEC_PWM_TRIP_ALT) {
        s_alt_isr_time_ms = alt_time_ms;
        (void)safety_trip(SAFETY_FAULT_ALT);
        s_alt_isr_time_ms = 0;
        return ESP_OK;
    }

    const bool ina_fresh = safety_ina_is_fresh(&ina);
    const bool adc_fresh = safety_adc_is_fresh(&adc);
    safety_handle_power(now, &ina, ina_fresh);
    safety_handle_adc(&adc, adc_fresh, adc_poll_due);

    if (s_state == SAFETY_STATE_TRIPPED || s_state == SAFETY_STATE_BOOT_FAILED) {
        return ESP_OK;
    }

    if (s_state == SAFETY_STATE_BOOT_SAFE || s_state == SAFETY_STATE_NO_DC ||
        s_state == SAFETY_STATE_POWER_INVALID) {
        if (!ina.valid || ina.bus_voltage_v < SAFETY_NO_DC_MAX_V) {
            s_dc_in_range = false;
            s_status.dc_in_range = false;
            s_status.startup_checks_complete = false;
            safety_reset_recovery_trial();
            safety_set_state(SAFETY_STATE_NO_DC);
            return ESP_OK;
        }
        if (ina.bus_voltage_v > SAFETY_HARD_OVERVOLTAGE_V) {
            (void)safety_trip(SAFETY_FAULT_OVERVOLTAGE);
            return ESP_OK;
        }
        if (ina.bus_voltage_v < SAFETY_DC_START_MIN_V) {
            s_dc_in_range = false;
            s_status.dc_in_range = false;
            s_status.startup_checks_complete = false;
            safety_reset_recovery_trial();
            safety_set_state(SAFETY_STATE_POWER_INVALID);
            return ESP_OK;
        }
        if (ina.bus_voltage_v <= SAFETY_DC_START_MAX_V && ina_fresh) {
            if (!s_dc_in_range) {
                s_dc_in_range = true;
                s_dc_stable_started_ms = now;
                s_status.dc_in_range = true;
            } else if ((uint32_t)(now - s_dc_stable_started_ms) >=
                       SAFETY_DC_STABLE_TIME_MS) {
                safety_begin_self_test(now);
            }
        }
    }

    if (s_state == SAFETY_STATE_POWER_SELF_TEST) {
        if (!ina.valid || ina.bus_voltage_v < SAFETY_NO_DC_MAX_V) {
            (void)tec_pwm_request_stop();
            s_status.permit = false;
            (void)tec_pwm_revoke_permit();
            s_dc_in_range = false;
            s_status.dc_in_range = false;
            s_status.startup_checks_complete = false;
            safety_reset_recovery_trial();
            safety_set_state(SAFETY_STATE_NO_DC);
            return ESP_OK;
        }
        if (ina.bus_voltage_v < SAFETY_DC_START_MIN_V) {
            (void)tec_pwm_request_stop();
            s_status.permit = false;
            (void)tec_pwm_revoke_permit();
            s_dc_in_range = false;
            s_status.dc_in_range = false;
            s_status.startup_checks_complete = false;
            safety_reset_recovery_trial();
            safety_set_state(SAFETY_STATE_POWER_INVALID);
            return ESP_OK;
        }
        safety_run_self_test(now, &ina, &adc, ina_fresh, adc_fresh);
    }

    if (s_state == SAFETY_STATE_READY || s_state == SAFETY_STATE_RUNNING) {
        if (tec.trip_latched || tec.state == TEC_PWM_STATE_TRIPPED) {
            (void)safety_trip(SAFETY_FAULT_PWM_ERROR);
            return ESP_OK;
        }
        if (tec.ramped_duty_percent == 0.0f && tec.target_duty_percent == 0.0f &&
            s_state == SAFETY_STATE_RUNNING) {
            s_status.permit = tec.permit;
            safety_set_state(SAFETY_STATE_READY);
        }
        safety_handle_fans(now);
    }
    if (s_state == SAFETY_STATE_POWER_INVALID &&
        tec.target_duty_percent == 0.0f && tec.ramped_duty_percent == 0.0f &&
        tec.permit) {
        if (tec_pwm_revoke_permit() == ESP_OK) {
            s_status.permit = false;
        }
    }
    if ((s_state == SAFETY_STATE_NO_DC || s_state == SAFETY_STATE_BOOT_SAFE) &&
        tec.target_duty_percent == 0.0f && tec.ramped_duty_percent == 0.0f &&
        tec.permit) {
        if (tec_pwm_revoke_permit() == ESP_OK) {
            s_status.permit = false;
        }
    }
    safety_update_recovery_trial(now, &tec);
    return ESP_OK;
}

esp_err_t safety_get_status(safety_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_lock);
    *status = s_status;
    status->alt_latched = s_alt_isr_latched;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t safety_get_fault_snapshot(safety_fault_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_lock);
    *snapshot = s_fault_snapshot;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t safety_get_persistence_request(safety_persistence_request_t *request)
{
    if (request == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_lock);
    *request = s_persistence_request;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t safety_ack_persistence_request(uint32_t sequence)
{
    if (sequence == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    if (!s_initialized) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_persistence_request.pending ||
        s_persistence_request.sequence != sequence) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_NOT_FOUND;
    }
    s_persisted_state = s_persistence_request.requested_state;
    s_persistence_request.pending = false;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}
