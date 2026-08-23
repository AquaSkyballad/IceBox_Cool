#include "tec_pwm.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

#include "gpio_def.h"

#if SOC_MCPWM_GROUPS <= TEC_PWM_MCPWM_GROUP_ID
#error "Selected TEC MCPWM group is not available on this target"
#endif

static const char *TAG = "tec_pwm";

static mcpwm_timer_handle_t s_timer;
static mcpwm_oper_handle_t s_operator;
static mcpwm_cmpr_handle_t s_comparator;
static mcpwm_gen_handle_t s_generator;
static mcpwm_fault_handle_t s_alt_fault;

static bool s_timer_enabled;
static bool s_timer_started;
static bool s_initialized;
static bool s_force_low_held;
static bool s_ost_latched;
static bool s_output_enabled;
static bool s_permit;
static bool s_trip_latched;

static float s_target_duty_percent;
static float s_ramped_duty_percent;
static float s_physical_duty_percent;
static uint32_t s_compare_ticks;
static tec_pwm_trip_reason_t s_last_trip_reason;
static uint32_t s_last_trip_time_ms;
static tec_pwm_trip_snapshot_t s_last_trip_snapshot;
static uint32_t s_sequence;
static esp_err_t s_last_error;

static tec_pwm_alt_isr_callback_t s_alt_callback;
static void *s_alt_callback_ctx;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t tec_pwm_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static uint32_t IRAM_ATTR tec_pwm_now_ms_from_isr(void)
{
    return (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
}

static tec_pwm_state_t tec_pwm_state_locked(void)
{
    if (!s_initialized) {
        return TEC_PWM_STATE_UNINITIALIZED;
    }
    if (s_trip_latched) {
        return TEC_PWM_STATE_TRIPPED;
    }
    if (!s_permit) {
        return TEC_PWM_STATE_SAFE_OFF;
    }
    if (s_output_enabled || s_ramped_duty_percent > 0.0f) {
        return TEC_PWM_STATE_RUNNING;
    }
    return TEC_PWM_STATE_READY;
}

static esp_err_t tec_pwm_configure_safe_gpio(void)
{
    esp_err_t ret = gpio_reset_pin(TEC_PWM_GPIO);
    if (ret == ESP_OK) {
        ret = gpio_set_level(TEC_PWM_GPIO, 0);
    }
    if (ret == ESP_OK) {
        ret = gpio_set_direction(TEC_PWM_GPIO, GPIO_MODE_OUTPUT);
    }
    if (ret == ESP_OK) {
        ret = gpio_set_level(TEC_PWM_GPIO, 0);
    }
    return ret;
}

static esp_err_t tec_pwm_reset_alt_gpio(void)
{
    esp_err_t ret = gpio_reset_pin(INA_ALT_GPIO);
    if (ret == ESP_OK) {
        ret = gpio_set_direction(INA_ALT_GPIO, GPIO_MODE_INPUT);
    }
    if (ret == ESP_OK) {
        ret = gpio_set_pull_mode(INA_ALT_GPIO, GPIO_FLOATING);
    }
    return ret;
}

static esp_err_t tec_pwm_release_resources(void)
{
    esp_err_t first_error = ESP_OK;

    if (s_generator != NULL) {
        esp_err_t ret = mcpwm_generator_set_force_level(s_generator, 0, true);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
    }
    if (s_timer_started && s_timer != NULL) {
        esp_err_t ret = mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_STOP_EMPTY);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_timer_started = false;
    }
    if (s_timer_enabled && s_timer != NULL) {
        esp_err_t ret = mcpwm_timer_disable(s_timer);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_timer_enabled = false;
    }
    if (s_alt_fault != NULL) {
        esp_err_t ret = mcpwm_del_fault(s_alt_fault);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_alt_fault = NULL;
    }
    if (s_generator != NULL) {
        esp_err_t ret = mcpwm_del_generator(s_generator);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_generator = NULL;
    }
    if (s_comparator != NULL) {
        esp_err_t ret = mcpwm_del_comparator(s_comparator);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_comparator = NULL;
    }
    if (s_operator != NULL) {
        esp_err_t ret = mcpwm_del_operator(s_operator);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_operator = NULL;
    }
    if (s_timer != NULL) {
        esp_err_t ret = mcpwm_del_timer(s_timer);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_timer = NULL;
    }
    esp_err_t ret = tec_pwm_reset_alt_gpio();
    if (ret != ESP_OK && first_error == ESP_OK) {
        first_error = ret;
    }
    ret = tec_pwm_configure_safe_gpio();
    if (ret != ESP_OK && first_error == ESP_OK) {
        first_error = ret;
    }
    return first_error;
}

static uint32_t tec_pwm_duty_to_ticks(float duty_percent)
{
    long ticks = lroundf((duty_percent * (float)TEC_PWM_PERIOD_TICKS) / 100.0f);
    if (ticks <= 0) {
        return 0;
    }
    if ((uint32_t)ticks > TEC_PWM_MAX_COMPARE_TICKS) {
        return TEC_PWM_MAX_COMPARE_TICKS;
    }
    return (uint32_t)ticks;
}

static float tec_pwm_ticks_to_duty(uint32_t ticks)
{
    return ((float)ticks * 100.0f) / (float)TEC_PWM_PERIOD_TICKS;
}

static void IRAM_ATTR tec_pwm_latch_trip_locked(tec_pwm_trip_reason_t reason,
                                                uint32_t time_ms,
                                                esp_err_t error)
{
    const bool capture_trip = !s_trip_latched ||
                              s_last_trip_reason == TEC_PWM_TRIP_POWER_ON;
    if (capture_trip) {
        s_last_trip_snapshot = (tec_pwm_trip_snapshot_t) {
            .target_duty_percent = s_target_duty_percent,
            .ramped_duty_percent = s_ramped_duty_percent,
            .physical_duty_percent = s_physical_duty_percent,
            .compare_ticks = s_compare_ticks,
            .output_enabled = s_output_enabled,
        };
        s_last_trip_reason = reason;
        s_last_trip_time_ms = time_ms;
    }
    s_target_duty_percent = 0.0f;
    s_ramped_duty_percent = 0.0f;
    s_physical_duty_percent = 0.0f;
    s_compare_ticks = 0;
    s_output_enabled = false;
    s_permit = false;
    s_trip_latched = true;
    if (capture_trip || error != ESP_OK) {
        s_last_error = error;
    }
    s_sequence++;
}

static esp_err_t tec_pwm_trip_from_task(tec_pwm_trip_reason_t reason, esp_err_t cause)
{
    esp_err_t force_ret = ESP_OK;
    esp_err_t compare_ret = ESP_OK;

    if (s_generator != NULL) {
        force_ret = mcpwm_generator_set_force_level(s_generator, 0, true);
    }
    if (s_comparator != NULL) {
        compare_ret = mcpwm_comparator_set_compare_value(s_comparator, 0);
    }

    const esp_err_t shutdown_error = force_ret != ESP_OK ? force_ret :
                                     (compare_ret != ESP_OK ? compare_ret : cause);
    portENTER_CRITICAL(&s_lock);
    s_force_low_held = true;
    tec_pwm_latch_trip_locked(reason, tec_pwm_now_ms(), shutdown_error);
    portEXIT_CRITICAL(&s_lock);

    if (force_ret != ESP_OK) {
        return force_ret;
    }
    if (compare_ret != ESP_OK) {
        return compare_ret;
    }
    return cause;
}

static bool IRAM_ATTR tec_pwm_on_ost_brake(mcpwm_oper_handle_t oper,
                                            const mcpwm_brake_event_data_t *event_data,
                                            void *user_ctx)
{
    (void)oper;
    (void)event_data;
    (void)user_ctx;

    tec_pwm_alt_isr_callback_t callback = NULL;
    void *callback_ctx = NULL;

    portENTER_CRITICAL_ISR(&s_lock);
    if (s_initialized) {
        s_ost_latched = true;
        tec_pwm_latch_trip_locked(TEC_PWM_TRIP_ALT, tec_pwm_now_ms_from_isr(), ESP_OK);
        callback = s_alt_callback;
        callback_ctx = s_alt_callback_ctx;
    }
    portEXIT_CRITICAL_ISR(&s_lock);

    return callback != NULL ? callback(callback_ctx) : false;
}

esp_err_t tec_pwm_init(tec_pwm_alt_isr_callback_t alt_callback, void *user_ctx)
{
    portENTER_CRITICAL(&s_lock);
    const bool already_initialized = s_initialized;
    portEXIT_CRITICAL(&s_lock);
    if (already_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = tec_pwm_configure_safe_gpio();
    if (ret != ESP_OK) {
        return ret;
    }
    ret = tec_pwm_reset_alt_gpio();
    if (ret != ESP_OK) {
        return ret;
    }

    const mcpwm_timer_config_t timer_config = {
        .group_id = TEC_PWM_MCPWM_GROUP_ID,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TEC_PWM_RESOLUTION_HZ,
        .period_ticks = TEC_PWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ret = mcpwm_new_timer(&timer_config, &s_timer);

    const mcpwm_operator_config_t operator_config = {
        .group_id = TEC_PWM_MCPWM_GROUP_ID,
        .intr_priority = 3,
    };
    if (ret == ESP_OK) {
        ret = mcpwm_new_operator(&operator_config, &s_operator);
    }
    if (ret == ESP_OK) {
        ret = mcpwm_operator_connect_timer(s_operator, s_timer);
    }

    const mcpwm_operator_event_callbacks_t operator_callbacks = {
        .on_brake_ost = tec_pwm_on_ost_brake,
    };
    if (ret == ESP_OK) {
        ret = mcpwm_operator_register_event_callbacks(s_operator, &operator_callbacks, NULL);
    }

    const mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    if (ret == ESP_OK) {
        ret = mcpwm_new_comparator(s_operator, &comparator_config, &s_comparator);
    }
    if (ret == ESP_OK) {
        ret = mcpwm_comparator_set_compare_value(s_comparator, 0);
    }

    const mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = TEC_PWM_GPIO,
    };
    if (ret == ESP_OK) {
        ret = mcpwm_new_generator(s_operator, &generator_config, &s_generator);
    }
    if (ret == ESP_OK) {
        ret = mcpwm_generator_set_force_level(s_generator, 0, true);
    }
    if (ret == ESP_OK) {
        ret = mcpwm_generator_set_action_on_timer_event(
            s_generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH));
    }
    if (ret == ESP_OK) {
        ret = mcpwm_generator_set_action_on_compare_event(
            s_generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           s_comparator,
                                           MCPWM_GEN_ACTION_LOW));
    }
    if (ret == ESP_OK) {
        ret = mcpwm_generator_set_action_on_brake_event(
            s_generator,
            MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_OPER_BRAKE_MODE_OST,
                                         MCPWM_GEN_ACTION_LOW));
    }

    const mcpwm_gpio_fault_config_t fault_config = {
        .group_id = TEC_PWM_MCPWM_GROUP_ID,
        .intr_priority = 3,
        .gpio_num = INA_ALT_GPIO,
        .flags.active_level = 0,
        .flags.pull_up = false,
        .flags.pull_down = false,
    };
    if (ret == ESP_OK) {
        ret = mcpwm_new_gpio_fault(&fault_config, &s_alt_fault);
    }

    const mcpwm_brake_config_t brake_config = {
        .fault = s_alt_fault,
        .brake_mode = MCPWM_OPER_BRAKE_MODE_OST,
    };
    if (ret == ESP_OK) {
        ret = mcpwm_operator_set_brake_on_fault(s_operator, &brake_config);
    }

    if (ret == ESP_OK) {
        portENTER_CRITICAL(&s_lock);
        s_initialized = true;
        s_force_low_held = true;
        s_ost_latched = false;
        s_output_enabled = false;
        s_permit = false;
        s_trip_latched = true;
        s_target_duty_percent = 0.0f;
        s_ramped_duty_percent = 0.0f;
        s_physical_duty_percent = 0.0f;
        s_compare_ticks = 0;
        s_last_trip_reason = TEC_PWM_TRIP_POWER_ON;
        s_last_trip_time_ms = tec_pwm_now_ms();
        memset(&s_last_trip_snapshot, 0, sizeof(s_last_trip_snapshot));
        s_sequence = 1;
        s_last_error = ESP_OK;
        s_alt_callback = alt_callback;
        s_alt_callback_ctx = user_ctx;
        portEXIT_CRITICAL(&s_lock);

        /* Check an already-active ALT before enabling or starting MCPWM. */
        if (gpio_get_level(INA_ALT_GPIO) == 0) {
            portENTER_CRITICAL(&s_lock);
            s_ost_latched = true;
            tec_pwm_latch_trip_locked(TEC_PWM_TRIP_ALT, tec_pwm_now_ms(), ESP_OK);
            portEXIT_CRITICAL(&s_lock);
        }
        ret = mcpwm_timer_enable(s_timer);
        s_timer_enabled = ret == ESP_OK;
    }
    if (ret == ESP_OK) {
        ret = mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP);
        s_timer_started = ret == ESP_OK;
    }

    if (ret != ESP_OK) {
        esp_err_t cleanup_ret = tec_pwm_release_resources();
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "initialization cleanup failed: %s", esp_err_to_name(cleanup_ret));
        }
        portENTER_CRITICAL(&s_lock);
        s_initialized = false;
        s_force_low_held = false;
        s_ost_latched = false;
        s_output_enabled = false;
        s_permit = false;
        s_trip_latched = false;
        s_target_duty_percent = 0.0f;
        s_ramped_duty_percent = 0.0f;
        s_physical_duty_percent = 0.0f;
        s_compare_ticks = 0;
        s_last_trip_reason = TEC_PWM_TRIP_NONE;
        s_last_trip_time_ms = 0;
        memset(&s_last_trip_snapshot, 0, sizeof(s_last_trip_snapshot));
        s_sequence++;
        s_last_error = ret;
        s_alt_callback = NULL;
        s_alt_callback_ctx = NULL;
        portEXIT_CRITICAL(&s_lock);
        return ret;
    }

    /* Close the ALT sampling-to-start race. The output is still force-low and
       unpermitted, so a newly active fault cannot produce a pulse here. */
    if (gpio_get_level(INA_ALT_GPIO) == 0) {
        portENTER_CRITICAL(&s_lock);
        s_ost_latched = true;
        tec_pwm_latch_trip_locked(TEC_PWM_TRIP_ALT, tec_pwm_now_ms(), ESP_OK);
        portEXIT_CRITICAL(&s_lock);
    }

    ESP_LOGI(TAG, "initialized: GPIO=%d, ALT=%d, group=%d, %u Hz, %u ticks",
             TEC_PWM_GPIO, INA_ALT_GPIO, TEC_PWM_MCPWM_GROUP_ID,
             TEC_PWM_FREQUENCY_HZ, TEC_PWM_PERIOD_TICKS);
    return ESP_OK;
}

esp_err_t tec_pwm_deinit(void)
{
    portENTER_CRITICAL(&s_lock);
    const bool initialized = s_initialized;
    portEXIT_CRITICAL(&s_lock);
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t first_error = tec_pwm_trip_from_task(TEC_PWM_TRIP_SOFTWARE_EMERGENCY, ESP_OK);

    portENTER_CRITICAL(&s_lock);
    s_initialized = false;
    s_alt_callback = NULL;
    s_alt_callback_ctx = NULL;
    portEXIT_CRITICAL(&s_lock);

    esp_err_t cleanup_ret = tec_pwm_release_resources();
    if (first_error == ESP_OK) {
        first_error = cleanup_ret;
    }

    portENTER_CRITICAL(&s_lock);
    s_force_low_held = false;
    s_ost_latched = false;
    s_output_enabled = false;
    s_permit = false;
    s_trip_latched = false;
    s_target_duty_percent = 0.0f;
    s_ramped_duty_percent = 0.0f;
    s_physical_duty_percent = 0.0f;
    s_compare_ticks = 0;
    s_last_trip_reason = TEC_PWM_TRIP_NONE;
    s_last_trip_time_ms = 0;
    memset(&s_last_trip_snapshot, 0, sizeof(s_last_trip_snapshot));
    s_sequence++;
    s_last_error = first_error;
    portEXIT_CRITICAL(&s_lock);
    return first_error;
}

esp_err_t tec_pwm_grant_permit(void)
{
    portENTER_CRITICAL(&s_lock);
    if (!s_initialized || s_trip_latched || s_permit ||
        s_target_duty_percent != 0.0f || s_ramped_duty_percent != 0.0f ||
        s_output_enabled) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    const bool force_low_held = s_force_low_held;
    portEXIT_CRITICAL(&s_lock);

    if (force_low_held) {
        esp_err_t ret = mcpwm_comparator_set_compare_value(s_comparator, 0);
        if (ret == ESP_OK) {
            ret = mcpwm_generator_set_force_level(s_generator, -1, true);
        }
        if (ret != ESP_OK) {
            (void)tec_pwm_trip_from_task(TEC_PWM_TRIP_DRIVER_ERROR, ret);
            return ret;
        }
    }

    portENTER_CRITICAL(&s_lock);
    if (!s_initialized || s_trip_latched || s_permit) {
        portEXIT_CRITICAL(&s_lock);
        if (force_low_held) {
            (void)mcpwm_generator_set_force_level(s_generator, 0, true);
        }
        return ESP_ERR_INVALID_STATE;
    }
    s_force_low_held = false;
    s_permit = true;
    s_last_error = ESP_OK;
    s_sequence++;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t tec_pwm_revoke_permit(void)
{
    portENTER_CRITICAL(&s_lock);
    if (!s_initialized || !s_permit || s_target_duty_percent != 0.0f ||
        s_ramped_duty_percent != 0.0f || s_output_enabled) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    s_permit = false;
    s_last_error = ESP_OK;
    s_sequence++;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t tec_pwm_set_target(float duty_percent)
{
    if (!isfinite(duty_percent) || duty_percent < 0.0f || duty_percent > 100.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    if (!s_initialized) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (duty_percent > 0.0f && (!s_permit || s_trip_latched)) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    s_target_duty_percent = duty_percent;
    s_last_error = ESP_OK;
    s_sequence++;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t tec_pwm_request_stop(void)
{
    return tec_pwm_set_target(0.0f);
}

esp_err_t tec_pwm_update_10ms(void)
{
    float target;
    float current;
    bool output_enabled;

    portENTER_CRITICAL(&s_lock);
    if (!s_initialized) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_trip_latched || !s_permit) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_OK;
    }
    target = s_target_duty_percent;
    current = s_ramped_duty_percent;
    output_enabled = s_output_enabled;
    portEXIT_CRITICAL(&s_lock);

    float next = current;
    if (target > current) {
        next = fminf(target, current + TEC_PWM_RAMP_STEP_PERCENT);
    } else if (target < current) {
        next = fmaxf(target, current - TEC_PWM_RAMP_STEP_PERCENT);
    }

    bool next_output_enabled = output_enabled;
    if (!output_enabled && next >= TEC_PWM_ENABLE_PERCENT) {
        next_output_enabled = true;
    } else if (output_enabled && next <= TEC_PWM_DISABLE_PERCENT) {
        next_output_enabled = false;
    }

    const uint32_t next_compare = next_output_enabled ? tec_pwm_duty_to_ticks(next) : 0;
    esp_err_t ret = mcpwm_comparator_set_compare_value(s_comparator, next_compare);
    if (ret != ESP_OK) {
        (void)tec_pwm_trip_from_task(TEC_PWM_TRIP_DRIVER_ERROR, ret);
        return ret;
    }

    portENTER_CRITICAL(&s_lock);
    if (!s_trip_latched && s_permit) {
        s_ramped_duty_percent = next;
        s_output_enabled = next_output_enabled;
        s_compare_ticks = next_compare;
        s_physical_duty_percent = next_output_enabled ?
            tec_pwm_ticks_to_duty(next_compare) : 0.0f;
        s_last_error = ESP_OK;
        s_sequence++;
    }
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t tec_pwm_emergency_shutdown(tec_pwm_trip_reason_t reason)
{
    if (reason == TEC_PWM_TRIP_NONE || reason == TEC_PWM_TRIP_POWER_ON ||
        reason == TEC_PWM_TRIP_ALT || reason == TEC_PWM_TRIP_DRIVER_ERROR) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    const bool initialized = s_initialized;
    portEXIT_CRITICAL(&s_lock);
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return tec_pwm_trip_from_task(reason, ESP_OK);
}

esp_err_t tec_pwm_recover(void)
{
    bool had_ost_latch;

    portENTER_CRITICAL(&s_lock);
    if (!s_initialized || !s_trip_latched || s_permit) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    had_ost_latch = s_ost_latched;
    s_ost_latched = false;
    portEXIT_CRITICAL(&s_lock);

    if (gpio_get_level(INA_ALT_GPIO) == 0) {
        portENTER_CRITICAL(&s_lock);
        s_ost_latched = had_ost_latch;
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = mcpwm_generator_set_force_level(s_generator, 0, true);
    if (ret == ESP_OK) {
        ret = mcpwm_comparator_set_compare_value(s_comparator, 0);
    }
    if (ret == ESP_OK && had_ost_latch) {
        ret = mcpwm_operator_recover_from_fault(s_operator, s_alt_fault);
    }
    if (ret != ESP_OK) {
        portENTER_CRITICAL(&s_lock);
        s_ost_latched = s_ost_latched || had_ost_latch;
        s_last_error = ret;
        portEXIT_CRITICAL(&s_lock);
        return ret;
    }

    if (gpio_get_level(INA_ALT_GPIO) == 0) {
        portENTER_CRITICAL(&s_lock);
        s_ost_latched = true;
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&s_lock);
    if (s_ost_latched) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    s_force_low_held = true;
    s_output_enabled = false;
    s_permit = false;
    s_trip_latched = false;
    s_target_duty_percent = 0.0f;
    s_ramped_duty_percent = 0.0f;
    s_physical_duty_percent = 0.0f;
    s_compare_ticks = 0;
    s_last_error = ESP_OK;
    s_sequence++;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t tec_pwm_get_status(tec_pwm_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    *status = (tec_pwm_status_t) {
        .state = tec_pwm_state_locked(),
        .target_duty_percent = s_target_duty_percent,
        .ramped_duty_percent = s_ramped_duty_percent,
        .physical_duty_percent = s_physical_duty_percent,
        .compare_ticks = s_compare_ticks,
        .output_enabled = s_output_enabled,
        .permit = s_permit,
        .trip_latched = s_trip_latched,
        .last_trip_reason = s_last_trip_reason,
        .last_trip_time_ms = s_last_trip_time_ms,
        .last_trip_snapshot = s_last_trip_snapshot,
        .sequence = s_sequence,
        .last_error = s_last_error,
    };
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}
