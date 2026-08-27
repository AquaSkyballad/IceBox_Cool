#include "control_algorithm.h"

#include <math.h>
#include <string.h>

#include "config.h"

typedef struct {
    bool initialized;
    bool configured;
    bool enabled;
    control_algorithm_config_t config;
    float temperature_integral;
    float current_integral;
    float filtered_current_a;
    float feedforward_current_a;
    float air_rate_c_per_s;
    float power_wall_limit_a;
    float requested_current_a;
    float current_ref_a;
    float duty_request_percent;
    float temperature_error_c;
    float temperature_wall_limit_a;
    uint8_t fan1_speed_percent;
    uint32_t last_temperature_sequence;
    uint32_t last_temperature_exec_ms;
    uint32_t last_temperature_timestamp_ms;
    uint32_t last_air_sequence;
    uint32_t last_air_timestamp_ms;
    float last_air_temperature_c;
    uint32_t last_current_sequence;
    uint32_t last_current_timestamp_ms;
    uint32_t last_fan_update_ms;
    uint32_t last_step_ms;
    bool have_power_wall_limit;
    bool have_filtered_current;
    bool have_air_sample;
    control_output_t status;
} control_state_internal_t;

static control_state_internal_t s_control;

static float clampf_local(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static bool finite_range(float value, float low, float high)
{
    return isfinite(value) && value >= low && value <= high;
}

static bool sample_is_fresh(uint32_t now_ms, uint32_t timestamp_ms, uint32_t max_age_ms)
{
    if (timestamp_ms == 0U) {
        return false;
    }
    return (uint32_t)(now_ms - timestamp_ms) <= max_age_ms;
}

static void reset_runtime(void)
{
    s_control.temperature_integral = 0.0f;
    s_control.current_integral = 0.0f;
    s_control.filtered_current_a = 0.0f;
    s_control.feedforward_current_a = 0.0f;
    s_control.air_rate_c_per_s = 0.0f;
    s_control.power_wall_limit_a = 0.0f;
    s_control.requested_current_a = 0.0f;
    s_control.current_ref_a = 0.0f;
    s_control.duty_request_percent = 0.0f;
    s_control.temperature_error_c = 0.0f;
    s_control.temperature_wall_limit_a = 0.0f;
    s_control.fan1_speed_percent = 0U;
    s_control.last_temperature_sequence = 0U;
    s_control.last_temperature_exec_ms = 0U;
    s_control.last_temperature_timestamp_ms = 0U;
    s_control.last_air_sequence = 0U;
    s_control.last_air_timestamp_ms = 0U;
    s_control.last_air_temperature_c = 0.0f;
    s_control.last_current_sequence = 0U;
    s_control.last_current_timestamp_ms = 0U;
    s_control.last_fan_update_ms = 0U;
    s_control.last_step_ms = 0U;
    s_control.have_power_wall_limit = false;
    s_control.have_filtered_current = false;
    s_control.have_air_sample = false;
}

esp_err_t control_algorithm_get_default_config(control_algorithm_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *config = (control_algorithm_config_t) {
        .version = CONTROL_ALGORITHM_CONFIG_VERSION,
        .temperature = {
            .feedback_channel = CONTROL_TEMPERATURE_COLD,
            .kp = PID_KP_DEFAULT,
            .ki = PID_KI_DEFAULT,
            .kd = PID_KD_DEFAULT,
            .integral_min = CONTROL_TEMP_INTEGRAL_MIN_DEFAULT,
            .integral_max = CONTROL_TEMP_INTEGRAL_MAX_DEFAULT,
            .output_min_current_a = CONTROL_TEMP_OUTPUT_MIN_CURRENT_DEFAULT,
            .output_max_current_a = CONTROL_TEMP_OUTPUT_MAX_CURRENT_DEFAULT,
            .anti_windup_gain = CONTROL_TEMP_ANTI_WINDUP_DEFAULT,
        },
        /* Feed-forward is intentionally off until the air-sensor dynamics
         * have been measured.  The remaining fields stay valid neutral values
         * so the complete configuration can be persisted and inspected. */
        .feedforward = {
            .enabled = false,
            .air_rate_deadband_c_per_s = 0.20f,
            .rate_filter_time_constant_s = 2.0f,
            .kff_rate_a_per_c_per_s = 0.0f,
            .ff_min_current_a = 0.0f,
            .ff_max_current_a = 0.0f,
            .rise_rate_a_per_s = 2.0f,
            .fall_rate_a_per_s = 2.0f,
            .decay_timeout_s = 2.0f,
        },
        .current = {
            .kp = CONTROL_CURRENT_KP_DEFAULT,
            .ki = CONTROL_CURRENT_KI_DEFAULT,
            .integral_min = CONTROL_CURRENT_INTEGRAL_MIN_DEFAULT,
            .integral_max = CONTROL_CURRENT_INTEGRAL_MAX_DEFAULT,
            .duty_min_percent = CONTROL_CURRENT_DUTY_MIN_DEFAULT,
            .duty_max_percent = CONTROL_CURRENT_DUTY_MAX_DEFAULT,
            .error_deadband_a = CONTROL_CURRENT_ERROR_DEADBAND_DEFAULT,
            .anti_windup_gain = CONTROL_CURRENT_ANTI_WINDUP_DEFAULT,
            .feedback_filter_time_constant_s = CONTROL_CURRENT_FILTER_TAU_DEFAULT,
        },
        .soft_limits = {
            .normal_current_limit_a = TEC_CURRENT_SOFTWARE_MAX_A,
            .user_current_limit_a = TEC_CURRENT_SOFTWARE_MAX_A,
            .hot_wall_start_c = 45.0f,
            .hot_wall_max_c = 60.0f,
            .tec_power_limit_w = 60.0f,
            .power_wall_hysteresis_w = 2.0f,
            .power_wall_attack_rate_a_per_s = 2.0f,
            .power_wall_release_rate_a_per_s = 2.0f,
            .fan_start_temp_c = 35.0f,
            .fan_full_speed_temp_c = 50.0f,
            .fan_min_speed_percent = 30U,
            .fan_max_speed_percent = 100U,
        },
        .timing = {
            .temperature_period_ms = 1000U,
            .fan_period_ms = 500U,
            .max_snapshot_age_ms = 120U,
            .mode_transition_policy = CONTROL_MODE_RESET_INTEGRATORS,
        },
    };
    return control_algorithm_config_is_valid(config) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

bool control_algorithm_config_is_valid(const control_algorithm_config_t *config)
{
    if (config == NULL || config->version != CONTROL_ALGORITHM_CONFIG_VERSION) {
        return false;
    }
    if (config->temperature.feedback_channel != CONTROL_TEMPERATURE_COLD &&
        config->temperature.feedback_channel != CONTROL_TEMPERATURE_AIR) {
        return false;
    }
    if (!finite_range(config->temperature.kp, 0.0f, 1000.0f) ||
        !finite_range(config->temperature.ki, 0.0f, 1000.0f) ||
        !finite_range(config->temperature.kd, 0.0f, 1000.0f) ||
        !finite_range(config->temperature.integral_min, -1000.0f, 1000.0f) ||
        !finite_range(config->temperature.integral_max, -1000.0f, 1000.0f) ||
        config->temperature.integral_min > config->temperature.integral_max ||
        !finite_range(config->temperature.output_min_current_a, 0.0f,
                      TEC_CURRENT_SOFTWARE_MAX_A) ||
        !finite_range(config->temperature.output_max_current_a, 0.0f,
                      TEC_CURRENT_SOFTWARE_MAX_A) ||
        config->temperature.output_min_current_a > config->temperature.output_max_current_a ||
        !finite_range(config->temperature.anti_windup_gain, 0.0f, 1000.0f)) {
        return false;
    }
    if (!finite_range(config->feedforward.air_rate_deadband_c_per_s, 0.0f, 1000.0f) ||
        !finite_range(config->feedforward.rate_filter_time_constant_s, 0.001f, 10000.0f) ||
        !finite_range(config->feedforward.kff_rate_a_per_c_per_s, 0.0f, 1000.0f) ||
        !finite_range(config->feedforward.ff_min_current_a, 0.0f,
                      TEC_CURRENT_SOFTWARE_MAX_A) ||
        !finite_range(config->feedforward.ff_max_current_a, 0.0f,
                      TEC_CURRENT_SOFTWARE_MAX_A) ||
        config->feedforward.ff_min_current_a > config->feedforward.ff_max_current_a ||
        !finite_range(config->feedforward.rise_rate_a_per_s, 0.0f, 1000.0f) ||
        !finite_range(config->feedforward.fall_rate_a_per_s, 0.0f, 1000.0f) ||
        !finite_range(config->feedforward.decay_timeout_s, 0.0f, 10000.0f)) {
        return false;
    }
    if (!finite_range(config->current.kp, 0.0f, 1000.0f) ||
        !finite_range(config->current.ki, 0.0f, 1000.0f) ||
        !finite_range(config->current.integral_min, -1000.0f, 1000.0f) ||
        !finite_range(config->current.integral_max, -1000.0f, 1000.0f) ||
        config->current.integral_min > config->current.integral_max ||
        !finite_range(config->current.duty_min_percent, 0.0f, 100.0f) ||
        !finite_range(config->current.duty_max_percent, 0.0f, 100.0f) ||
        config->current.duty_min_percent > config->current.duty_max_percent ||
        !finite_range(config->current.error_deadband_a, 0.0f,
                      TEC_CURRENT_SOFTWARE_MAX_A) ||
        !finite_range(config->current.anti_windup_gain, 0.0f, 1000.0f) ||
        !finite_range(config->current.feedback_filter_time_constant_s, 0.001f, 10000.0f)) {
        return false;
    }
    if (!finite_range(config->soft_limits.normal_current_limit_a, 0.0f,
                      TEC_CURRENT_SOFTWARE_MAX_A) ||
        !finite_range(config->soft_limits.user_current_limit_a, 0.0f,
                      TEC_CURRENT_SOFTWARE_MAX_A) ||
        !finite_range(config->soft_limits.hot_wall_start_c, -50.0f, 200.0f) ||
        !finite_range(config->soft_limits.hot_wall_max_c, -50.0f, 250.0f) ||
        config->soft_limits.hot_wall_start_c >= config->soft_limits.hot_wall_max_c ||
        !finite_range(config->soft_limits.tec_power_limit_w, 0.001f, 1000.0f) ||
        !finite_range(config->soft_limits.power_wall_hysteresis_w, 0.0f, 1000.0f) ||
        !finite_range(config->soft_limits.power_wall_attack_rate_a_per_s, 0.0f, 1000.0f) ||
        !finite_range(config->soft_limits.power_wall_release_rate_a_per_s, 0.0f, 1000.0f) ||
        !finite_range(config->soft_limits.fan_start_temp_c, -50.0f, 200.0f) ||
        !finite_range(config->soft_limits.fan_full_speed_temp_c, -50.0f, 250.0f) ||
        config->soft_limits.fan_start_temp_c >= config->soft_limits.fan_full_speed_temp_c ||
        config->soft_limits.fan_min_speed_percent > config->soft_limits.fan_max_speed_percent ||
        config->soft_limits.fan_max_speed_percent > 100U ||
        config->timing.temperature_period_ms == 0U ||
        config->timing.fan_period_ms == 0U ||
        config->timing.max_snapshot_age_ms == 0U ||
        (config->timing.mode_transition_policy != CONTROL_MODE_RESET_INTEGRATORS &&
         config->timing.mode_transition_policy != CONTROL_MODE_FREEZE_INTEGRATORS)) {
        return false;
    }
    return true;
}

static float update_feedforward(const control_input_snapshot_t *input, float dt_s,
                                bool *active, bool *disturbance)
{
    const control_feedforward_config_t *cfg = &s_control.config.feedforward;
    *active = false;
    *disturbance = false;
    if (input->disturbance_hint != CONTROL_DISTURBANCE_NONE) {
        *disturbance = true;
    }
    if (!cfg->enabled) {
        s_control.feedforward_current_a = 0.0f;
        s_control.air_rate_c_per_s = 0.0f;
        return 0.0f;
    }

    bool new_air = input->adc.sequence != 0U &&
                   input->adc.sequence != s_control.last_air_sequence;
    float desired = 0.0f;
    if (new_air && input->adc.air_valid &&
        sample_is_fresh(input->now_ms, input->adc.sample_time_ms,
                        s_control.config.timing.max_snapshot_age_ms)) {
        const uint32_t timestamp = input->adc.sample_time_ms;
        if (s_control.have_air_sample && timestamp != s_control.last_air_timestamp_ms) {
            const uint32_t elapsed_ms = (uint32_t)(timestamp - s_control.last_air_timestamp_ms);
            if (elapsed_ms > 0U && elapsed_ms <= s_control.config.timing.max_snapshot_age_ms) {
                const float elapsed_s = (float)elapsed_ms / 1000.0f;
                const float raw_rate = (input->adc.air_temp_c - s_control.last_air_temperature_c) /
                                       elapsed_s;
                const float tau = cfg->rate_filter_time_constant_s;
                const float alpha = clampf_local(elapsed_s / (tau + elapsed_s), 0.0f, 1.0f);
                s_control.air_rate_c_per_s += alpha * (raw_rate - s_control.air_rate_c_per_s);
            }
        }
        s_control.last_air_sequence = input->adc.sequence;
        s_control.last_air_timestamp_ms = timestamp;
        s_control.last_air_temperature_c = input->adc.air_temp_c;
        s_control.have_air_sample = true;
        const float disturbance_rate = fmaxf(0.0f,
                                             s_control.air_rate_c_per_s -
                                             cfg->air_rate_deadband_c_per_s);
        if (disturbance_rate > 0.0f) {
            *disturbance = true;
            desired = clampf_local(cfg->kff_rate_a_per_c_per_s * disturbance_rate,
                                   cfg->ff_min_current_a, cfg->ff_max_current_a);
        }
    } else {
        s_control.air_rate_c_per_s = 0.0f;
    }

    const float elapsed = dt_s > 0.0f ? dt_s : 0.0f;
    const float rate = desired > s_control.feedforward_current_a ?
                       cfg->rise_rate_a_per_s : cfg->fall_rate_a_per_s;
    const float delta = rate * elapsed;
    if (desired > s_control.feedforward_current_a) {
        s_control.feedforward_current_a = fminf(desired, s_control.feedforward_current_a + delta);
    } else {
        s_control.feedforward_current_a = fmaxf(desired, s_control.feedforward_current_a - delta);
    }
    s_control.feedforward_current_a = clampf_local(s_control.feedforward_current_a,
                                                   0.0f, cfg->ff_max_current_a);
    *active = s_control.feedforward_current_a > 0.0f;
    return s_control.feedforward_current_a;
}

static float calculate_temperature_wall(const control_input_snapshot_t *input, uint32_t *flags)
{
    const control_soft_limits_config_t *cfg = &s_control.config.soft_limits;
    if (!input->adc.hot_valid ||
        !sample_is_fresh(input->now_ms, input->adc.sample_time_ms,
                         s_control.config.timing.max_snapshot_age_ms)) {
        *flags |= CONTROL_FLAG_INPUT_INVALID;
        return 0.0f;
    }
    if (input->adc.hot_temp_c <= cfg->hot_wall_start_c) {
        return fminf(cfg->normal_current_limit_a, cfg->user_current_limit_a);
    }
    *flags |= CONTROL_FLAG_TEMP_WALL_ACTIVE;
    if (input->adc.hot_temp_c >= cfg->hot_wall_max_c) {
        return 0.0f;
    }
    const float fraction = (cfg->hot_wall_max_c - input->adc.hot_temp_c) /
                           (cfg->hot_wall_max_c - cfg->hot_wall_start_c);
    return fminf(cfg->normal_current_limit_a, cfg->user_current_limit_a) * fraction;
}

static float calculate_power_wall(const control_input_snapshot_t *input, uint32_t *flags,
                                  float dt_s)
{
    const control_soft_limits_config_t *cfg = &s_control.config.soft_limits;
    float raw_limit = 0.0f;
    const float tec_voltage_v = input->ina.bus_voltage_v - input->adc.tec_n_voltage_v;
    if (input->ina.valid && input->adc.tec_n_valid &&
        isfinite(tec_voltage_v) && tec_voltage_v > 0.1f &&
        sample_is_fresh(input->now_ms, input->ina.timestamp_ms,
                        s_control.config.timing.max_snapshot_age_ms) &&
        sample_is_fresh(input->now_ms, input->adc.sample_time_ms,
                        s_control.config.timing.max_snapshot_age_ms)) {
        raw_limit = clampf_local(cfg->tec_power_limit_w / tec_voltage_v,
                                 0.0f, fminf(cfg->normal_current_limit_a,
                                             cfg->user_current_limit_a));
    } else {
        *flags |= CONTROL_FLAG_POWER_INPUT_INVALID | CONTROL_FLAG_INPUT_INVALID;
    }

    if (!s_control.have_power_wall_limit) {
        s_control.power_wall_limit_a = raw_limit;
        s_control.have_power_wall_limit = true;
    } else if (raw_limit < s_control.power_wall_limit_a) {
        const float step = cfg->power_wall_attack_rate_a_per_s * fmaxf(dt_s, 0.0f);
        s_control.power_wall_limit_a = fmaxf(raw_limit, s_control.power_wall_limit_a - step);
    } else if (raw_limit > s_control.power_wall_limit_a +
                              cfg->power_wall_hysteresis_w /
                              fmaxf(tec_voltage_v, 0.1f)) {
        const float step = cfg->power_wall_release_rate_a_per_s * fmaxf(dt_s, 0.0f);
        s_control.power_wall_limit_a = fminf(raw_limit, s_control.power_wall_limit_a + step);
    }
    if (s_control.power_wall_limit_a < fminf(cfg->normal_current_limit_a,
                                             cfg->user_current_limit_a) - 1e-4f) {
        *flags |= CONTROL_FLAG_POWER_WALL_ACTIVE;
    }
    return s_control.power_wall_limit_a;
}

static void update_fan(const control_input_snapshot_t *input, uint32_t *flags)
{
    if (!input->adc.hot_valid ||
        !sample_is_fresh(input->now_ms, input->adc.sample_time_ms,
                         s_control.config.timing.max_snapshot_age_ms)) {
        s_control.fan1_speed_percent = s_control.config.soft_limits.fan_max_speed_percent;
        *flags |= CONTROL_FLAG_INPUT_INVALID;
        return;
    }
    if (s_control.last_fan_update_ms != 0U &&
        (uint32_t)(input->now_ms - s_control.last_fan_update_ms) <
        s_control.config.timing.fan_period_ms) {
        return;
    }
    s_control.last_fan_update_ms = input->now_ms;
    const control_soft_limits_config_t *cfg = &s_control.config.soft_limits;
    float fraction = (input->adc.hot_temp_c - cfg->fan_start_temp_c) /
                     (cfg->fan_full_speed_temp_c - cfg->fan_start_temp_c);
    fraction = clampf_local(fraction, 0.0f, 1.0f);
    const float speed = (float)cfg->fan_min_speed_percent +
                        fraction * (float)(cfg->fan_max_speed_percent -
                                           cfg->fan_min_speed_percent);
    s_control.fan1_speed_percent = (uint8_t)lroundf(speed);
}

esp_err_t control_algorithm_init(void)
{
    if (s_control.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(&s_control, 0, sizeof(s_control));
    s_control.initialized = true;
    s_control.status.state = CONTROL_STATE_DISABLED;
    return ESP_OK;
}

esp_err_t control_algorithm_init_with_config(const control_algorithm_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!control_algorithm_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = control_algorithm_init();
    if (ret != ESP_OK) {
        return ret;
    }
    return control_algorithm_configure(config);
}

esp_err_t control_algorithm_configure(const control_algorithm_config_t *config)
{
    if (!s_control.initialized || config == NULL) {
        return config == NULL ? ESP_ERR_INVALID_ARG : ESP_ERR_INVALID_STATE;
    }
    if (!control_algorithm_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    s_control.config = *config;
    s_control.configured = true;
    reset_runtime();
    s_control.status.state = s_control.enabled ? CONTROL_STATE_READY : CONTROL_STATE_DISABLED;
    return ESP_OK;
}

esp_err_t control_algorithm_enable(void)
{
    if (!s_control.initialized || !s_control.configured) {
        return ESP_ERR_INVALID_STATE;
    }
    s_control.enabled = true;
    s_control.status.state = CONTROL_STATE_READY;
    return ESP_OK;
}

esp_err_t control_algorithm_disable(control_disable_reason_t reason)
{
    if (!s_control.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (reason < CONTROL_DISABLE_REQUESTED || reason > CONTROL_DISABLE_MANUAL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_control.enabled = false;
    if (s_control.configured &&
        s_control.config.timing.mode_transition_policy == CONTROL_MODE_RESET_INTEGRATORS) {
        reset_runtime();
    }
    s_control.status.state = CONTROL_STATE_DISABLED;
    return ESP_OK;
}

esp_err_t control_algorithm_reset_integrators(control_reset_reason_t reason)
{
    if (!s_control.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (reason < CONTROL_RESET_REQUESTED || reason > CONTROL_RESET_SAFETY) {
        return ESP_ERR_INVALID_ARG;
    }
    s_control.temperature_integral = 0.0f;
    s_control.current_integral = 0.0f;
    return ESP_OK;
}

esp_err_t control_algorithm_step(const control_input_snapshot_t *input,
                                 const control_actuator_snapshot_t *actuator,
                                 control_output_t *output)
{
    if (input == NULL || actuator == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_control.initialized || !s_control.configured) {
        memset(output, 0, sizeof(*output));
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t flags = CONTROL_FLAG_NONE;
    const bool permitted = s_control.enabled && actuator->permit && !actuator->trip_latched;
    const float step_dt_s = s_control.last_step_ms == 0U ? 0.0f :
                            (float)(uint32_t)(input->now_ms - s_control.last_step_ms) / 1000.0f;
    s_control.last_step_ms = input->now_ms;

    if (!permitted || !input->temperature_setpoint_valid) {
        if (actuator->trip_latched ||
            (s_control.enabled && !actuator->permit) ||
            (!s_control.enabled && s_control.config.timing.mode_transition_policy ==
             CONTROL_MODE_RESET_INTEGRATORS)) {
            reset_runtime();
        }
        s_control.duty_request_percent = 0.0f;
        s_control.current_ref_a = 0.0f;
        flags |= CONTROL_FLAG_INPUT_INVALID;
        s_control.status = (control_output_t) {
            .ramped_duty_percent = actuator->ramped_duty_percent,
            .physical_duty_percent = actuator->physical_duty_percent,
            .output_enabled = actuator->output_enabled,
            .fan1_speed_percent = s_control.fan1_speed_percent,
            .adc_sequence = input->adc.sequence,
            .adc_sample_time_ms = input->adc.sample_time_ms,
            .ina_sequence = input->ina.sequence,
            .ina_timestamp_ms = input->ina.timestamp_ms,
            .temperature_valid = false,
            .current_valid = false,
            .flags = flags,
            .state = s_control.enabled ? CONTROL_STATE_READY : CONTROL_STATE_DISABLED,
            .sequence = s_control.status.sequence + 1U,
        };
        *output = s_control.status;
        return ESP_OK;
    }

    const bool new_temperature = input->adc.sequence != 0U &&
                                 input->adc.sequence != s_control.last_temperature_sequence;
    const bool feedback_valid = s_control.config.temperature.feedback_channel ==
                                CONTROL_TEMPERATURE_COLD ? input->adc.cold_valid :
                                input->adc.air_valid;
    const bool temp_fresh = feedback_valid &&
                            sample_is_fresh(input->now_ms, input->adc.sample_time_ms,
                                            s_control.config.timing.max_snapshot_age_ms);
    if (!temp_fresh) {
        flags |= CONTROL_FLAG_INPUT_INVALID;
        /* A stale/invalid temperature input must not leave the previous duty
           request active while waiting for the next INA226 sample. */
        s_control.duty_request_percent = 0.0f;
        s_control.current_integral = 0.0f;
    }
    const bool temperature_due = new_temperature && temp_fresh &&
                                 (s_control.last_temperature_exec_ms == 0U ||
                                  (uint32_t)(input->now_ms - s_control.last_temperature_exec_ms) >=
                                  s_control.config.timing.temperature_period_ms);
    float temp_dt_s = 0.0f;
    bool ff_active = false;
    bool disturbance = false;
    if (temperature_due) {
        if (s_control.last_temperature_timestamp_ms != 0U &&
            input->adc.sample_time_ms > s_control.last_temperature_timestamp_ms) {
            temp_dt_s = (float)(input->adc.sample_time_ms -
                                s_control.last_temperature_timestamp_ms) / 1000.0f;
        }
        const float feedback = s_control.config.temperature.feedback_channel ==
                               CONTROL_TEMPERATURE_COLD ? input->adc.cold_temp_c :
                               input->adc.air_temp_c;
        const float setpoint_error = feedback - input->temperature_setpoint_c;
        float derivative = 0.0f;
        if (temp_dt_s > 0.0f && s_control.last_temperature_exec_ms != 0U) {
            derivative = (setpoint_error - s_control.temperature_error_c) / temp_dt_s;
        }
        const float ff = update_feedforward(input, temp_dt_s, &ff_active, &disturbance);
        if (temp_dt_s > 0.0f) {
            s_control.temperature_integral = clampf_local(
                s_control.temperature_integral +
                s_control.config.temperature.ki * setpoint_error * temp_dt_s,
                s_control.config.temperature.integral_min,
                s_control.config.temperature.integral_max);
        }
        s_control.temperature_error_c = setpoint_error;
        s_control.requested_current_a = clampf_local(
            s_control.config.temperature.kp * setpoint_error +
            s_control.temperature_integral +
            s_control.config.temperature.kd * derivative + ff,
            s_control.config.temperature.output_min_current_a,
            s_control.config.temperature.output_max_current_a);
        s_control.last_temperature_sequence = input->adc.sequence;
        s_control.last_temperature_exec_ms = input->now_ms;
        s_control.last_temperature_timestamp_ms = input->adc.sample_time_ms;
        flags |= CONTROL_FLAG_TEMPERATURE_UPDATED;
    } else if (new_temperature) {
        s_control.last_temperature_sequence = input->adc.sequence;
    }

    const float temp_limit = calculate_temperature_wall(input, &flags);
    s_control.temperature_wall_limit_a = temp_limit;
    const float power_limit = calculate_power_wall(input, &flags, step_dt_s);
    const float limit = fminf(temp_limit, power_limit);
    s_control.current_ref_a = temp_fresh ?
        clampf_local(s_control.requested_current_a, 0.0f, limit) : 0.0f;
    if (temperature_due && temp_dt_s > 0.0f) {
        const float correction = s_control.config.temperature.anti_windup_gain *
                                (s_control.current_ref_a - s_control.requested_current_a) * temp_dt_s;
        s_control.temperature_integral = clampf_local(
            s_control.temperature_integral + correction,
            s_control.config.temperature.integral_min,
            s_control.config.temperature.integral_max);
        if (fabsf(correction) > 1e-6f) {
            flags |= CONTROL_FLAG_INTEGRAL_FROZEN;
        }
    }

    const bool new_current = input->ina.sequence != 0U &&
                             input->ina.sequence != s_control.last_current_sequence;
    const bool current_fresh = input->ina.valid &&
                               sample_is_fresh(input->now_ms, input->ina.timestamp_ms,
                                               s_control.config.timing.max_snapshot_age_ms);
    if (!current_fresh) {
        flags |= CONTROL_FLAG_INPUT_INVALID;
        s_control.duty_request_percent = 0.0f;
        s_control.current_integral = 0.0f;
    } else if (new_current) {
        float current_dt_s = 0.0f;
        if (s_control.last_current_timestamp_ms != 0U &&
            input->ina.timestamp_ms > s_control.last_current_timestamp_ms) {
            current_dt_s = (float)(input->ina.timestamp_ms -
                                   s_control.last_current_timestamp_ms) / 1000.0f;
        }
        const float tau = s_control.config.current.feedback_filter_time_constant_s;
        if (!s_control.have_filtered_current || current_dt_s <= 0.0f) {
            s_control.filtered_current_a = input->ina.current_a;
            s_control.have_filtered_current = true;
        } else {
            const float alpha = clampf_local(current_dt_s / (tau + current_dt_s), 0.0f, 1.0f);
            s_control.filtered_current_a += alpha *
                                           (input->ina.current_a - s_control.filtered_current_a);
        }
        float error = s_control.current_ref_a - s_control.filtered_current_a;
        if (fabsf(error) <= s_control.config.current.error_deadband_a) {
            error = 0.0f;
        }
        if (current_dt_s > 0.0f) {
            s_control.current_integral = clampf_local(
                s_control.current_integral + s_control.config.current.ki * error * current_dt_s,
                s_control.config.current.integral_min,
                s_control.config.current.integral_max);
        }
        const float raw_duty = s_control.config.current.kp * error + s_control.current_integral;
        s_control.duty_request_percent = s_control.current_ref_a <= 0.0f ? 0.0f :
            clampf_local(raw_duty,
                         s_control.config.current.duty_min_percent,
                         s_control.config.current.duty_max_percent);
        if (current_dt_s > 0.0f) {
            const float correction = s_control.config.current.anti_windup_gain *
                                     (s_control.duty_request_percent - raw_duty) * current_dt_s;
            s_control.current_integral = clampf_local(
                s_control.current_integral + correction,
                s_control.config.current.integral_min,
                s_control.config.current.integral_max);
            if (fabsf(correction) > 1e-6f) {
                flags |= CONTROL_FLAG_INTEGRAL_FROZEN;
            }
        }
        s_control.last_current_sequence = input->ina.sequence;
        s_control.last_current_timestamp_ms = input->ina.timestamp_ms;
        flags |= CONTROL_FLAG_CURRENT_UPDATED;
    }

    /* Keep the temperature-input fail-safe dominant even when the current
       snapshot is fresh but has not advanced in this invocation. */
    if (!temp_fresh) {
        s_control.duty_request_percent = 0.0f;
        s_control.current_integral = 0.0f;
    }

    const float capacity_threshold = fmaxf(
        0.0f, s_control.config.current.duty_max_percent - 0.2f);
    if (s_control.current_ref_a > 0.0f &&
        (actuator->ramped_duty_percent >= capacity_threshold ||
         actuator->physical_duty_percent >= capacity_threshold) &&
        s_control.filtered_current_a + s_control.config.current.error_deadband_a <
        s_control.current_ref_a) {
        flags |= CONTROL_FLAG_CAPACITY_LIMITED;
    }
    update_fan(input, &flags);
    if (ff_active) {
        flags |= CONTROL_FLAG_FEEDFORWARD_ACTIVE;
    }
    if (disturbance) {
        flags |= CONTROL_FLAG_DISTURBANCE_DETECTED;
    }

    control_state_t state = s_control.duty_request_percent > 0.0f ?
                            CONTROL_STATE_RUNNING : CONTROL_STATE_READY;
    s_control.status = (control_output_t) {
        .requested_current_a = s_control.requested_current_a,
        .temperature_pi_current_a = clampf_local(
            s_control.config.temperature.kp * s_control.temperature_error_c +
            s_control.temperature_integral,
            s_control.config.temperature.output_min_current_a,
            s_control.config.temperature.output_max_current_a),
        .feedforward_current_a = s_control.feedforward_current_a,
        .current_ref_a = s_control.current_ref_a,
        .filtered_current_a = s_control.filtered_current_a,
        .duty_request_percent = s_control.duty_request_percent,
        .ramped_duty_percent = actuator->ramped_duty_percent,
        .physical_duty_percent = actuator->physical_duty_percent,
        .output_enabled = actuator->output_enabled,
        .fan1_speed_percent = s_control.fan1_speed_percent,
        .temperature_error_c = s_control.temperature_error_c,
        .air_temperature_rate_c_per_s = s_control.air_rate_c_per_s,
        .temperature_wall_current_limit_a = s_control.temperature_wall_limit_a,
        .power_wall_current_limit_a = s_control.power_wall_limit_a,
        .adc_sequence = input->adc.sequence,
        .adc_sample_time_ms = input->adc.sample_time_ms,
        .ina_sequence = input->ina.sequence,
        .ina_timestamp_ms = input->ina.timestamp_ms,
        .temperature_valid = temp_fresh,
        .current_valid = current_fresh,
        .flags = flags,
        .state = state,
        .sequence = s_control.status.sequence + 1U,
    };
    *output = s_control.status;
    return ESP_OK;
}

esp_err_t control_algorithm_get_status(control_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_control.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    *status = s_control.status;
    return ESP_OK;
}
