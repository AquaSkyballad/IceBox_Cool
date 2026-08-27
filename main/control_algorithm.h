#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "adc_sensors.h"
#include "esp_err.h"
#include "ina226.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_ALGORITHM_CONFIG_VERSION 1U

typedef enum {
    CONTROL_TEMPERATURE_COLD = 0,
    CONTROL_TEMPERATURE_AIR,
} control_temperature_channel_t;

typedef enum {
    CONTROL_DISTURBANCE_NONE = 0,
    CONTROL_DISTURBANCE_DOOR_OPEN,
    CONTROL_DISTURBANCE_LOAD_INSERTED,
} control_disturbance_hint_t;

typedef enum {
    CONTROL_DISABLE_REQUESTED = 0,
    CONTROL_DISABLE_SAFETY,
    CONTROL_DISABLE_MANUAL,
} control_disable_reason_t;

typedef enum {
    CONTROL_RESET_REQUESTED = 0,
    CONTROL_RESET_MODE_CHANGE,
    CONTROL_RESET_SAFETY,
} control_reset_reason_t;

typedef enum {
    CONTROL_STATE_UNINITIALIZED = 0,
    CONTROL_STATE_DISABLED,
    CONTROL_STATE_READY,
    CONTROL_STATE_RUNNING,
} control_state_t;

typedef enum {
    CONTROL_MODE_RESET_INTEGRATORS = 0,
    CONTROL_MODE_FREEZE_INTEGRATORS,
} control_mode_transition_policy_t;

typedef enum {
    CONTROL_FLAG_NONE = 0,
    CONTROL_FLAG_TEMP_WALL_ACTIVE = 1U << 0,
    CONTROL_FLAG_POWER_WALL_ACTIVE = 1U << 1,
    CONTROL_FLAG_POWER_INPUT_INVALID = 1U << 2,
    CONTROL_FLAG_INTEGRAL_FROZEN = 1U << 3,
    CONTROL_FLAG_CAPACITY_LIMITED = 1U << 4,
    CONTROL_FLAG_DISTURBANCE_DETECTED = 1U << 5,
    CONTROL_FLAG_FEEDFORWARD_ACTIVE = 1U << 6,
    CONTROL_FLAG_INPUT_INVALID = 1U << 7,
    CONTROL_FLAG_TEMPERATURE_UPDATED = 1U << 8,
    CONTROL_FLAG_CURRENT_UPDATED = 1U << 9,
} control_status_flags_t;

typedef struct {
    control_temperature_channel_t feedback_channel;
    float kp;
    float ki;
    float kd;
    float integral_min;
    float integral_max;
    float output_min_current_a;
    float output_max_current_a;
    float anti_windup_gain;
} control_temperature_config_t;

typedef struct {
    bool enabled;
    float air_rate_deadband_c_per_s;
    float rate_filter_time_constant_s;
    float kff_rate_a_per_c_per_s;
    float ff_min_current_a;
    float ff_max_current_a;
    float rise_rate_a_per_s;
    float fall_rate_a_per_s;
    float decay_timeout_s;
} control_feedforward_config_t;

typedef struct {
    float kp;
    float ki;
    float integral_min;
    float integral_max;
    float duty_min_percent;
    float duty_max_percent;
    float error_deadband_a;
    float anti_windup_gain;
    float feedback_filter_time_constant_s;
} control_current_config_t;

typedef struct {
    float normal_current_limit_a;
    float user_current_limit_a;
    float hot_wall_start_c;
    float hot_wall_max_c;
    float tec_power_limit_w;
    float power_wall_hysteresis_w;
    float power_wall_attack_rate_a_per_s;
    float power_wall_release_rate_a_per_s;
    float fan_start_temp_c;
    float fan_full_speed_temp_c;
    uint8_t fan_min_speed_percent;
    uint8_t fan_max_speed_percent;
} control_soft_limits_config_t;

typedef struct {
    uint32_t temperature_period_ms;
    uint32_t fan_period_ms;
    uint32_t max_snapshot_age_ms;
    control_mode_transition_policy_t mode_transition_policy;
} control_timing_config_t;

typedef struct {
    uint32_t version;
    control_temperature_config_t temperature;
    control_feedforward_config_t feedforward;
    control_current_config_t current;
    control_soft_limits_config_t soft_limits;
    control_timing_config_t timing;
} control_algorithm_config_t;

typedef struct {
    adc_sensor_snapshot_t adc;
    ina226_snapshot_t ina;
    float temperature_setpoint_c;
    bool temperature_setpoint_valid;
    control_disturbance_hint_t disturbance_hint;
    uint32_t now_ms;
} control_input_snapshot_t;

typedef struct {
    bool permit;
    bool trip_latched;
    float ramped_duty_percent;
    float physical_duty_percent;
    bool output_enabled;
} control_actuator_snapshot_t;

typedef struct {
    float requested_current_a;
    float temperature_pi_current_a;
    float feedforward_current_a;
    float current_ref_a;
    float filtered_current_a;
    float duty_request_percent;
    float ramped_duty_percent;
    float physical_duty_percent;
    bool output_enabled;
    uint8_t fan1_speed_percent;
    float temperature_error_c;
    float air_temperature_rate_c_per_s;
    float temperature_wall_current_limit_a;
    float power_wall_current_limit_a;
    uint32_t adc_sequence;
    uint32_t adc_sample_time_ms;
    uint32_t ina_sequence;
    uint32_t ina_timestamp_ms;
    bool temperature_valid;
    bool current_valid;
    uint32_t flags;
    control_state_t state;
    uint32_t sequence;
} control_output_t;

typedef control_output_t control_status_t;

esp_err_t control_algorithm_init(void);
/* Build the complete first-boot configuration from Kconfig and fixed
 * development defaults.  This does not alter algorithm state. */
esp_err_t control_algorithm_get_default_config(control_algorithm_config_t *config);
/* Validate a complete configuration without changing algorithm state. */
bool control_algorithm_config_is_valid(const control_algorithm_config_t *config);
/* Convenience entry point for callers that already have a complete
 * configuration object. It performs init followed by configure in one call. */
esp_err_t control_algorithm_init_with_config(const control_algorithm_config_t *config);
esp_err_t control_algorithm_configure(const control_algorithm_config_t *config);
esp_err_t control_algorithm_enable(void);
esp_err_t control_algorithm_disable(control_disable_reason_t reason);
esp_err_t control_algorithm_reset_integrators(control_reset_reason_t reason);
esp_err_t control_algorithm_step(const control_input_snapshot_t *input,
                                 const control_actuator_snapshot_t *actuator,
                                 control_output_t *output);
esp_err_t control_algorithm_get_status(control_status_t *status);

#ifdef __cplusplus
}
#endif
