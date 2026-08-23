#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TEC_PWM_MCPWM_GROUP_ID       1
#define TEC_PWM_FREQUENCY_HZ         100000U
#define TEC_PWM_RESOLUTION_HZ        80000000U
#define TEC_PWM_PERIOD_TICKS         800U
#define TEC_PWM_MAX_COMPARE_TICKS    (TEC_PWM_PERIOD_TICKS - 1U)
#define TEC_PWM_UPDATE_PERIOD_MS     10U
#define TEC_PWM_RAMP_STEP_PERCENT    1.0f
#define TEC_PWM_ENABLE_PERCENT       6.0f
#define TEC_PWM_DISABLE_PERCENT      4.0f

typedef enum {
    TEC_PWM_STATE_UNINITIALIZED = 0,
    TEC_PWM_STATE_SAFE_OFF,
    TEC_PWM_STATE_READY,
    TEC_PWM_STATE_RUNNING,
    TEC_PWM_STATE_TRIPPED,
} tec_pwm_state_t;

typedef enum {
    TEC_PWM_TRIP_NONE = 0,
    TEC_PWM_TRIP_POWER_ON,
    TEC_PWM_TRIP_ALT,
    TEC_PWM_TRIP_SOFTWARE_EMERGENCY,
    TEC_PWM_TRIP_DRIVER_ERROR,
} tec_pwm_trip_reason_t;

typedef struct {
    float target_duty_percent;
    float ramped_duty_percent;
    float physical_duty_percent;
    uint32_t compare_ticks;
    bool output_enabled;
} tec_pwm_trip_snapshot_t;

typedef struct {
    tec_pwm_state_t state;
    float target_duty_percent;
    float ramped_duty_percent;
    float physical_duty_percent;
    uint32_t compare_ticks;
    bool output_enabled;
    bool permit;
    bool trip_latched;
    tec_pwm_trip_reason_t last_trip_reason;
    uint32_t last_trip_time_ms;
    tec_pwm_trip_snapshot_t last_trip_snapshot;
    uint32_t sequence;
    esp_err_t last_error;
} tec_pwm_status_t;

/*
 * Runs in the MCPWM ISR after ALT has already caused an OST hardware brake.
 * The callback may only call APIs documented as ISR-safe and returns whether
 * a higher-priority task was woken.
 */
typedef bool (*tec_pwm_alt_isr_callback_t)(void *user_ctx);

esp_err_t tec_pwm_init(tec_pwm_alt_isr_callback_t alt_callback, void *user_ctx);
esp_err_t tec_pwm_deinit(void);

esp_err_t tec_pwm_grant_permit(void);
esp_err_t tec_pwm_revoke_permit(void);

esp_err_t tec_pwm_set_target(float duty_percent);
esp_err_t tec_pwm_request_stop(void);
esp_err_t tec_pwm_update_10ms(void);

esp_err_t tec_pwm_emergency_shutdown(tec_pwm_trip_reason_t reason);
esp_err_t tec_pwm_recover(void);

/* Available before initialization so callers can observe UNINITIALIZED state. */
esp_err_t tec_pwm_get_status(tec_pwm_status_t *status);

#ifdef __cplusplus
}
#endif
