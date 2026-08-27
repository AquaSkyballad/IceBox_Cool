#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Product role: FAN1 is the controllable hot-side fan. FAN2 is the
 * fixed-speed cold-side fan; its TACH remains available for safety checks and
 * diagnostics, while the generic low-level setter APIs are retained for
 * hardware test code. */

#define FAN_PWM_FREQUENCY_HZ       23000U
#define FAN_PWM_RESOLUTION_HZ      1000000U
#define FAN_PWM_PERIOD_TICKS       ((FAN_PWM_RESOLUTION_HZ + FAN_PWM_FREQUENCY_HZ / 2U) / FAN_PWM_FREQUENCY_HZ)
#define FAN_TACH_PULSES_PER_REV    2U
#define FAN_TACH_GLITCH_FILTER_NS  1000U

typedef enum {
    FAN_ID_1 = 0,
    FAN_ID_2,
    FAN_ID_MAX,
} fan_id_t;

typedef struct {
    bool initialized;
    /* Applied speed after quantization to FAN_PWM_PERIOD_TICKS. */
    uint8_t speed_percent;
    uint32_t gpio_duty_ticks;
    uint32_t tach_pulses;
    uint32_t rpm;
    bool tach_has_signal;
    uint32_t sample_time_ms;
    esp_err_t last_error;
} fan_status_t;

typedef struct {
    uint32_t pulses;
    uint32_t elapsed_ms;
    uint32_t rpm;
    bool has_signal;
} fan_tach_sample_t;

esp_err_t fan_control_init(void);
esp_err_t fan_control_deinit(void);
esp_err_t fan_control_set_speed(fan_id_t fan_id, uint8_t speed_percent);
esp_err_t fan_control_set_pwm_duty(fan_id_t fan_id, uint32_t gpio_duty_ticks);
esp_err_t fan_control_set_full_speed(fan_id_t fan_id);
esp_err_t fan_control_get_status(fan_id_t fan_id, fan_status_t *status);

esp_err_t fan_control_tach_start(fan_id_t fan_id);
esp_err_t fan_control_tach_clear(fan_id_t fan_id);
esp_err_t fan_control_tach_read(fan_id_t fan_id, fan_tach_sample_t *sample);

#ifdef __cplusplus
}
#endif
