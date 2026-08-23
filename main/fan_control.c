#include "fan_control.h"

#include <string.h>

#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "gpio_def.h"

static const char *TAG = "fan_driver";

/* 1 MHz timer resolution gives 43 ticks at 23 kHz (actual frequency ~23.26 kHz). */

static const gpio_num_t s_pwm_gpio[FAN_ID_MAX] = {FAN1_PWM_GPIO, FAN2_PWM_GPIO};
static const gpio_num_t s_tach_gpio[FAN_ID_MAX] = {FAN1_TACH_GPIO, FAN2_TACH_GPIO};

static mcpwm_timer_handle_t s_timer;
static mcpwm_oper_handle_t s_operator[FAN_ID_MAX];
static mcpwm_cmpr_handle_t s_comparator[FAN_ID_MAX];
static mcpwm_gen_handle_t s_generator[FAN_ID_MAX];
static pcnt_unit_handle_t s_pcnt[FAN_ID_MAX];
static pcnt_channel_handle_t s_pcnt_channel[FAN_ID_MAX];
static int64_t s_tach_start_us[FAN_ID_MAX];
static fan_status_t s_status[FAN_ID_MAX];
static bool s_initialized;

static uint8_t fan_gpio_duty_to_speed_percent(uint32_t gpio_duty_ticks)
{
    const uint32_t released_ticks = FAN_PWM_PERIOD_TICKS - gpio_duty_ticks;
    uint32_t speed_percent = (released_ticks * 100U + FAN_PWM_PERIOD_TICKS / 2U) /
                             FAN_PWM_PERIOD_TICKS;
    if (speed_percent > 100U) {
        speed_percent = 100U;
    }
    return (uint8_t)speed_percent;
}

static uint32_t fan_speed_to_gpio_duty(uint8_t speed_percent)
{
    const uint32_t gpio_duty = ((100U - speed_percent) * FAN_PWM_PERIOD_TICKS + 50U) / 100U;
    return gpio_duty > FAN_PWM_PERIOD_TICKS ? FAN_PWM_PERIOD_TICKS : gpio_duty;
}

static esp_err_t fan_release_resources(void)
{
    esp_err_t first_error = ESP_OK;

    /* Release the PWM input before stopping the timer, so teardown is also a
       defined full-speed state for a powered standard four-wire fan. */
    for (int i = 0; i < FAN_ID_MAX; ++i) {
        if (s_comparator[i] != NULL) {
            esp_err_t ret = mcpwm_comparator_set_compare_value(s_comparator[i], 0);
            if (ret != ESP_OK && first_error == ESP_OK) {
                first_error = ret;
            }
        }
    }

    if (s_timer != NULL) {
        /* Deletion requires the timer to be stopped and disabled. */
        esp_err_t ret = mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_STOP_EMPTY);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        ret = mcpwm_timer_disable(s_timer);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
    }
    for (int i = 0; i < FAN_ID_MAX; ++i) {
        if (s_pcnt[i] != NULL) {
            esp_err_t ret = pcnt_unit_stop(s_pcnt[i]);
            if (ret != ESP_OK && first_error == ESP_OK) {
                first_error = ret;
            }
            ret = pcnt_unit_disable(s_pcnt[i]);
            if (ret != ESP_OK && first_error == ESP_OK) {
                first_error = ret;
            }
        }
        if (s_pcnt_channel[i] != NULL) {
            esp_err_t ret = pcnt_del_channel(s_pcnt_channel[i]);
            if (ret != ESP_OK && first_error == ESP_OK) {
                first_error = ret;
            }
            s_pcnt_channel[i] = NULL;
        }
        if (s_pcnt[i] != NULL) {
            esp_err_t ret = pcnt_del_unit(s_pcnt[i]);
            if (ret != ESP_OK && first_error == ESP_OK) {
                first_error = ret;
            }
            s_pcnt[i] = NULL;
        }
        if (s_generator[i] != NULL) {
            esp_err_t ret = mcpwm_del_generator(s_generator[i]);
            if (ret != ESP_OK && first_error == ESP_OK) {
                first_error = ret;
            }
            s_generator[i] = NULL;
        }
        if (s_comparator[i] != NULL) {
            esp_err_t ret = mcpwm_del_comparator(s_comparator[i]);
            if (ret != ESP_OK && first_error == ESP_OK) {
                first_error = ret;
            }
            s_comparator[i] = NULL;
        }
        if (s_operator[i] != NULL) {
            esp_err_t ret = mcpwm_del_operator(s_operator[i]);
            if (ret != ESP_OK && first_error == ESP_OK) {
                first_error = ret;
            }
            s_operator[i] = NULL;
        }
    }
    if (s_timer != NULL) {
        esp_err_t ret = mcpwm_del_timer(s_timer);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_timer = NULL;
    }
    return first_error;
}

static esp_err_t fan_init_pwm(void)
{
    const mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = FAN_PWM_RESOLUTION_HZ,
        .period_ticks = FAN_PWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    esp_err_t ret = mcpwm_new_timer(&timer_config, &s_timer);
    if (ret != ESP_OK) {
        return ret;
    }

    for (int i = 0; i < FAN_ID_MAX; ++i) {
        const mcpwm_operator_config_t operator_config = {.group_id = 0};
        ret = mcpwm_new_operator(&operator_config, &s_operator[i]);
        if (ret != ESP_OK) {
            return ret;
        }
        ret = mcpwm_operator_connect_timer(s_operator[i], s_timer);
        if (ret != ESP_OK) {
            return ret;
        }

        const mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };
        ret = mcpwm_new_comparator(s_operator[i], &comparator_config, &s_comparator[i]);
        if (ret != ESP_OK) {
            return ret;
        }
        const mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = s_pwm_gpio[i],
        };
        ret = mcpwm_new_generator(s_operator[i], &generator_config, &s_generator[i]);
        if (ret != ESP_OK) {
            return ret;
        }

        /* GPIO duty is high time. High drives the external NMOS and slows the fan. */
        ret = mcpwm_generator_set_action_on_timer_event(
            s_generator[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH));
        if (ret == ESP_OK) {
            ret = mcpwm_generator_set_action_on_compare_event(
                s_generator[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                               s_comparator[i],
                                               MCPWM_GEN_ACTION_LOW));
        }
        if (ret != ESP_OK) {
            return ret;
        }
        ret = mcpwm_comparator_set_compare_value(s_comparator[i], 0);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    ret = mcpwm_timer_enable(s_timer);
    if (ret == ESP_OK) {
        ret = mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP);
    }
    return ret;
}

static esp_err_t fan_init_tach(void)
{
    for (int i = 0; i < FAN_ID_MAX; ++i) {
        const pcnt_unit_config_t unit_config = {
            .high_limit = 32767,
            .low_limit = -32768,
        };
        esp_err_t ret = pcnt_new_unit(&unit_config, &s_pcnt[i]);
        if (ret != ESP_OK) {
            return ret;
        }

        const pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = FAN_TACH_GLITCH_FILTER_NS,
        };
        ret = pcnt_unit_set_glitch_filter(s_pcnt[i], &filter_config);
        if (ret != ESP_OK) {
            return ret;
        }

        const pcnt_chan_config_t channel_config = {
            .edge_gpio_num = s_tach_gpio[i],
            .level_gpio_num = -1,
        };
        ret = pcnt_new_channel(s_pcnt[i], &channel_config, &s_pcnt_channel[i]);
        if (ret != ESP_OK) {
            return ret;
        }
        ret = pcnt_channel_set_edge_action(s_pcnt_channel[i],
                                           PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                           PCNT_CHANNEL_EDGE_ACTION_HOLD);
        if (ret != ESP_OK) {
            return ret;
        }
        ret = pcnt_unit_enable(s_pcnt[i]);
        if (ret == ESP_OK) {
            ret = pcnt_unit_clear_count(s_pcnt[i]);
        }
        if (ret == ESP_OK) {
            ret = pcnt_unit_start(s_pcnt[i]);
        }
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

esp_err_t fan_control_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(s_status, 0, sizeof(s_status));
    esp_err_t ret = fan_init_pwm();
    if (ret == ESP_OK) {
        ret = fan_init_tach();
    }
    if (ret != ESP_OK) {
        esp_err_t cleanup_ret = fan_release_resources();
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "initialization cleanup failed: %s", esp_err_to_name(cleanup_ret));
        }
        return ret;
    }

    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
    for (int i = 0; i < FAN_ID_MAX; ++i) {
        s_status[i].initialized = true;
        s_status[i].speed_percent = 100;
        s_status[i].gpio_duty_ticks = 0;
        s_status[i].sample_time_ms = now_ms;
        s_status[i].last_error = ESP_OK;
        s_tach_start_us[i] = 0;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "initialized: PWM=%u Hz, period=%u ticks", FAN_PWM_FREQUENCY_HZ,
             FAN_PWM_PERIOD_TICKS);
    return ESP_OK;
}

esp_err_t fan_control_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = fan_release_resources();
    memset(s_status, 0, sizeof(s_status));
    memset(s_tach_start_us, 0, sizeof(s_tach_start_us));
    s_initialized = false;
    return ret;
}

esp_err_t fan_control_set_pwm_duty(fan_id_t fan_id, uint32_t gpio_duty_ticks)
{
    if (fan_id >= FAN_ID_MAX || gpio_duty_ticks > FAN_PWM_PERIOD_TICKS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = mcpwm_comparator_set_compare_value(s_comparator[fan_id], gpio_duty_ticks);
    if (ret == ESP_OK) {
        s_status[fan_id].gpio_duty_ticks = gpio_duty_ticks;
        s_status[fan_id].speed_percent = fan_gpio_duty_to_speed_percent(gpio_duty_ticks);
        s_status[fan_id].last_error = ESP_OK;
    } else {
        s_status[fan_id].last_error = ret;
    }
    return ret;
}

esp_err_t fan_control_set_speed(fan_id_t fan_id, uint8_t speed_percent)
{
    if (fan_id >= FAN_ID_MAX || speed_percent > 100U) {
        return ESP_ERR_INVALID_ARG;
    }
    return fan_control_set_pwm_duty(fan_id, fan_speed_to_gpio_duty(speed_percent));
}

esp_err_t fan_control_set_full_speed(fan_id_t fan_id)
{
    return fan_control_set_speed(fan_id, 100);
}

esp_err_t fan_control_get_status(fan_id_t fan_id, fan_status_t *status)
{
    if (fan_id >= FAN_ID_MAX || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    *status = s_status[fan_id];
    return ESP_OK;
}

esp_err_t fan_control_tach_start(fan_id_t fan_id)
{
    if (fan_id >= FAN_ID_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = pcnt_unit_stop(s_pcnt[fan_id]);
    if (ret == ESP_OK) {
        ret = pcnt_unit_clear_count(s_pcnt[fan_id]);
    }
    const int64_t start_us = esp_timer_get_time();
    if (ret == ESP_OK) {
        ret = pcnt_unit_start(s_pcnt[fan_id]);
    }
    if (ret == ESP_OK) {
        s_tach_start_us[fan_id] = start_us;
        s_status[fan_id].last_error = ESP_OK;
    } else {
        s_tach_start_us[fan_id] = 0;
        s_status[fan_id].last_error = ret;
    }
    return ret;
}

esp_err_t fan_control_tach_clear(fan_id_t fan_id)
{
    if (fan_id >= FAN_ID_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = pcnt_unit_stop(s_pcnt[fan_id]);
    if (ret == ESP_OK) {
        ret = pcnt_unit_clear_count(s_pcnt[fan_id]);
    }
    const int64_t start_us = esp_timer_get_time();
    if (ret == ESP_OK) {
        ret = pcnt_unit_start(s_pcnt[fan_id]);
    }
    if (ret == ESP_OK) {
        /* Clearing starts a fresh, well-defined measurement window. */
        s_tach_start_us[fan_id] = start_us;
        s_status[fan_id].last_error = ESP_OK;
    } else {
        s_tach_start_us[fan_id] = 0;
        s_status[fan_id].last_error = ret;
    }
    return ret;
}

esp_err_t fan_control_tach_read(fan_id_t fan_id, fan_tach_sample_t *sample)
{
    if (fan_id >= FAN_ID_MAX || sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_tach_start_us[fan_id] == 0) {
        s_status[fan_id].last_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = pcnt_unit_stop(s_pcnt[fan_id]);
    if (ret != ESP_OK) {
        s_status[fan_id].last_error = ret;
        return ret;
    }
    const int64_t end_us = esp_timer_get_time();
    int count = 0;
    ret = pcnt_unit_get_count(s_pcnt[fan_id], &count);
    esp_err_t restart_ret = pcnt_unit_start(s_pcnt[fan_id]);
    if (ret == ESP_OK && restart_ret != ESP_OK) {
        ret = restart_ret;
    }
    if (ret != ESP_OK) {
        s_status[fan_id].last_error = ret;
        return ret;
    }
    const int64_t elapsed_us = end_us - s_tach_start_us[fan_id];
    if (elapsed_us <= 0 || count < 0) {
        s_status[fan_id].last_error = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    const uint32_t elapsed_ms = (uint32_t)(elapsed_us / 1000LL);
    const uint32_t rpm = (uint32_t)(((uint64_t)count * 60ULL * 1000000ULL) /
                                    ((uint64_t)FAN_TACH_PULSES_PER_REV * (uint64_t)elapsed_us));
    sample->pulses = (uint32_t)count;
    sample->elapsed_ms = elapsed_ms;
    sample->rpm = rpm;
    sample->has_signal = count > 0;

    s_status[fan_id].tach_pulses = sample->pulses;
    s_status[fan_id].rpm = sample->rpm;
    s_status[fan_id].tach_has_signal = sample->has_signal;
    s_status[fan_id].sample_time_ms = (uint32_t)(end_us / 1000LL);
    s_status[fan_id].last_error = ESP_OK;
    return ESP_OK;
}
