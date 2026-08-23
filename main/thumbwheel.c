#include "thumbwheel.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "gpio_def.h"

static const char *TAG = "thumbwheel";

typedef struct {
    thumbwheel_key_t key;
    gpio_num_t gpio;
    TimerHandle_t long_timer;
    volatile TickType_t last_accepted_edge;
    volatile TickType_t press_tick;
    volatile bool pressed;
    volatile bool long_reported;
} key_state_t;

static QueueHandle_t s_event_queue;
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_queue_overflow_count;
static bool s_initialized;

static key_state_t s_keys[THUMBWHEEL_KEY_MAX] = {
    {.key = THUMBWHEEL_KEY1, .gpio = KEY1_GPIO},
    {.key = THUMBWHEEL_KEY2, .gpio = KEY2_GPIO},
    {.key = THUMBWHEEL_KEY3, .gpio = KEY3_GPIO},
};

static uint32_t tick_to_ms(TickType_t ticks)
{
    return (uint32_t)(ticks * portTICK_PERIOD_MS);
}

static void queue_event_from_isr(const thumbwheel_event_t *event, BaseType_t *higher_priority)
{
    if (s_event_queue == NULL || xQueueSendFromISR(s_event_queue, event, higher_priority) != pdTRUE) {
        s_queue_overflow_count++;
    }
}

static void queue_event_from_task(const thumbwheel_event_t *event)
{
    if (s_event_queue == NULL || xQueueSend(s_event_queue, event, 0) != pdTRUE) {
        s_queue_overflow_count++;
    }
}

static void long_press_timer_callback(TimerHandle_t timer)
{
    key_state_t *state = (key_state_t *)pvTimerGetTimerID(timer);
    if (state == NULL || s_event_queue == NULL) {
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    bool emit = false;
    thumbwheel_event_t event = {
        .key = state->key,
        .type = THUMBWHEEL_EVENT_LONG_PRESS,
        .duration_ms = 0,
        .timestamp_ms = tick_to_ms(now),
    };

    portENTER_CRITICAL(&s_state_lock);
    if (state->pressed && !state->long_reported) {
        state->long_reported = true;
        event.duration_ms = tick_to_ms(now - state->press_tick);
        emit = true;
    }
    portEXIT_CRITICAL(&s_state_lock);

    if (emit) {
        queue_event_from_task(&event);
    }
}

static void IRAM_ATTR thumbwheel_isr(void *arg)
{
    key_state_t *state = (key_state_t *)arg;
    if (state == NULL || s_event_queue == NULL) {
        return;
    }

    const TickType_t now = xTaskGetTickCountFromISR();
    if ((now - state->last_accepted_edge) < pdMS_TO_TICKS(THUMBWHEEL_DEBOUNCE_MS)) {
        return;
    }

    const bool pressed_now = gpio_get_level(state->gpio) == 0;
    BaseType_t higher_priority = pdFALSE;
    bool accepted = false;
    bool emit_short = false;
    bool emit_release = false;
    bool start_timer = false;
    bool stop_timer = false;
    uint32_t duration_ms = 0;

    portENTER_CRITICAL_ISR(&s_state_lock);
    if (pressed_now != state->pressed) {
        state->last_accepted_edge = now;
        accepted = true;
        if (pressed_now) {
            state->pressed = true;
            state->press_tick = now;
            state->long_reported = false;
            start_timer = true;
        } else {
            state->pressed = false;
            duration_ms = tick_to_ms(now - state->press_tick);
            emit_short = !state->long_reported;
            emit_release = true;
            stop_timer = true;
        }
    }
    portEXIT_CRITICAL_ISR(&s_state_lock);

    if (start_timer && state->long_timer != NULL) {
        xTimerStartFromISR(state->long_timer, &higher_priority);
    }
    if (stop_timer && state->long_timer != NULL) {
        xTimerStopFromISR(state->long_timer, &higher_priority);
    }

    if (accepted && emit_short) {
        const thumbwheel_event_t event = {
            .key = state->key,
            .type = THUMBWHEEL_EVENT_SHORT_PRESS,
            .duration_ms = duration_ms,
            .timestamp_ms = tick_to_ms(now),
        };
        queue_event_from_isr(&event, &higher_priority);
    }
    if (accepted && emit_release) {
        const thumbwheel_event_t event = {
            .key = state->key,
            .type = THUMBWHEEL_EVENT_RELEASE,
            .duration_ms = duration_ms,
            .timestamp_ms = tick_to_ms(now),
        };
        queue_event_from_isr(&event, &higher_priority);
    }
    if (higher_priority == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t thumbwheel_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_event_queue = xQueueCreate(THUMBWHEEL_EVENT_QUEUE_LEN, sizeof(thumbwheel_event_t));
    if (s_event_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return ret;
    }

    const gpio_config_t config = {
        .pin_bit_mask = BIT64(KEY1_GPIO) | BIT64(KEY2_GPIO) | BIT64(KEY3_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ret = gpio_config(&config);
    if (ret != ESP_OK) {
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return ret;
    }

    for (size_t i = 0; i < THUMBWHEEL_KEY_MAX; ++i) {
        s_keys[i].last_accepted_edge = xTaskGetTickCount() - pdMS_TO_TICKS(THUMBWHEEL_DEBOUNCE_MS);
        s_keys[i].pressed = gpio_get_level(s_keys[i].gpio) == 0;
        s_keys[i].press_tick = xTaskGetTickCount();
        s_keys[i].long_reported = false;
        s_keys[i].long_timer = xTimerCreate(
            "key_long",
            pdMS_TO_TICKS(THUMBWHEEL_LONG_PRESS_MS),
            pdFALSE,
            &s_keys[i],
            long_press_timer_callback);
        if (s_keys[i].long_timer == NULL) {
            for (size_t j = 0; j < i; ++j) {
                xTimerDelete(s_keys[j].long_timer, 0);
                s_keys[j].long_timer = NULL;
            }
            vQueueDelete(s_event_queue);
            s_event_queue = NULL;
            return ESP_ERR_NO_MEM;
        }
        ret = gpio_isr_handler_add(s_keys[i].gpio, thumbwheel_isr, &s_keys[i]);
        if (ret != ESP_OK) {
            for (size_t j = 0; j <= i; ++j) {
                gpio_isr_handler_remove(s_keys[j].gpio);
                xTimerDelete(s_keys[j].long_timer, 0);
                s_keys[j].long_timer = NULL;
            }
            vQueueDelete(s_event_queue);
            s_event_queue = NULL;
            return ret;
        }
        if (s_keys[i].pressed) {
            /* A key held while the driver starts still gets long-press timing. */
            if (xTimerStart(s_keys[i].long_timer, 0) != pdPASS) {
                for (size_t j = 0; j <= i; ++j) {
                    gpio_isr_handler_remove(s_keys[j].gpio);
                    if (s_keys[j].long_timer != NULL) {
                        xTimerDelete(s_keys[j].long_timer, 0);
                        s_keys[j].long_timer = NULL;
                    }
                }
                vQueueDelete(s_event_queue);
                s_event_queue = NULL;
                return ESP_ERR_TIMEOUT;
            }
        }
    }

    s_queue_overflow_count = 0;
    s_initialized = true;
    ESP_LOGI(TAG, "initialized: debounce=%u ms, long_press=%u ms",
             THUMBWHEEL_DEBOUNCE_MS, THUMBWHEEL_LONG_PRESS_MS);
    return ESP_OK;
}

esp_err_t thumbwheel_get_event(thumbwheel_event_t *event)
{
    if (event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized || s_event_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueReceive(s_event_queue, event, 0) != pdTRUE) {
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

esp_err_t thumbwheel_wait_event(uint32_t timeout_ticks, thumbwheel_event_t *event)
{
    if (event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized || s_event_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueueReceive(s_event_queue, event, timeout_ticks) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t thumbwheel_get_key_level(thumbwheel_key_t key, int *level)
{
    if (level == NULL || key >= THUMBWHEEL_KEY_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    *level = gpio_get_level(s_keys[key].gpio);
    return ESP_OK;
}

uint32_t thumbwheel_get_queue_overflow_count(void)
{
    return s_queue_overflow_count;
}
