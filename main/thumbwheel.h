#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define THUMBWHEEL_DEBOUNCE_MS      30U
#define THUMBWHEEL_LONG_PRESS_MS    1200U
#define THUMBWHEEL_EVENT_QUEUE_LEN  10U

typedef enum {
    KEY1 = 0,
    KEY2,
    KEY3,
    KEY_MAX,
} thumbwheel_key_t;

#define THUMBWHEEL_KEY1     KEY1
#define THUMBWHEEL_KEY2     KEY2
#define THUMBWHEEL_KEY3     KEY3
#define THUMBWHEEL_KEY_MAX  KEY_MAX

typedef enum {
    KEY_SHORT_PRESS = 0,
    KEY_LONG_PRESS,
    KEY_RELEASE,
} thumbwheel_event_type_t;

/* Prefixed aliases keep call sites readable without changing event values. */
#define THUMBWHEEL_EVENT_SHORT_PRESS KEY_SHORT_PRESS
#define THUMBWHEEL_EVENT_LONG_PRESS  KEY_LONG_PRESS
#define THUMBWHEEL_EVENT_RELEASE     KEY_RELEASE

typedef struct {
    thumbwheel_key_t key;
    thumbwheel_event_type_t type;
    uint32_t duration_ms;
    uint32_t timestamp_ms;
} thumbwheel_event_t;

esp_err_t thumbwheel_init(void);
esp_err_t thumbwheel_get_event(thumbwheel_event_t *event);
esp_err_t thumbwheel_wait_event(uint32_t timeout_ticks, thumbwheel_event_t *event);
esp_err_t thumbwheel_get_key_level(thumbwheel_key_t key, int *level);
uint32_t thumbwheel_get_queue_overflow_count(void);

#ifdef __cplusplus
}
#endif
