#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_WIDTH              240
#define DISPLAY_HEIGHT             320
#define DISPLAY_SPI_FREQUENCY_HZ   40000000U
#define DISPLAY_BACKLIGHT_MAX      100U

typedef struct {
    bool initialized;
    bool sleeping;
    uint8_t backlight_percent;
    esp_err_t last_error;
} display_status_t;

esp_err_t display_init(void);
esp_err_t display_deinit(void);
esp_lcd_panel_handle_t display_get_panel(void);
esp_err_t display_set_backlight(uint8_t percent);
esp_err_t display_sleep(void);
esp_err_t display_wakeup(void);
esp_err_t display_draw_bitmap(int x_start, int y_start, int x_end, int y_end, const void *color_data);
esp_err_t display_get_status(display_status_t *status);

#ifdef __cplusplus
}
#endif
