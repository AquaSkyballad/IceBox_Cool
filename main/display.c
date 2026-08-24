#include "display.h"

#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gpio_def.h"

static const char *TAG = "display";

#define DISPLAY_BL_TIMER        LEDC_TIMER_0
#define DISPLAY_BL_CHANNEL      LEDC_CHANNEL_0
#define DISPLAY_BL_MODE         LEDC_LOW_SPEED_MODE
#define DISPLAY_BL_RESOLUTION   LEDC_TIMER_10_BIT
#define DISPLAY_BL_FREQUENCY_HZ 1000U
#define DISPLAY_BL_MAX_DUTY     ((1U << 10) - 1U)

/* Keep sample-specific panel choices in one place for bench adjustment. */
#define DISPLAY_SPI_HOST        SPI2_HOST
#define DISPLAY_SWAP_XY        false
#define DISPLAY_MIRROR_X       false
#define DISPLAY_MIRROR_Y       false
#define DISPLAY_GAP_X          0
#define DISPLAY_GAP_Y          0
#define DISPLAY_INVERT_COLOR   false
#define DISPLAY_COLOR_ORDER    LCD_RGB_ELEMENT_ORDER_BGR

static esp_lcd_panel_io_handle_t s_io;
static esp_lcd_panel_handle_t s_panel;
static display_status_t s_status = {
    .last_error = ESP_ERR_INVALID_STATE,
};

static esp_err_t display_set_error(esp_err_t error)
{
    s_status.last_error = error;
    return error;
}

static esp_err_t display_init_backlight(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = DISPLAY_BL_MODE,
        .timer_num = DISPLAY_BL_TIMER,
        .duty_resolution = DISPLAY_BL_RESOLUTION,
        .freq_hz = DISPLAY_BL_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        return ret;
    }

    const ledc_channel_config_t channel_config = {
        .gpio_num = TFT_BLK_GPIO,
        .speed_mode = DISPLAY_BL_MODE,
        .channel = DISPLAY_BL_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = DISPLAY_BL_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    return ledc_channel_config(&channel_config);
}

esp_err_t display_init(void)
{
    if (s_status.initialized) {
        return display_set_error(ESP_ERR_INVALID_STATE);
    }

    esp_err_t ret = display_init_backlight();
    if (ret != ESP_OK) {
        return display_set_error(ret);
    }

    const spi_bus_config_t bus_config = {
        .mosi_io_num = TFT_MOSI_GPIO,
        .miso_io_num = -1,
        .sclk_io_num = TFT_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t),
    };
    ret = spi_bus_initialize(DISPLAY_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        return display_set_error(ret);
    }

    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = TFT_CS_GPIO,
        .dc_gpio_num = TFT_DC_GPIO,
        .spi_mode = 0,
        .pclk_hz = DISPLAY_SPI_FREQUENCY_HZ,
        .trans_queue_depth = 4,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)DISPLAY_SPI_HOST, &io_config, &s_io);
    if (ret != ESP_OK) {
        esp_err_t cleanup_ret = spi_bus_free(DISPLAY_SPI_HOST);
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus cleanup failed: %s", esp_err_to_name(cleanup_ret));
        }
        return display_set_error(ret);
    }

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TFT_RST_GPIO,
        .rgb_ele_order = DISPLAY_COLOR_ORDER,
        .bits_per_pixel = 16,
    };
    ret = esp_lcd_new_panel_st7789(s_io, &panel_config, &s_panel);
    if (ret != ESP_OK) {
        esp_err_t cleanup_ret = esp_lcd_panel_io_del(s_io);
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "panel IO cleanup failed: %s", esp_err_to_name(cleanup_ret));
        }
        s_io = NULL;
        cleanup_ret = spi_bus_free(DISPLAY_SPI_HOST);
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus cleanup failed: %s", esp_err_to_name(cleanup_ret));
        }
        return display_set_error(ret);
    }

    ret = esp_lcd_panel_reset(s_panel);
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_init(s_panel);
    }
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_swap_xy(s_panel, DISPLAY_SWAP_XY);
    }
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_mirror(s_panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    }
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_set_gap(s_panel, DISPLAY_GAP_X, DISPLAY_GAP_Y);
    }
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_invert_color(s_panel, DISPLAY_INVERT_COLOR);
    }
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_disp_on_off(s_panel, true);
    }
    if (ret != ESP_OK) {
        esp_err_t cleanup_ret = esp_lcd_panel_del(s_panel);
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "panel cleanup failed: %s", esp_err_to_name(cleanup_ret));
        }
        cleanup_ret = esp_lcd_panel_io_del(s_io);
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "panel IO cleanup failed: %s", esp_err_to_name(cleanup_ret));
        }
        s_panel = NULL;
        s_io = NULL;
        cleanup_ret = spi_bus_free(DISPLAY_SPI_HOST);
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus cleanup failed: %s", esp_err_to_name(cleanup_ret));
        }
        return display_set_error(ret);
    }

    s_status.initialized = true;
    s_status.sleeping = false;
    s_status.backlight_percent = 0;
    s_status.last_error = ESP_OK;
    ESP_LOGI(TAG, "initialized ST7789: %dx%d at %u Hz", DISPLAY_WIDTH, DISPLAY_HEIGHT,
             DISPLAY_SPI_FREQUENCY_HZ);
    return ESP_OK;
}

esp_err_t display_deinit(void)
{
    if (!s_status.initialized) {
        return display_set_error(ESP_ERR_INVALID_STATE);
    }

    esp_err_t first_error = display_set_backlight(0);

    if (s_panel != NULL) {
        esp_err_t ret = esp_lcd_panel_del(s_panel);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_panel = NULL;
    }
    if (s_io != NULL) {
        esp_err_t ret = esp_lcd_panel_io_del(s_io);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_io = NULL;
    }
    esp_err_t ret = spi_bus_free(DISPLAY_SPI_HOST);
    if (ret != ESP_OK && first_error == ESP_OK) {
        first_error = ret;
    }
    s_status.initialized = false;
    s_status.sleeping = false;
    s_status.backlight_percent = 0;
    s_status.last_error = first_error;
    return first_error;
}

esp_lcd_panel_handle_t display_get_panel(void)
{
    return s_panel;
}

esp_lcd_panel_io_handle_t display_get_panel_io(void)
{
    return s_io;
}

esp_err_t display_set_backlight(uint8_t percent)
{
    if (percent > DISPLAY_BACKLIGHT_MAX) {
        return display_set_error(ESP_ERR_INVALID_ARG);
    }
    if (!s_status.initialized) {
        return display_set_error(ESP_ERR_INVALID_STATE);
    }
    const uint32_t duty = ((uint32_t)percent * DISPLAY_BL_MAX_DUTY) /
                          DISPLAY_BACKLIGHT_MAX;
    /* Backlight updates are serialized by the display owner. The thread-safe
       combined API depends on the LEDC fade service in IDF 5.5, which is not
       needed for an immediate duty change. */
    esp_err_t ret = ledc_set_duty(DISPLAY_BL_MODE, DISPLAY_BL_CHANNEL, duty);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(DISPLAY_BL_MODE, DISPLAY_BL_CHANNEL);
    }
    if (ret == ESP_OK) {
        s_status.backlight_percent = percent;
        s_status.last_error = ESP_OK;
    } else {
        s_status.last_error = ret;
    }
    return ret;
}

esp_err_t display_sleep(void)
{
    if (!s_status.initialized || s_panel == NULL) {
        return display_set_error(ESP_ERR_INVALID_STATE);
    }
    /* Sleep intentionally leaves the backlight disabled; wakeup does not
       restore brightness unless the caller explicitly sets it again. */
    esp_err_t ret = display_set_backlight(0);
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_disp_on_off(s_panel, false);
    }
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_disp_sleep(s_panel, true);
    }
    if (ret == ESP_OK) {
        s_status.sleeping = true;
    }
    return display_set_error(ret);
}

esp_err_t display_wakeup(void)
{
    if (!s_status.initialized || s_panel == NULL) {
        return display_set_error(ESP_ERR_INVALID_STATE);
    }
    esp_err_t ret = esp_lcd_panel_disp_sleep(s_panel, false);
    if (ret == ESP_OK) {
        /* IDF's ST7789 helper waits 100 ms; keep the panel wake delay at 120 ms. */
        vTaskDelay(pdMS_TO_TICKS(20));
        ret = esp_lcd_panel_disp_on_off(s_panel, true);
    }
    if (ret == ESP_OK) {
        s_status.sleeping = false;
    }
    return display_set_error(ret);
}

esp_err_t display_draw_bitmap(int x_start, int y_start, int x_end, int y_end,
                              const void *color_data)
{
    if (!s_status.initialized || s_panel == NULL) {
        return display_set_error(ESP_ERR_INVALID_STATE);
    }
    if (color_data == NULL) {
        return display_set_error(ESP_ERR_INVALID_ARG);
    }
    if (x_start < 0 || y_start < 0 || x_end <= x_start || y_end <= y_start ||
        x_end > DISPLAY_WIDTH || y_end > DISPLAY_HEIGHT) {
        return display_set_error(ESP_ERR_INVALID_ARG);
    }
    esp_err_t ret = esp_lcd_panel_draw_bitmap(s_panel, x_start, y_start, x_end, y_end,
                                              color_data);
    if (ret != ESP_OK) {
        s_status.last_error = ret;
    }
    return ret;
}

esp_err_t display_get_status(display_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *status = s_status;
    return ESP_OK;
}
