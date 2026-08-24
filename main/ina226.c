#include "ina226.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "gpio_def.h"

static const char *TAG = "ina226";

enum {
    INA226_REG_CONFIGURATION = 0x00,
    INA226_REG_SHUNT_VOLTAGE = 0x01,
    INA226_REG_BUS_VOLTAGE = 0x02,
    INA226_REG_CURRENT = 0x04,
    INA226_REG_CALIBRATION = 0x05,
    INA226_REG_MASK_ENABLE = 0x06,
    INA226_REG_ALERT_LIMIT = 0x07,
    INA226_REG_MANUFACTURER_ID = 0xFE,
    INA226_REG_DIE_ID = 0xFF,
};

/* Configuration register fields. 588 us is conversion code 011 for both ADCs.
   Bit 14 is marked reserved in the INA226 data sheet, but its POR/readback
   value is 1 (the documented reset value is 0x4127). Keep that bit set when
   composing a configuration word; otherwise a write/readback check can
   report ESP_ERR_INVALID_RESPONSE even though the writable fields are valid. */
#define INA226_CONFIG_RESERVED_BIT14     0x4000U
#define INA226_CONFIG_AVERAGING_16       0x0400U
#define INA226_CONFIG_BUS_CT_588US       0x00C0U
#define INA226_CONFIG_SHUNT_CT_588US     0x0018U
#define INA226_CONFIG_CONTINUOUS_SH_BUS  0x0007U

/* Mask/Enable: SOL enabled, active-low alert, latched until Mask/Enable is read. */
#define INA226_MASK_SOL                  0x8000U
#define INA226_MASK_ALERT_LATCH_ENABLE   0x0001U
#define INA226_MASK_ALERT_POLARITY_HIGH  0x0002U
#define INA226_MASK_AFF                  0x0010U
#define INA226_MASK_CVRF                 0x0008U
#define INA226_MASK_OVF                  0x0004U
/* Function-select, polarity, and latch bits are writable configuration.
   AFF/CVRF/OVF are dynamic status bits and must not affect readback checks. */
#define INA226_MASK_CONFIGURATION_BITS   0xFC03U

#define INA226_INITIAL_CONVERSION_TIMEOUT_MS  50U
#define INA226_CONVERSION_POLL_INTERVAL_MS    1U

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_device;
static StaticSemaphore_t s_i2c_mutex_storage;
static SemaphoreHandle_t s_i2c_mutex;
static portMUX_TYPE s_lifecycle_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static ina226_snapshot_t s_snapshot = {
    .valid = false,
    .last_error = ESP_ERR_INVALID_STATE,
};
static bool s_initialized;
static uint16_t s_pending_alert_bits;

static esp_err_t ina226_ensure_mutex(void)
{
    portENTER_CRITICAL(&s_lifecycle_lock);
    if (s_i2c_mutex == NULL) {
        s_i2c_mutex = xSemaphoreCreateMutexStatic(&s_i2c_mutex_storage);
    }
    const bool created = s_i2c_mutex != NULL;
    portEXIT_CRITICAL(&s_lifecycle_lock);
    return created ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t ina226_lock(void)
{
    if (s_i2c_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(INA226_I2C_TIMEOUT_MS)) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

static void ina226_unlock(void)
{
    (void)xSemaphoreGive(s_i2c_mutex);
}

static esp_err_t ina226_write_register(uint8_t reg, uint16_t value)
{
    /* All register helpers are called with s_i2c_mutex held. */
    if (s_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)value,
    };
    return i2c_master_transmit(s_device, data, sizeof(data), INA226_I2C_TIMEOUT_MS);
}

static esp_err_t ina226_read_register(uint8_t reg, uint16_t *value)
{
    if (s_device == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2];
    esp_err_t ret = i2c_master_transmit_receive(
        s_device, &reg, 1, data, sizeof(data), INA226_I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return ret;
    }

    *value = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

static esp_err_t ina226_release_resources_locked(void)
{
    esp_err_t first_error = ESP_OK;

    if (s_device != NULL) {
        esp_err_t ret = i2c_master_bus_rm_device(s_device);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_device = NULL;
    }
    if (s_bus != NULL) {
        esp_err_t ret = i2c_del_master_bus(s_bus);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_bus = NULL;
    }
    return first_error;
}

static void ina226_mark_snapshot_invalid(esp_err_t error)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.valid = false;
    s_snapshot.last_error = error;
    s_snapshot.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static void ina226_capture_alert_bits(uint16_t mask_enable)
{
    s_pending_alert_bits |= mask_enable & (INA226_MASK_AFF | INA226_MASK_OVF);
}

static esp_err_t ina226_wait_for_conversion_locked(bool wait)
{
    const int64_t deadline_us = esp_timer_get_time() +
                                (int64_t)INA226_INITIAL_CONVERSION_TIMEOUT_MS * 1000LL;
    TickType_t poll_delay_ticks = pdMS_TO_TICKS(INA226_CONVERSION_POLL_INTERVAL_MS);
    if (poll_delay_ticks == 0) {
        /* CONFIG_FREERTOS_HZ may be 100, where a 1 ms delay rounds to zero. */
        poll_delay_ticks = 1;
    }

    do {
        uint16_t mask_enable = 0;
        esp_err_t ret = ina226_read_register(INA226_REG_MASK_ENABLE, &mask_enable);
        if (ret != ESP_OK) {
            return ret;
        }
        ina226_capture_alert_bits(mask_enable);
        if ((mask_enable & INA226_MASK_CVRF) != 0U) {
            return ESP_OK;
        }
        if (!wait) {
            return ESP_ERR_NOT_FINISHED;
        }
        vTaskDelay(poll_delay_ticks);
    } while (esp_timer_get_time() < deadline_us);

    return ESP_ERR_TIMEOUT;
}

static esp_err_t ina226_read_measurement_locked(uint16_t *bus_raw, uint16_t *current_raw,
                                                bool wait_for_conversion)
{
    esp_err_t ret = ina226_wait_for_conversion_locked(wait_for_conversion);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ina226_read_register(INA226_REG_BUS_VOLTAGE, bus_raw);
    if (ret == ESP_OK) {
        ret = ina226_read_register(INA226_REG_CURRENT, current_raw);
    }
    return ret;
}

static void ina226_publish_snapshot(uint16_t bus_raw, uint16_t current_raw)
{
    const int16_t signed_current = (int16_t)current_raw;
    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.bus_voltage_v = (float)bus_raw * 0.00125f;
    s_snapshot.current_a = (float)signed_current * INA226_CURRENT_LSB_A;
    s_snapshot.valid = true;
    s_snapshot.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
    s_snapshot.sequence++;
    s_snapshot.last_error = ESP_OK;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static esp_err_t ina226_verify_identity(void)
{
    uint16_t manufacturer = 0;
    uint16_t die_id = 0;

    esp_err_t ret = ina226_read_register(INA226_REG_MANUFACTURER_ID, &manufacturer);
    if (ret != ESP_OK) {
        return ret;
    }
    if (manufacturer != 0x5449U) {
        ESP_LOGE(TAG, "unexpected manufacturer ID: 0x%04x", manufacturer);
        return ESP_ERR_NOT_FOUND;
    }

    ret = ina226_read_register(INA226_REG_DIE_ID, &die_id);
    if (ret != ESP_OK) {
        return ret;
    }
    if (die_id != 0x2260U) {
        ESP_LOGE(TAG, "unexpected die ID: 0x%04x", die_id);
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

static esp_err_t ina226_apply_configuration(void)
{
    const uint16_t configuration = INA226_CONFIG_RESERVED_BIT14 |
                                   INA226_CONFIG_AVERAGING_16 |
                                   INA226_CONFIG_BUS_CT_588US |
                                   INA226_CONFIG_SHUNT_CT_588US |
                                   INA226_CONFIG_CONTINUOUS_SH_BUS;
    const uint16_t alert_limit = (uint16_t)((INA226_ALT_CURRENT_A *
                                             INA226_SHUNT_RESISTOR_OHM * 1000000.0f) /
                                            2.5f);
    const uint16_t mask_enable = INA226_MASK_SOL | INA226_MASK_ALERT_LATCH_ENABLE;

    esp_err_t ret = ina226_write_register(INA226_REG_CONFIGURATION, configuration);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ina226_write_register(INA226_REG_CALIBRATION, INA226_CALIBRATION_VALUE);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ina226_write_register(INA226_REG_ALERT_LIMIT, alert_limit);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ina226_write_register(INA226_REG_MASK_ENABLE, mask_enable);
    if (ret != ESP_OK) {
        return ret;
    }

    uint16_t value = 0;
    ret = ina226_read_register(INA226_REG_CONFIGURATION, &value);
    if (ret != ESP_OK || value != configuration) {
        if (ret == ESP_OK) {
            ESP_LOGE(TAG, "configuration readback mismatch: wrote 0x%04x, read 0x%04x",
                     configuration, value);
        }
        return ret == ESP_OK ? ESP_ERR_INVALID_RESPONSE : ret;
    }
    ret = ina226_read_register(INA226_REG_CALIBRATION, &value);
    if (ret != ESP_OK || value != INA226_CALIBRATION_VALUE) {
        if (ret == ESP_OK) {
            ESP_LOGE(TAG, "calibration readback mismatch: wrote 0x%04x, read 0x%04x",
                     INA226_CALIBRATION_VALUE, value);
        }
        return ret == ESP_OK ? ESP_ERR_INVALID_RESPONSE : ret;
    }
    ret = ina226_read_register(INA226_REG_ALERT_LIMIT, &value);
    if (ret != ESP_OK || value != alert_limit) {
        if (ret == ESP_OK) {
            ESP_LOGE(TAG, "alert-limit readback mismatch: wrote 0x%04x, read 0x%04x",
                     alert_limit, value);
        }
        return ret == ESP_OK ? ESP_ERR_INVALID_RESPONSE : ret;
    }
    ret = ina226_read_register(INA226_REG_MASK_ENABLE, &value);
    if (ret != ESP_OK) {
        return ret;
    }
    ina226_capture_alert_bits(value);
    if ((value & INA226_MASK_CONFIGURATION_BITS) != mask_enable) {
        ESP_LOGE(TAG, "mask/enable readback mismatch: wrote 0x%04x, read 0x%04x",
                 mask_enable, value);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t ina226_init(int i2c_port)
{
    esp_err_t ret = ina226_ensure_mutex();
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ina226_lock();
    if (ret != ESP_OK) {
        return ret;
    }

    if (s_initialized || s_bus != NULL || s_device != NULL) {
        ina226_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    s_pending_alert_bits = 0;
    ina226_mark_snapshot_invalid(ESP_ERR_INVALID_STATE);

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    ret = i2c_new_master_bus(&bus_config, &s_bus);
    if (ret != ESP_OK) {
        ina226_mark_snapshot_invalid(ret);
        ina226_unlock();
        return ret;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA226_I2C_ADDRESS,
        .scl_speed_hz = INA226_I2C_FREQUENCY_HZ,
    };
    ret = i2c_master_bus_add_device(s_bus, &device_config, &s_device);
    if (ret != ESP_OK) {
        (void)ina226_release_resources_locked();
        ina226_mark_snapshot_invalid(ret);
        ina226_unlock();
        return ret;
    }

    ret = ina226_verify_identity();
    if (ret == ESP_OK) {
        ret = ina226_apply_configuration();
    }
    if (ret != ESP_OK) {
        esp_err_t cleanup_ret = ina226_release_resources_locked();
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "initialization cleanup failed: %s", esp_err_to_name(cleanup_ret));
        }
        ina226_mark_snapshot_invalid(ret);
        ina226_unlock();
        return ret;
    }

    uint16_t bus_raw = 0;
    uint16_t current_raw = 0;
    ret = ina226_read_measurement_locked(&bus_raw, &current_raw, true);
    if (ret != ESP_OK) {
        esp_err_t cleanup_ret = ina226_release_resources_locked();
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "initial measurement cleanup failed: %s", esp_err_to_name(cleanup_ret));
        }
        ina226_mark_snapshot_invalid(ret);
        ina226_unlock();
        return ret;
    }

    ina226_publish_snapshot(bus_raw, current_raw);
    s_initialized = true;
    ESP_LOGI(TAG, "initialized: 100 kHz, average=16, conversion=588 us, SOL=6 A");
    ina226_unlock();
    return ESP_OK;
}

esp_err_t ina226_deinit(void)
{
    esp_err_t ret = ina226_ensure_mutex();
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ina226_lock();
    if (ret != ESP_OK) {
        return ret;
    }

    if (!s_initialized && s_device == NULL && s_bus == NULL) {
        ina226_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t first_error = ina226_release_resources_locked();
    s_initialized = false;
    s_pending_alert_bits = 0;
    ina226_mark_snapshot_invalid(ESP_ERR_INVALID_STATE);
    ina226_unlock();
    return first_error;
}

esp_err_t ina226_reconfigure(void)
{
    esp_err_t ret = ina226_ensure_mutex();
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ina226_lock();
    if (ret != ESP_OK) {
        return ret;
    }

    if (!s_initialized || s_bus == NULL || s_device == NULL) {
        ina226_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    /* Reset the controller first so a failed transaction cannot leave the bus
       state machine wedged for the next configuration attempt. */
    ret = i2c_master_bus_reset(s_bus);
    if (ret == ESP_OK) {
        ret = ina226_verify_identity();
    }
    if (ret == ESP_OK) {
        ret = ina226_apply_configuration();
    }
    if (ret == ESP_OK) {
        uint16_t bus_raw = 0;
        uint16_t current_raw = 0;
        ret = ina226_read_measurement_locked(&bus_raw, &current_raw, true);
        if (ret == ESP_OK) {
            ina226_publish_snapshot(bus_raw, current_raw);
        }
    }
    if (ret != ESP_OK) {
        ina226_mark_snapshot_invalid(ret);
    }
    ina226_unlock();
    return ret;
}

esp_err_t ina226_update(void)
{
    esp_err_t ret = ina226_ensure_mutex();
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ina226_lock();
    if (ret != ESP_OK) {
        return ret;
    }

    if (!s_initialized) {
        ina226_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t bus_raw = 0;
    uint16_t current_raw = 0;
    ret = ina226_read_measurement_locked(&bus_raw, &current_raw, false);
    if (ret != ESP_OK) {
        ina226_mark_snapshot_invalid(ret);
        ina226_unlock();
        return ret;
    }

    ina226_publish_snapshot(bus_raw, current_raw);
    ina226_unlock();
    return ESP_OK;
}

esp_err_t ina226_get_snapshot(ina226_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_snapshot_lock);
    *snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);
    return ESP_OK;
}

esp_err_t ina226_read_alert_status(ina226_alert_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = ina226_ensure_mutex();
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ina226_lock();
    if (ret != ESP_OK) {
        return ret;
    }
    if (!s_initialized) {
        ina226_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    uint16_t value = 0;
    ret = ina226_read_register(INA226_REG_MASK_ENABLE, &value);
    if (ret != ESP_OK) {
        ina226_unlock();
        return ret;
    }
    ina226_capture_alert_bits(value);
    status->mask_enable = value | s_pending_alert_bits;
    status->alert_function_active = (status->mask_enable & INA226_MASK_AFF) != 0;
    status->conversion_ready = (value & INA226_MASK_CVRF) != 0;
    status->math_overflow = (status->mask_enable & INA226_MASK_OVF) != 0;
    s_pending_alert_bits = 0;
    ina226_unlock();
    return ESP_OK;
}

esp_err_t ina226_clear_alert(void)
{
    ina226_alert_status_t status;
    return ina226_read_alert_status(&status);
}
