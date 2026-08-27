#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INA226_I2C_ADDRESS          0x40
#define INA226_I2C_FREQUENCY_HZ     100000U
#define INA226_I2C_TIMEOUT_MS       50U

#define INA226_SHUNT_RESISTOR_OHM   0.002f
#define INA226_CURRENT_LSB_A        0.001f
#define INA226_CALIBRATION_VALUE    0x0A00U

typedef struct {
    float bus_voltage_v;
    float current_a;
    /* true means the values came from a completed INA226 conversion. DC range,
       stability, NO_DC, and fault severity are interpreted by the safety layer. */
    bool valid;
    uint32_t timestamp_ms;
    uint32_t sequence;
    esp_err_t last_error;
} ina226_snapshot_t;

typedef struct {
    uint16_t mask_enable;
    bool alert_function_active;
    bool conversion_ready;
    bool math_overflow;
} ina226_alert_status_t;

esp_err_t ina226_init(int i2c_port);
esp_err_t ina226_deinit(void);

/* Read one completed conversion result and replace the cached snapshot.
   Returns ESP_ERR_NOT_FINISHED when CVRF has not asserted yet; in that case
   the previous completed snapshot remains available unchanged. */
esp_err_t ina226_update(void);
esp_err_t ina226_get_snapshot(ina226_snapshot_t *snapshot);

/* Re-probe, restore volatile configuration, and publish one valid sample. */
esp_err_t ina226_reconfigure(void);

/* Read/clear the latched alert status. Reading Mask/Enable clears AFF; the
   driver preserves an AFF observed while checking conversion readiness. */
esp_err_t ina226_read_alert_status(ina226_alert_status_t *status);
esp_err_t ina226_clear_alert(void);

#ifdef __cplusplus
}
#endif
