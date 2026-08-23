#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float air_temp_c;
    float cold_temp_c;
    float hot_temp_c;
    float tec_n_voltage_v;

    bool air_valid;
    bool cold_valid;
    bool hot_valid;
    bool tec_n_valid;

    uint32_t sample_time_ms;
    uint32_t sequence;
} adc_sensor_snapshot_t;

esp_err_t adc_sensors_init(void);
esp_err_t adc_sensors_deinit(void);

/* Sample all four channels once and atomically replace the cached snapshot. */
esp_err_t adc_sensors_update(void);
esp_err_t adc_sensors_get_snapshot(adc_sensor_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
