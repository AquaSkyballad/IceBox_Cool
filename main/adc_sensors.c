#include "adc_sensors.h"

#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "config.h"
#include "gpio_def.h"

_Static_assert(ADC_SENSOR_SAMPLE_COUNT > 0U, "ADC sample count must be non-zero");
_Static_assert(ADC_SENSOR_MAX_READ_ATTEMPTS >= ADC_SENSOR_SAMPLE_COUNT,
               "ADC read attempts must cover the requested samples");

static const char *TAG = "adc_sensors";

typedef enum {
    ADC_SENSOR_CHANNEL_AIR = 0,
    ADC_SENSOR_CHANNEL_COLD,
    ADC_SENSOR_CHANNEL_HOT,
    ADC_SENSOR_CHANNEL_TEC_N,
    ADC_SENSOR_CHANNEL_COUNT,
} adc_sensor_channel_id_t;

typedef enum {
    ADC_DIAG_NOT_SAMPLED = 0,
    ADC_DIAG_OK,
    ADC_DIAG_READ_FAILED,
    ADC_DIAG_CALIBRATION_FAILED,
    ADC_DIAG_VOLTAGE_OUT_OF_RANGE,
    ADC_DIAG_MATH_INVALID,
    ADC_DIAG_PHYSICAL_VALUE_OUT_OF_RANGE,
} adc_sensor_diag_error_t;

typedef struct {
    int average_raw;
    int voltage_mv;
    uint32_t successful_samples;
    adc_sensor_diag_error_t error;
    esp_err_t driver_error;
} adc_sensor_channel_diag_t;

static const adc_channel_t s_channels[ADC_SENSOR_CHANNEL_COUNT] = {
    AIR_ADC_CHANNEL,
    COLD_ADC_CHANNEL,
    HOT_ADC_CHANNEL,
    G_ADC_CHANNEL,
};

static const char *const s_channel_names[ADC_SENSOR_CHANNEL_COUNT] = {
    "AIR",
    "COLD",
    "HOT",
    "TEC_N",
};

static adc_oneshot_unit_handle_t s_adc_unit;
static adc_cali_handle_t s_calibration;
static portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static adc_sensor_snapshot_t s_snapshot;
static adc_sensor_channel_diag_t s_diagnostics[ADC_SENSOR_CHANNEL_COUNT];
static bool s_initialized;

static uint32_t adc_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static void adc_reset_snapshot(void)
{
    const adc_sensor_snapshot_t invalid_snapshot = {
        .air_temp_c = NAN,
        .cold_temp_c = NAN,
        .hot_temp_c = NAN,
        .tec_n_voltage_v = NAN,
        .sample_time_ms = adc_now_ms(),
    };

    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot = invalid_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

static const char *adc_diag_error_name(adc_sensor_diag_error_t error)
{
    switch (error) {
    case ADC_DIAG_NOT_SAMPLED:
        return "not sampled";
    case ADC_DIAG_OK:
        return "ok";
    case ADC_DIAG_READ_FAILED:
        return "read failed";
    case ADC_DIAG_CALIBRATION_FAILED:
        return "calibration failed";
    case ADC_DIAG_VOLTAGE_OUT_OF_RANGE:
        return "voltage out of range";
    case ADC_DIAG_MATH_INVALID:
        return "invalid math result";
    case ADC_DIAG_PHYSICAL_VALUE_OUT_OF_RANGE:
        return "physical value out of range";
    default:
        return "unknown";
    }
}

static void adc_update_diagnostic(adc_sensor_channel_id_t channel_id,
                                  const adc_sensor_channel_diag_t *diagnostic)
{
    const adc_sensor_channel_diag_t previous = s_diagnostics[channel_id];
    s_diagnostics[channel_id] = *diagnostic;

    if (previous.error == diagnostic->error &&
        previous.driver_error == diagnostic->driver_error) {
        return;
    }
    if (diagnostic->error == ADC_DIAG_OK) {
        if (previous.error != ADC_DIAG_NOT_SAMPLED) {
            ESP_LOGI(TAG, "%s channel recovered", s_channel_names[channel_id]);
        }
        return;
    }

    ESP_LOGW(TAG,
             "%s invalid: %s, samples=%" PRIu32 ", raw=%d, mv=%d, driver=%s",
             s_channel_names[channel_id],
             adc_diag_error_name(diagnostic->error),
             diagnostic->successful_samples,
             diagnostic->average_raw,
             diagnostic->voltage_mv,
             esp_err_to_name(diagnostic->driver_error));
}

static esp_err_t adc_sample_channel(adc_sensor_channel_id_t channel_id,
                                    adc_sensor_channel_diag_t *diagnostic)
{
    memset(diagnostic, 0, sizeof(*diagnostic));
    diagnostic->error = ADC_DIAG_READ_FAILED;
    diagnostic->driver_error = ESP_ERR_TIMEOUT;

    int discarded_raw = 0;
    /* The first conversion after a channel switch is intentionally discarded.
       A failure here does not consume one of the 16 valid samples; the bounded
       loop below still decides whether this channel can publish a result. */
    (void)adc_oneshot_read(s_adc_unit, s_channels[channel_id], &discarded_raw);

    uint32_t raw_sum = 0;
    esp_err_t last_read_error = ESP_OK;
    for (uint32_t attempt = 0;
         attempt < ADC_SENSOR_MAX_READ_ATTEMPTS &&
         diagnostic->successful_samples < ADC_SENSOR_SAMPLE_COUNT;
         ++attempt) {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(s_adc_unit, s_channels[channel_id], &raw);
        if (ret != ESP_OK) {
            last_read_error = ret;
            continue;
        }
        raw_sum += (uint32_t)raw;
        diagnostic->successful_samples++;
    }

    if (diagnostic->successful_samples < ADC_SENSOR_SAMPLE_COUNT) {
        diagnostic->driver_error = last_read_error == ESP_OK ? ESP_ERR_TIMEOUT : last_read_error;
        return diagnostic->driver_error;
    }

    diagnostic->average_raw = (int)(raw_sum / ADC_SENSOR_SAMPLE_COUNT);
    esp_err_t ret = adc_cali_raw_to_voltage(
        s_calibration, diagnostic->average_raw, &diagnostic->voltage_mv);
    if (ret != ESP_OK) {
        diagnostic->error = ADC_DIAG_CALIBRATION_FAILED;
        diagnostic->driver_error = ret;
        return ret;
    }

    diagnostic->error = ADC_DIAG_OK;
    diagnostic->driver_error = ESP_OK;
    return ESP_OK;
}

static esp_err_t adc_convert_ntc(const adc_sensor_channel_diag_t *diagnostic,
                                 float pullup_resistance_ohm,
                                 int valid_mv_min,
                                 int valid_mv_max,
                                 float valid_temp_min_c,
                                 float valid_temp_max_c,
                                 float *temperature_c,
                                 adc_sensor_diag_error_t *conversion_error)
{
    const int voltage_mv = diagnostic->voltage_mv;
    if (voltage_mv <= valid_mv_min || voltage_mv >= valid_mv_max ||
        voltage_mv >= ADC_SENSOR_BOARD_SUPPLY_MV) {
        *conversion_error = ADC_DIAG_VOLTAGE_OUT_OF_RANGE;
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float denominator_mv = (float)(ADC_SENSOR_BOARD_SUPPLY_MV - voltage_mv);
    if (!(denominator_mv > 0.0f)) {
        *conversion_error = ADC_DIAG_MATH_INVALID;
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float resistance_ohm = pullup_resistance_ohm * (float)voltage_mv /
                                 denominator_mv;
    if (!(resistance_ohm > 0.0f) || !isfinite(resistance_ohm)) {
        *conversion_error = ADC_DIAG_MATH_INVALID;
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float logarithm = logf(resistance_ohm / ADC_NTC_NOMINAL_RESISTANCE_OHM);
    const float reciprocal_temp = (1.0f / ADC_NTC_REFERENCE_TEMP_K) +
                                  logarithm / ADC_NTC_BETA_K;
    if (!isfinite(logarithm) || !(reciprocal_temp > 0.0f) ||
        !isfinite(reciprocal_temp)) {
        *conversion_error = ADC_DIAG_MATH_INVALID;
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float result_c = (1.0f / reciprocal_temp) - 273.15f;
    if (!isfinite(result_c)) {
        *conversion_error = ADC_DIAG_MATH_INVALID;
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (result_c < valid_temp_min_c || result_c > valid_temp_max_c) {
        *conversion_error = ADC_DIAG_PHYSICAL_VALUE_OUT_OF_RANGE;
        return ESP_ERR_INVALID_RESPONSE;
    }

    *temperature_c = result_c;
    *conversion_error = ADC_DIAG_OK;
    return ESP_OK;
}

static esp_err_t adc_convert_tec_n(const adc_sensor_channel_diag_t *diagnostic,
                                   float *voltage_v,
                                   adc_sensor_diag_error_t *conversion_error)
{
    if (diagnostic->voltage_mv <= ADC_TEC_N_VALID_MV_MIN ||
        diagnostic->voltage_mv >= ADC_TEC_N_VALID_MV_MAX) {
        *conversion_error = ADC_DIAG_VOLTAGE_OUT_OF_RANGE;
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float adc_voltage_v = (float)diagnostic->voltage_mv / 1000.0f;
    const float result_v = (adc_voltage_v - ADC_TEC_N_OFFSET_V) / ADC_TEC_N_SCALE;
    if (!isfinite(result_v)) {
        *conversion_error = ADC_DIAG_MATH_INVALID;
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (result_v < ADC_TEC_N_VALID_VOLTAGE_V_MIN ||
        result_v > ADC_TEC_N_VALID_VOLTAGE_V_MAX) {
        *conversion_error = ADC_DIAG_PHYSICAL_VALUE_OUT_OF_RANGE;
        return ESP_ERR_INVALID_RESPONSE;
    }

    *voltage_v = result_v;
    *conversion_error = ADC_DIAG_OK;
    return ESP_OK;
}

static esp_err_t adc_release_resources(void)
{
    esp_err_t first_error = ESP_OK;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (s_calibration != NULL) {
        esp_err_t ret = adc_cali_delete_scheme_curve_fitting(s_calibration);
        if (ret != ESP_OK) {
            first_error = ret;
        }
        s_calibration = NULL;
    }
#endif

    if (s_adc_unit != NULL) {
        esp_err_t ret = adc_oneshot_del_unit(s_adc_unit);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        s_adc_unit = NULL;
    }
    return first_error;
}

esp_err_t adc_sensors_init(void)
{
    if (s_initialized || s_adc_unit != NULL || s_calibration != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    adc_reset_snapshot();
    memset(s_diagnostics, 0, sizeof(s_diagnostics));

    const adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&unit_config, &s_adc_unit);
    if (ret != ESP_OK) {
        return ret;
    }

    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    for (size_t i = 0; i < ADC_SENSOR_CHANNEL_COUNT; ++i) {
        ret = adc_oneshot_config_channel(s_adc_unit, s_channels[i], &channel_config);
        if (ret != ESP_OK) {
            (void)adc_release_resources();
            return ret;
        }
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    const adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = ADC_UNIT_1,
        .chan = AIR_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&calibration_config, &s_calibration);
#else
    ret = ESP_ERR_NOT_SUPPORTED;
#endif
    if (ret != ESP_OK) {
        (void)adc_release_resources();
        return ret;
    }

    s_initialized = true;
    ret = adc_sensors_update();
    if (ret != ESP_OK) {
        s_initialized = false;
        (void)adc_release_resources();
        adc_reset_snapshot();
        return ret;
    }

    ESP_LOGI(TAG, "initialized: ADC1 oneshot, 12 dB, average=%u",
             ADC_SENSOR_SAMPLE_COUNT);
    return ESP_OK;
}

esp_err_t adc_sensors_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_initialized = false;
    esp_err_t ret = adc_release_resources();
    memset(s_diagnostics, 0, sizeof(s_diagnostics));
    adc_reset_snapshot();
    return ret;
}

esp_err_t adc_sensors_update(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    adc_sensor_snapshot_t next_snapshot = {
        .air_temp_c = NAN,
        .cold_temp_c = NAN,
        .hot_temp_c = NAN,
        .tec_n_voltage_v = NAN,
    };
    adc_sensor_channel_diag_t diagnostics[ADC_SENSOR_CHANNEL_COUNT];
    esp_err_t first_error = ESP_OK;

    for (size_t i = 0; i < ADC_SENSOR_CHANNEL_COUNT; ++i) {
        esp_err_t ret = adc_sample_channel((adc_sensor_channel_id_t)i, &diagnostics[i]);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
    }

    adc_sensor_diag_error_t conversion_error = ADC_DIAG_OK;
    if (diagnostics[ADC_SENSOR_CHANNEL_AIR].error == ADC_DIAG_OK) {
        esp_err_t ret = adc_convert_ntc(&diagnostics[ADC_SENSOR_CHANNEL_AIR],
                                        ADC_AIR_PULLUP_RESISTANCE_OHM,
                                        ADC_AIR_VALID_MV_MIN,
                                        ADC_AIR_VALID_MV_MAX,
                                        ADC_AIR_VALID_TEMP_C_MIN,
                                        ADC_AIR_VALID_TEMP_C_MAX,
                                        &next_snapshot.air_temp_c,
                                        &conversion_error);
        diagnostics[ADC_SENSOR_CHANNEL_AIR].error = conversion_error;
        next_snapshot.air_valid = ret == ESP_OK;
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        if (ret != ESP_OK) {
            diagnostics[ADC_SENSOR_CHANNEL_AIR].driver_error = ret;
        }
    }

    if (diagnostics[ADC_SENSOR_CHANNEL_COLD].error == ADC_DIAG_OK) {
        esp_err_t ret = adc_convert_ntc(&diagnostics[ADC_SENSOR_CHANNEL_COLD],
                                        ADC_COLD_PULLUP_RESISTANCE_OHM,
                                        ADC_COLD_VALID_MV_MIN,
                                        ADC_COLD_VALID_MV_MAX,
                                        ADC_COLD_VALID_TEMP_C_MIN,
                                        ADC_COLD_VALID_TEMP_C_MAX,
                                        &next_snapshot.cold_temp_c,
                                        &conversion_error);
        diagnostics[ADC_SENSOR_CHANNEL_COLD].error = conversion_error;
        next_snapshot.cold_valid = ret == ESP_OK;
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        if (ret != ESP_OK) {
            diagnostics[ADC_SENSOR_CHANNEL_COLD].driver_error = ret;
        }
    }

    if (diagnostics[ADC_SENSOR_CHANNEL_HOT].error == ADC_DIAG_OK) {
        esp_err_t ret = adc_convert_ntc(&diagnostics[ADC_SENSOR_CHANNEL_HOT],
                                        ADC_HOT_PULLUP_RESISTANCE_OHM,
                                        ADC_HOT_VALID_MV_MIN,
                                        ADC_HOT_VALID_MV_MAX,
                                        ADC_HOT_VALID_TEMP_C_MIN,
                                        ADC_HOT_VALID_TEMP_C_MAX,
                                        &next_snapshot.hot_temp_c,
                                        &conversion_error);
        diagnostics[ADC_SENSOR_CHANNEL_HOT].error = conversion_error;
        next_snapshot.hot_valid = ret == ESP_OK;
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        if (ret != ESP_OK) {
            diagnostics[ADC_SENSOR_CHANNEL_HOT].driver_error = ret;
        }
    }

    if (diagnostics[ADC_SENSOR_CHANNEL_TEC_N].error == ADC_DIAG_OK) {
        esp_err_t ret = adc_convert_tec_n(&diagnostics[ADC_SENSOR_CHANNEL_TEC_N],
                                          &next_snapshot.tec_n_voltage_v,
                                          &conversion_error);
        diagnostics[ADC_SENSOR_CHANNEL_TEC_N].error = conversion_error;
        next_snapshot.tec_n_valid = ret == ESP_OK;
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        if (ret != ESP_OK) {
            diagnostics[ADC_SENSOR_CHANNEL_TEC_N].driver_error = ret;
        }
    }

    next_snapshot.sample_time_ms = adc_now_ms();
    portENTER_CRITICAL(&s_snapshot_lock);
    next_snapshot.sequence = s_snapshot.sequence + 1U;
    s_snapshot = next_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);

    for (size_t i = 0; i < ADC_SENSOR_CHANNEL_COUNT; ++i) {
        adc_update_diagnostic((adc_sensor_channel_id_t)i, &diagnostics[i]);
    }
    return first_error;
}

esp_err_t adc_sensors_get_snapshot(adc_sensor_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_snapshot_lock);
    *snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);
    return ESP_OK;
}
