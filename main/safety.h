#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "event_defs.h"
#include "ina226.h"
#include "adc_sensors.h"
#include "fan_control.h"
#include "tec_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t reset_reason;
    bool first_power_on;
    bool power_on_signal;
    safety_persistent_state_t persisted_state;
    bool manual_reset_authorized;
} safety_boot_context_t;

typedef struct {
    safety_state_t state;
    safety_fault_reason_t last_fault;
    bool permit;
    bool trip_latched;
    bool dc_in_range;
    bool startup_checks_complete;
    bool alt_latched;
    uint32_t sequence;
    uint32_t last_fault_time_ms;
    esp_err_t last_error;
} safety_status_t;

typedef struct {
    safety_fault_reason_t reason;
    uint32_t timestamp_ms;
    ina226_snapshot_t ina;
    adc_sensor_snapshot_t adc;
    fan_status_t fan1;
    fan_status_t fan2;
    tec_pwm_status_t tec;
} safety_fault_snapshot_t;

typedef struct {
    bool pending;
    safety_persistent_state_t requested_state;
    safety_fault_reason_t reason;
    uint32_t sequence;
} safety_persistence_request_t;

esp_err_t safety_init(const safety_boot_context_t *boot);
/* Apply the state read by the NVS manager after the initial hard-safe setup. */
esp_err_t safety_apply_persisted_state(safety_persistent_state_t state,
                                       bool manual_reset_authorized);
esp_err_t safety_tick_10ms(void);

esp_err_t safety_set_duty(float duty_percent);
esp_err_t safety_emergency_shutdown(safety_fault_reason_t reason);
esp_err_t safety_mark_boot_failed(void);

esp_err_t safety_get_status(safety_status_t *status);
esp_err_t safety_get_fault_snapshot(safety_fault_snapshot_t *snapshot);
esp_err_t safety_get_persistence_request(safety_persistence_request_t *request);
/* A persistence request is acknowledged only after the caller has completed
 * the corresponding NVS read/write successfully. The sequence prevents an
 * older request from acknowledging a newer one. */
esp_err_t safety_ack_persistence_request(uint32_t sequence);

#ifdef __cplusplus
}
#endif
