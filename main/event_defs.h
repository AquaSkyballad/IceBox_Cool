/**
 * @file    event_defs.h
 * @brief   系统事件类型定义
 *
 */

#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  事件类型
 * ================================================================ */

typedef enum {
    SAFETY_STATE_BOOT_SAFE = 0,
    SAFETY_STATE_NO_DC,
    SAFETY_STATE_POWER_INVALID,
    SAFETY_STATE_POWER_SELF_TEST,
    SAFETY_STATE_READY,
    SAFETY_STATE_RUNNING,
    SAFETY_STATE_TRIPPED,
    SAFETY_STATE_BOOT_FAILED,
} safety_state_t;

typedef enum {
    SAFETY_FAULT_NONE = 0,
    SAFETY_FAULT_ALT,
    SAFETY_FAULT_OVERCURRENT,
    SAFETY_FAULT_OVERVOLTAGE,
    SAFETY_FAULT_LOW_VOLTAGE,
    SAFETY_FAULT_INA_COMMUNICATION,
    SAFETY_FAULT_HOT_INVALID,
    SAFETY_FAULT_HOT_OVERTEMPERATURE,
    SAFETY_FAULT_HOT_STALE,
    SAFETY_FAULT_COLD_INVALID,
    SAFETY_FAULT_FAN1_STALL,
    SAFETY_FAULT_FAN2_STALL,
    SAFETY_FAULT_TEC_OPEN,
    SAFETY_FAULT_STARTUP_CURRENT,
    SAFETY_FAULT_STARTUP_TIMEOUT,
    SAFETY_FAULT_PWM_ERROR,
    SAFETY_FAULT_BOOT_FAILURE,
} safety_fault_reason_t;

typedef enum {
    SAFETY_PERSIST_NORMAL = 0,
    SAFETY_PERSIST_RETRY_PENDING,
    SAFETY_PERSIST_LOCKED,
} safety_persistent_state_t;

#ifdef __cplusplus
}
#endif
