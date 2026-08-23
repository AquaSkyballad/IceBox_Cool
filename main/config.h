/**
 * @file    config.h
 * @brief   系统配置参数和硬件常量定义
 *
 * 集中管理所有可配置参数,提供默认值用于故障恢复
 */

#pragma once

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ADC采样 */
#define ADC_SENSOR_SAMPLE_COUNT          16U
#define ADC_SENSOR_MAX_READ_ATTEMPTS     20U
#define ADC_SENSOR_BOARD_SUPPLY_MV       3300

/* 10kΩ B3950 NTC，基准点为25°C。 */
#define ADC_NTC_NOMINAL_RESISTANCE_OHM   10000.0f
#define ADC_NTC_BETA_K                   3950.0f
#define ADC_NTC_REFERENCE_TEMP_K         298.15f
#define ADC_AIR_PULLUP_RESISTANCE_OHM    33000.0f
#define ADC_COLD_PULLUP_RESISTANCE_OHM   33000.0f
#define ADC_HOT_PULLUP_RESISTANCE_OHM    3300.0f

/* TEC-N偏置分压网络：V_N = (V_ADC - offset) / scale。 */
#define ADC_TEC_N_OFFSET_V               0.18469f
#define ADC_TEC_N_SCALE                  0.12313f

/*
 * 首版只排除电源轨端点和非有限计算结果。样机标定后在这里收紧各路
 * 校准电压与物理量范围，ADC驱动内不增加分散阈值。
 */
#define ADC_AIR_VALID_MV_MIN             0
#define ADC_AIR_VALID_MV_MAX             ADC_SENSOR_BOARD_SUPPLY_MV
#define ADC_COLD_VALID_MV_MIN            0
#define ADC_COLD_VALID_MV_MAX            ADC_SENSOR_BOARD_SUPPLY_MV
#define ADC_HOT_VALID_MV_MIN             0
#define ADC_HOT_VALID_MV_MAX             ADC_SENSOR_BOARD_SUPPLY_MV
#define ADC_TEC_N_VALID_MV_MIN            0
#define ADC_TEC_N_VALID_MV_MAX            ADC_SENSOR_BOARD_SUPPLY_MV

#define ADC_AIR_VALID_TEMP_C_MIN         (-INFINITY)
#define ADC_AIR_VALID_TEMP_C_MAX         INFINITY
#define ADC_COLD_VALID_TEMP_C_MIN        (-INFINITY)
#define ADC_COLD_VALID_TEMP_C_MAX        INFINITY
#define ADC_HOT_VALID_TEMP_C_MIN         (-INFINITY)
#define ADC_HOT_VALID_TEMP_C_MAX         INFINITY
#define ADC_TEC_N_VALID_VOLTAGE_V_MIN    (-INFINITY)
#define ADC_TEC_N_VALID_VOLTAGE_V_MAX    INFINITY

#ifdef __cplusplus
}
#endif
