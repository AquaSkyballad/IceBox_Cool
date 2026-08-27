/**
 * @file board_config.h
 * @brief Board-specific hardware and power-stage constants.
 *
 * Values in this file are part of the board contract and are intentionally
 * not exposed as ordinary Kconfig options.
 */

#pragma once

/* TEC PWM safety profile. */
#define TEC_PWM_UPDATE_PERIOD_MS     10U
#define TEC_PWM_RAMP_STEP_PERCENT    1.0f
#define TEC_PWM_ENABLE_PERCENT       6.0f
#define TEC_PWM_DISABLE_PERCENT      4.0f

/* Startup TEC connectivity checks. */
#define SAFETY_TEC_CONNECT_VALID_SAMPLES 3U
#define SAFETY_TEC_CLOSED_MAX_DIFF_V     1.0f
#define SAFETY_TEC_OPEN_MAX_ABS_V        1.0f
