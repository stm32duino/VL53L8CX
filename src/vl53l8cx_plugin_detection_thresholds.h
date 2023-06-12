/**
 ******************************************************************************
 * @file    vl53l8cx_plugin_detection_thresholds.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    13 January 2023
 * @brief   Header file for the VL53L8CX thresholds detection structures.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#ifndef VL53L8CX_PLUGIN_DETECTION_THRESHOLDS_H_
#define VL53L8CX_PLUGIN_DETECTION_THRESHOLDS_H_

#include "vl53l8cx_class.h"

/**
 * @brief Macro VL53L8CX_NB_THRESHOLDS indicates the number of checkers. This
 * value cannot be changed.
 */

#define VL53L8CX_NB_THRESHOLDS        ((uint8_t)64U)

/**
 * @brief Inner Macro for API. Not for user, only for development.
 */

#define VL53L8CX_DCI_DET_THRESH_CONFIG      ((uint16_t)0x5488U)
#define VL53L8CX_DCI_DET_THRESH_GLOBAL_CONFIG   ((uint16_t)0xB6E0U)
#define VL53L8CX_DCI_DET_THRESH_START     ((uint16_t)0xB6E8U)
#define VL53L8CX_DCI_DET_THRESH_VALID_STATUS    ((uint16_t)0xB9F0U)

/**
 * @brief Macro VL53L8CX_LAST_THRESHOLD is used to indicate the end of checkers
 * programming.
 */

#define VL53L8CX_LAST_THRESHOLD       ((uint8_t)128U)

/**
 * @brief The following macro are used to define the 'param_type' of a checker.
 * They indicate what is the measurement to catch.
 */

#define VL53L8CX_DISTANCE_MM        ((uint8_t)1U)
#define VL53L8CX_SIGNAL_PER_SPAD_KCPS             ((uint8_t)2U)
#define VL53L8CX_RANGE_SIGMA_MM       ((uint8_t)4U)
#define VL53L8CX_AMBIENT_PER_SPAD_KCPS      ((uint8_t)8U)
#define VL53L8CX_NB_TARGET_DETECTED     ((uint8_t)9U)
#define VL53L8CX_TARGET_STATUS        ((uint8_t)12U)
#define VL53L8CX_NB_SPADS_ENABLED     ((uint8_t)13U)
#define VL53L8CX_MOTION_INDICATOR             ((uint8_t)19U)

/**
 * @brief The following macro are used to define the 'type' of a checker.
 * They indicate the window of measurements, defined by low and a high
 * thresholds.
 */

#define VL53L8CX_IN_WINDOW        ((uint8_t)0U)
#define VL53L8CX_OUT_OF_WINDOW        ((uint8_t)1U)
#define VL53L8CX_LESS_THAN_EQUAL_MIN_CHECKER    ((uint8_t)2U)
#define VL53L8CX_GREATER_THAN_MAX_CHECKER   ((uint8_t)3U)
#define VL53L8CX_EQUAL_MIN_CHECKER      ((uint8_t)4U)
#define VL53L8CX_NOT_EQUAL_MIN_CHECKER      ((uint8_t)5U)

/**
 * @brief The following macro are used to define multiple checkers in the same
 * zone, using operators. Please note that the first checker MUST always be a OR
 * operation.
 */

#define VL53L8CX_OPERATION_NONE       ((uint8_t)0U)
#define VL53L8CX_OPERATION_OR       ((uint8_t)0U)
#define VL53L8CX_OPERATION_AND        ((uint8_t)2U)

/**
 * @brief Structure VL53L8CX_DetectionThresholds contains a single threshold.
 * This structure  is never used alone, it must be used as an array of 64
 * thresholds (defined by macro VL53L8CX_NB_THRESHOLDS).
 */

typedef struct {

  /* Low threshold */
  int32_t   param_low_thresh;
  /* High threshold */
  int32_t   param_high_thresh;
  /* Measurement to catch (VL53L8CX_MEDIAN_RANGE_MM,...)*/
  uint8_t   measurement;
  /* Windows type (VL53L8CX_IN_WINDOW, VL53L8CX_OUT_WINDOW, ...) */
  uint8_t   type;
  /* Zone id. Please read VL53L5 user manual to find the zone id.Set macro
   * VL53L8CX_LAST_THRESHOLD to indicates the end of checkers */
  uint8_t   zone_num;
  /* Mathematics operation (AND/OR). The first threshold is always OR.*/
  uint8_t   mathematic_operation;
} VL53L8CX_DetectionThresholds;


#endif /* VL53L8CX_PLUGIN_DETECTION_THRESHOLDS_H_ */
