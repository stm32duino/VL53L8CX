/**
 ******************************************************************************
 * @file    vl53l8cx_plugin_motion_indicator.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    13 January 2023
 * @brief   Header file for the VL53L8CX motion indicator structures.
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

#ifndef VL53L8CX_PLUGIN_MOTION_INDICATOR_H_
#define VL53L8CX_PLUGIN_MOTION_INDICATOR_H_

#include "vl53l8cx_class.h"

/**
 * @brief Motion indicator internal configuration structure.
 */

typedef struct {
  int32_t  ref_bin_offset;
  uint32_t detection_threshold;
  uint32_t extra_noise_sigma;
  uint32_t null_den_clip_value;
  uint8_t  mem_update_mode;
  uint8_t  mem_update_choice;
  uint8_t  sum_span;
  uint8_t  feature_length;
  uint8_t  nb_of_aggregates;
  uint8_t  nb_of_temporal_accumulations;
  uint8_t  min_nb_for_global_detection;
  uint8_t  global_indicator_format_1;
  uint8_t  global_indicator_format_2;
  uint8_t  spare_1;
  uint8_t  spare_2;
  uint8_t  spare_3;
  int8_t   map_id[64];
  uint8_t  indicator_format_1[32];
  uint8_t  indicator_format_2[32];
} VL53L8CX_Motion_Configuration;

#endif /* VL53L8CX_PLUGIN_MOTION_INDICATOR_H_ */
