/**
 ******************************************************************************
 * @file    vl53l7cx_api.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    13 January 2023
 * @brief   Header file for the VL53L7CX main structures.
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

#ifndef VL53L8CX_API_H_
#define VL53L8CX_API_H_

#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION < 6010050)
  #pragma anon_unions
#endif



#include "platform.h"

/**
 * @brief Current driver version.
 */

#define VL53L8CX_API_REVISION     "VL53L8CX_1.0.4"

/**
 * @brief Default I2C address of VL53L8CX sensor. Can be changed using function
 * vl53l8cx_set_i2c_address() function is called.
 */

#define VL53L8CX_DEFAULT_I2C_ADDRESS          ((uint16_t)0x52)

/**
 * @brief Macro VL53L8CX_RESOLUTION_4X4 or VL53L8CX_RESOLUTION_8X8 allows
 * setting sensor in 4x4 mode or 8x8 mode, using function
 * vl53l8cx_set_resolution().
 */

#define VL53L8CX_RESOLUTION_4X4     ((uint8_t) 16U)
#define VL53L8CX_RESOLUTION_8X8     ((uint8_t) 64U)


/**
 * @brief Macro VL53L8CX_TARGET_ORDER_STRONGEST or VL53L8CX_TARGET_ORDER_CLOSEST
 *  are used to select the target order for data output.
 */

#define VL53L8CX_TARGET_ORDER_CLOSEST   ((uint8_t) 1U)
#define VL53L8CX_TARGET_ORDER_STRONGEST   ((uint8_t) 2U)

/**
 * @brief Macro VL53L8CX_RANGING_MODE_CONTINUOUS and
 * VL53L8CX_RANGING_MODE_AUTONOMOUS are used to change the ranging mode.
 * Autonomous mode can be used to set a precise integration time, whereas
 * continuous is always maximum.
 */

#define VL53L8CX_RANGING_MODE_CONTINUOUS  ((uint8_t) 1U)
#define VL53L8CX_RANGING_MODE_AUTONOMOUS  ((uint8_t) 3U)

/**
 * @brief The default power mode is VL53L8CX_POWER_MODE_WAKEUP. User can choose
 * the mode VL53L8CX_POWER_MODE_SLEEP to save power consumption is the device
 * is not used. The low power mode retains the firmware and the configuration.
 * Both modes can be changed using function vl53l8cx_set_power_mode().
 */

#define VL53L8CX_POWER_MODE_SLEEP   ((uint8_t) 0U)
#define VL53L8CX_POWER_MODE_WAKEUP    ((uint8_t) 1U)

/**
 * @brief Macro VL53L8CX_STATUS_OK indicates that VL53L5 sensor has no error.
 * Macro VL53L8CX_STATUS_ERROR indicates that something is wrong (value,
 * I2C access, ...). Macro VL53L8CX_MCU_ERROR is used to indicate a MCU issue.
 */

#define VL53L8CX_STATUS_OK          ((uint8_t) 0U)
#define VL53L8CX_STATUS_TIMEOUT_ERROR   ((uint8_t) 1U)
#define VL53L8CX_STATUS_CORRUPTED_FRAME   ((uint8_t) 2U)
#define VL53L8CX_STATUS_LASER_SAFETY    ((uint8_t) 3U)
#define VL53L8CX_MCU_ERROR          ((uint8_t) 66U)
#define VL53L8CX_STATUS_INVALID_PARAM   ((uint8_t) 127U)
#define VL53L8CX_STATUS_ERROR       ((uint8_t) 255U)

/**
 * @brief Definitions for Range results block headers
 */

#if VL53L8CX_NB_TARGET_PER_ZONE == 1

  #define VL53L8CX_START_BH       ((uint32_t)0x0000000DU)
  #define VL53L8CX_METADATA_BH      ((uint32_t)0x54B400C0U)
  #define VL53L8CX_COMMONDATA_BH      ((uint32_t)0x54C00040U)
  #define VL53L8CX_AMBIENT_RATE_BH    ((uint32_t)0x54D00104U)
  #define VL53L8CX_SPAD_COUNT_BH      ((uint32_t)0x55D00404U)
  #define VL53L8CX_NB_TARGET_DETECTED_BH  ((uint32_t)0xDB840401U)
  #define VL53L8CX_SIGNAL_RATE_BH     ((uint32_t)0xDBC40404U)
  #define VL53L8CX_RANGE_SIGMA_MM_BH    ((uint32_t)0xDEC40402U)
  #define VL53L8CX_DISTANCE_BH      ((uint32_t)0xDF440402U)
  #define VL53L8CX_REFLECTANCE_BH     ((uint32_t)0xE0440401U)
  #define VL53L8CX_TARGET_STATUS_BH   ((uint32_t)0xE0840401U)
  #define VL53L8CX_MOTION_DETECT_BH   ((uint32_t)0xD85808C0U)

  #define VL53L8CX_METADATA_IDX     ((uint16_t)0x54B4U)
  #define VL53L8CX_SPAD_COUNT_IDX     ((uint16_t)0x55D0U)
  #define VL53L8CX_AMBIENT_RATE_IDX   ((uint16_t)0x54D0U)
  #define VL53L8CX_NB_TARGET_DETECTED_IDX ((uint16_t)0xDB84U)
  #define VL53L8CX_SIGNAL_RATE_IDX    ((uint16_t)0xDBC4U)
  #define VL53L8CX_RANGE_SIGMA_MM_IDX   ((uint16_t)0xDEC4U)
  #define VL53L8CX_DISTANCE_IDX     ((uint16_t)0xDF44U)
  #define VL53L8CX_REFLECTANCE_EST_PC_IDX ((uint16_t)0xE044U)
  #define VL53L8CX_TARGET_STATUS_IDX    ((uint16_t)0xE084U)
  #define VL53L8CX_MOTION_DETEC_IDX   ((uint16_t)0xD858U)

#else
  #define VL53L8CX_START_BH       ((uint32_t)0x0000000DU)
  #define VL53L8CX_METADATA_BH      ((uint32_t)0x54B400C0U)
  #define VL53L8CX_COMMONDATA_BH      ((uint32_t)0x54C00040U)
  #define VL53L8CX_AMBIENT_RATE_BH    ((uint32_t)0x54D00104U)
  #define VL53L8CX_NB_TARGET_DETECTED_BH  ((uint32_t)0x57D00401U)
  #define VL53L8CX_SPAD_COUNT_BH      ((uint32_t)0x55D00404U)
  #define VL53L8CX_SIGNAL_RATE_BH     ((uint32_t)0x58900404U)
  #define VL53L8CX_RANGE_SIGMA_MM_BH    ((uint32_t)0x64900402U)
  #define VL53L8CX_DISTANCE_BH      ((uint32_t)0x66900402U)
  #define VL53L8CX_REFLECTANCE_BH     ((uint32_t)0x6A900401U)
  #define VL53L8CX_TARGET_STATUS_BH   ((uint32_t)0x6B900401U)
  #define VL53L8CX_MOTION_DETECT_BH   ((uint32_t)0xCC5008C0U)

  #define VL53L8CX_METADATA_IDX     ((uint16_t)0x54B4U)
  #define VL53L8CX_SPAD_COUNT_IDX     ((uint16_t)0x55D0U)
  #define VL53L8CX_AMBIENT_RATE_IDX   ((uint16_t)0x54D0U)
  #define VL53L8CX_NB_TARGET_DETECTED_IDX ((uint16_t)0x57D0U)
  #define VL53L8CX_SIGNAL_RATE_IDX    ((uint16_t)0x5890U)
  #define VL53L8CX_RANGE_SIGMA_MM_IDX   ((uint16_t)0x6490U)
  #define VL53L8CX_DISTANCE_IDX     ((uint16_t)0x6690U)
  #define VL53L8CX_REFLECTANCE_EST_PC_IDX ((uint16_t)0x6A90U)
  #define VL53L8CX_TARGET_STATUS_IDX    ((uint16_t)0x6B90U)
  #define VL53L8CX_MOTION_DETEC_IDX   ((uint16_t)0xCC50U)
#endif


/**
 * @brief Inner Macro for API. Not for user, only for development.
 */

#define VL53L8CX_NVM_DATA_SIZE      ((uint16_t)492U)
#define VL53L8CX_CONFIGURATION_SIZE   ((uint16_t)972U)
#define VL53L8CX_OFFSET_BUFFER_SIZE   ((uint16_t)488U)
#define VL53L8CX_XTALK_BUFFER_SIZE    ((uint16_t)776U)

#define VL53L8CX_DCI_ZONE_CONFIG    ((uint16_t)0x5450U)
#define VL53L8CX_DCI_FREQ_HZ      ((uint16_t)0x5458U)
#define VL53L8CX_DCI_INT_TIME     ((uint16_t)0x545CU)
#define VL53L8CX_DCI_FW_NB_TARGET   ((uint16_t)0x5478)
#define VL53L8CX_DCI_RANGING_MODE   ((uint16_t)0xAD30U)
#define VL53L8CX_DCI_DSS_CONFIG     ((uint16_t)0xAD38U)
#define VL53L8CX_DCI_TARGET_ORDER   ((uint16_t)0xAE64U)
#define VL53L8CX_DCI_SHARPENER      ((uint16_t)0xAED8U)
#define VL53L8CX_DCI_INTERNAL_CP    ((uint16_t)0xB39CU)
#define VL53L8CX_DCI_SYNC_PIN     ((uint16_t)0xB5F0U)
#define VL53L8CX_DCI_MOTION_DETECTOR_CFG ((uint16_t)0xBFACU)
#define VL53L8CX_DCI_SINGLE_RANGE   ((uint16_t)0xD964U)
#define VL53L8CX_DCI_OUTPUT_CONFIG    ((uint16_t)0xD968U)
#define VL53L8CX_DCI_OUTPUT_ENABLES   ((uint16_t)0xD970U)
#define VL53L8CX_DCI_OUTPUT_LIST    ((uint16_t)0xD980U)
#define VL53L8CX_DCI_PIPE_CONTROL   ((uint16_t)0xDB80U)

#define VL53L8CX_UI_CMD_STATUS      ((uint16_t)0x2C00U)
#define VL53L8CX_UI_CMD_START     ((uint16_t)0x2C04U)
#define VL53L8CX_UI_CMD_END       ((uint16_t)0x2FFFU)

/**
 * @brief Inner values for API. Max buffer size depends of the selected output.
 */

#ifndef VL53L8CX_DISABLE_AMBIENT_PER_SPAD
  #define L5CX_AMB_SIZE 260U
#else
  #define L5CX_AMB_SIZE 0U
#endif

#ifndef VL53L8CX_DISABLE_NB_SPADS_ENABLED
  #define L5CX_SPAD_SIZE  260U
#else
  #define L5CX_SPAD_SIZE  0U
#endif

#ifndef VL53L8CX_DISABLE_NB_TARGET_DETECTED
  #define L5CX_NTAR_SIZE  68U
#else
  #define L5CX_NTAR_SIZE  0U
#endif

#ifndef VL53L8CX_DISABLE_SIGNAL_PER_SPAD
  #define L5CX_SPS_SIZE ((256U * VL53L8CX_NB_TARGET_PER_ZONE) + 4U)
#else
  #define L5CX_SPS_SIZE 0U
#endif

#ifndef VL53L8CX_DISABLE_RANGE_SIGMA_MM
  #define L5CX_SIGR_SIZE ((128U * VL53L8CX_NB_TARGET_PER_ZONE) + 4U)
#else
  #define L5CX_SIGR_SIZE  0U
#endif

#ifndef VL53L8CX_DISABLE_DISTANCE_MM
  #define L5CX_DIST_SIZE ((128U * VL53L8CX_NB_TARGET_PER_ZONE) + 4U)
#else
  #define L5CX_DIST_SIZE  0U
#endif

#ifndef VL53L8CX_DISABLE_REFLECTANCE_PERCENT
  #define L5CX_RFLEST_SIZE ((64U *VL53L8CX_NB_TARGET_PER_ZONE) + 4U)
#else
  #define L5CX_RFLEST_SIZE  0U
#endif

#ifndef VL53L8CX_DISABLE_TARGET_STATUS
  #define L5CX_STA_SIZE ((64U  *VL53L8CX_NB_TARGET_PER_ZONE) + 4U)
#else
  #define L5CX_STA_SIZE 0U
#endif

#ifndef VL53L8CX_DISABLE_MOTION_INDICATOR
  #define L5CX_MOT_SIZE 144U
#else
  #define L5CX_MOT_SIZE 0U
#endif

/**
 * @brief Macro VL53L8CX_MAX_RESULTS_SIZE indicates the maximum size used by
 * output through I2C. Value 40 corresponds to headers + meta-data + common-data
 * and 20 corresponds to the footer.
 */

#define VL53L8CX_MAX_RESULTS_SIZE ( 40U \
  + L5CX_AMB_SIZE + L5CX_SPAD_SIZE + L5CX_NTAR_SIZE + L5CX_SPS_SIZE \
  + L5CX_SIGR_SIZE + L5CX_DIST_SIZE + L5CX_RFLEST_SIZE + L5CX_STA_SIZE \
  + L5CX_MOT_SIZE + 20U)

/**
 * @brief Macro VL53L8CX_TEMPORARY_BUFFER_SIZE can be used to know the size of
 * the temporary buffer. The minimum size is 1024, and the maximum depends of
 * the output configuration.
 */

#if VL53L8CX_MAX_RESULTS_SIZE < 1024U
  #define VL53L8CX_TEMPORARY_BUFFER_SIZE ((uint32_t) 1024U)
#else
  #define VL53L8CX_TEMPORARY_BUFFER_SIZE ((uint32_t) VL53L8CX_MAX_RESULTS_SIZE)
#endif


/**
 * @brief Structure VL53L8CX_Configuration contains the sensor configuration.
 * User MUST not manually change these field, except for the sensor address.
 */

typedef struct {
  /* Platform, filled by customer into the 'platform.h' file */
  VL53L8CX_Platform platform;
  /* Results streamcount, value auto-incremented at each range */
  uint8_t           streamcount;
  /* Size of data read though I2C */
  uint32_t          data_read_size;
  /* Address of default configuration buffer */
  uint8_t           *default_configuration;
  /* Address of default Xtalk buffer */
  uint8_t           *default_xtalk;
  /* Offset buffer */
  uint8_t           offset_data[VL53L8CX_OFFSET_BUFFER_SIZE];
  /* Xtalk buffer */
  uint8_t           xtalk_data[VL53L8CX_XTALK_BUFFER_SIZE];
  /* Temporary buffer used for internal driver processing */
  uint8_t         temp_buffer[VL53L8CX_TEMPORARY_BUFFER_SIZE];
  /* Auto-stop flag for stopping the sensor */
  uint8_t       is_auto_stop_enabled;
} VL53L8CX_Configuration;


/**
 * @brief Structure VL53L8CX_ResultsData contains the ranging results of
 * VL53L8CX. If user wants more than 1 target per zone, the results can be split
 * into 2 sub-groups :
 * - Per zone results. These results are common to all targets (ambient_per_spad
 * , nb_target_detected and nb_spads_enabled).
 * - Per target results : These results are different relative to the detected
 * target (signal_per_spad, range_sigma_mm, distance_mm, reflectance,
 * target_status).
 */

typedef struct {
  /* Internal sensor silicon temperature */
  int8_t silicon_temp_degc;

  /* Ambient noise in kcps/spads */
#ifndef VL53L8CX_DISABLE_AMBIENT_PER_SPAD
  uint32_t ambient_per_spad[VL53L8CX_RESOLUTION_8X8];
#endif

  /* Number of valid target detected for 1 zone */
#ifndef VL53L8CX_DISABLE_NB_TARGET_DETECTED
  uint8_t nb_target_detected[VL53L8CX_RESOLUTION_8X8];
#endif

  /* Number of spads enabled for this ranging */
#ifndef VL53L8CX_DISABLE_NB_SPADS_ENABLED
  uint32_t nb_spads_enabled[VL53L8CX_RESOLUTION_8X8];
#endif

  /* Signal returned to the sensor in kcps/spads */
#ifndef VL53L8CX_DISABLE_SIGNAL_PER_SPAD
  uint32_t signal_per_spad[(VL53L8CX_RESOLUTION_8X8
                            *VL53L8CX_NB_TARGET_PER_ZONE)];
#endif

  /* Sigma of the current distance in mm */
#ifndef VL53L8CX_DISABLE_RANGE_SIGMA_MM
  uint16_t range_sigma_mm[(VL53L8CX_RESOLUTION_8X8
                           *VL53L8CX_NB_TARGET_PER_ZONE)];
#endif

  /* Measured distance in mm */
#ifndef VL53L8CX_DISABLE_DISTANCE_MM
  int16_t distance_mm[(VL53L8CX_RESOLUTION_8X8
                       *VL53L8CX_NB_TARGET_PER_ZONE)];
#endif

  /* Estimated reflectance in percent */
#ifndef VL53L8CX_DISABLE_REFLECTANCE_PERCENT
  uint8_t reflectance[(VL53L8CX_RESOLUTION_8X8
                       *VL53L8CX_NB_TARGET_PER_ZONE)];
#endif

  /* Status indicating the measurement validity (5 & 9 means ranging OK)*/
#ifndef VL53L8CX_DISABLE_TARGET_STATUS
  uint8_t target_status[(VL53L8CX_RESOLUTION_8X8
                         *VL53L8CX_NB_TARGET_PER_ZONE)];
#endif

  /* Motion detector results */
#ifndef VL53L8CX_DISABLE_MOTION_INDICATOR
  struct {
    uint32_t global_indicator_1;
    uint32_t global_indicator_2;
    uint8_t  status;
    uint8_t  nb_of_detected_aggregates;
    uint8_t  nb_of_aggregates;
    uint8_t  spare;
    uint32_t motion[32];
  } motion_indicator;
#endif

} VL53L8CX_ResultsData;


union Block_header {
  uint32_t bytes;
  struct {
    uint32_t type : 4;
    uint32_t size : 12;
    uint32_t idx : 16;
  };
};

#endif //VL53L8CX_API_H_
