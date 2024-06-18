/**
 ******************************************************************************
 * @file    vl53l8cx_class.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    13 January 2023
 * @brief   Abstract Class for VL53L8CX sensor.
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

#ifndef __VL53L8CX_CLASS_H
#define __VL53L8CX_CLASS_H

/* Includes ------------------------------------------------------------------*/
#include "Arduino.h"
#include "platform.h"
#include "vl53l8cx_api.h"
#include "vl53l8cx_plugin_detection_thresholds.h"
#include "vl53l8cx_plugin_motion_indicator.h"
#include "vl53l8cx_plugin_xtalk.h"




/* Classes -------------------------------------------------------------------*/
/** Class representing a VL53LX sensor component
 */

class VL53L8CX {
  public:
    /** Constructor
     * @param[in] i2c device I2C to be used for communication
     * @param[in] lpn_pin pin to be used as component LPn
     * @param[in] i2c_rst_pin pin to be used as component I2C_RST
     */
    VL53L8CX(TwoWire *i2c, int lpn_pin, int i2c_rst_pin = -1)
    {
      memset((void *)&_dev, 0x0, sizeof(VL53L8CX_Configuration));
      _dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
      _dev.platform.dev_i2c = i2c;
      _dev.platform.dev_spi = NULL;
      _dev.platform.lpn_pin = lpn_pin;
      _dev.platform.i2c_rst_pin = i2c_rst_pin;
      p_dev = &_dev;
    }

    /** Constructor
    * @param spi object of an helper class which handles the SPI peripheral
    * @param cs_pin the chip select pin
    * @param spi_speed the SPI speed
    * @param[in] lpn_pin pin to be used as component LPn
    * @param[in] i2c_rst_pin pin to be used as component I2C_RST
    */
    VL53L8CX(SPIClass *spi, int cs_pin, int lpn_pin = -1,  uint32_t spi_speed = 2000000, int i2c_rst_pin = -1)
    {
      memset((void *)&_dev, 0x0, sizeof(VL53L8CX_Configuration));
      _dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
      _dev.platform.dev_i2c = NULL;
      _dev.platform.dev_spi = spi;
      _dev.platform.cs_pin = cs_pin;
      _dev.platform.spi_speed = spi_speed ;
      _dev.platform.lpn_pin = lpn_pin;
      _dev.platform.i2c_rst_pin = i2c_rst_pin;
      p_dev = &_dev;
    }


    /** Destructor
     */
    virtual ~VL53L8CX() {}
    /* warning: VL53LX class inherits from GenericSensor, RangeSensor and LightSensor, that haven`t a destructor.
       The warning should request to introduce a virtual destructor to make sure to delete the object */

    virtual int begin()
    {
      if (_dev.platform.lpn_pin >= 0) {
        pinMode(_dev.platform.lpn_pin, OUTPUT);
        digitalWrite(_dev.platform.lpn_pin, LOW);
      }
      if (_dev.platform.i2c_rst_pin >= 0) {
        pinMode(_dev.platform.i2c_rst_pin, OUTPUT);
        if (_dev.platform.dev_i2c) {
          digitalWrite(_dev.platform.i2c_rst_pin, LOW);
        }
        if (_dev.platform.dev_spi) {
          digitalWrite(_dev.platform.i2c_rst_pin, HIGH);
        }
      }

      if (_dev.platform.dev_spi) {
        // Configure CS pin
        pinMode(_dev.platform.cs_pin, OUTPUT);
        digitalWrite(_dev.platform.cs_pin, HIGH);
      }
      return 0;
    }

    virtual int end()
    {
      if (_dev.platform.lpn_pin >= 0) {
        pinMode(_dev.platform.lpn_pin, INPUT);
      }
      if (_dev.platform.i2c_rst_pin >= 0) {
        pinMode(_dev.platform.i2c_rst_pin, INPUT);
      }
      /* Reset CS configuration */
      if (_dev.platform.dev_spi) {
        // Configure CS pin
        pinMode(_dev.platform.cs_pin, INPUT);
      }
      return 0;
    }

    /*** Interface Methods ***/
    /*** High level API ***/
    /**
     * @brief       PowerOn the sensor
     * @return      void
     */
    virtual void vl53l8cx_on(void)
    {
      if (_dev.platform.lpn_pin >= 0) {
        digitalWrite(_dev.platform.lpn_pin, HIGH);
      }
      delay(10);
    }

    /**
     * @brief       PowerOff the sensor
     * @return      void
     */
    virtual void vl53l8cx_off(void)
    {
      if (_dev.platform.lpn_pin >= 0) {
        digitalWrite(_dev.platform.lpn_pin, LOW);
      }
      delay(10);
    }

    /**
     * @brief       Reset I2C peripheral of the sensor
     * @return      void
     */
    virtual void vl53l8cx_i2c_reset(void)
    {
      if (_dev.platform.dev_i2c && _dev.platform.i2c_rst_pin >= 0) {
        digitalWrite(_dev.platform.i2c_rst_pin, LOW);
        delay(10);
        digitalWrite(_dev.platform.i2c_rst_pin, HIGH);
        delay(10);
        digitalWrite(_dev.platform.i2c_rst_pin, LOW);
        delay(10);
      }
    }

    /**
     * @brief  Initialize the sensor
     * @param (uint8_t) addr : New I2C address.
     * @return (uint8_t) status : 0 if init_sensor is OK.
     */
    int init_sensor(uint8_t addr = VL53L8CX_DEFAULT_I2C_ADDRESS)
    {
      uint8_t status = VL53L8CX_STATUS_OK;
      uint8_t isAlive = 0;

      // Reset the sensor by toggling the LPN pin
      vl53l8cx_off();
      vl53l8cx_on();
      if (_dev.platform.dev_i2c && (addr != _dev.platform.address)) {
        status = vl53l8cx_set_i2c_address(addr);
        if (status != VL53L8CX_STATUS_OK) {
          return VL53L8CX_STATUS_ERROR;
        }
      }

      status = vl53l8cx_is_alive(&isAlive);


      if (!isAlive || status != VL53L8CX_STATUS_OK) {
        return VL53L8CX_STATUS_ERROR;
      }

      // Init VL53L8CX sensor
      status = vl53l8cx_init();
      if (status != VL53L8CX_STATUS_OK) {
        ;
        return VL53L8CX_STATUS_ERROR;
      }

      return (int)status;
    }


    /* vl53l8cx_api.h */
    /**
     * @brief This function is used to check if the sensor is alive.
     * @param (uint8_t) *p_is_alive : 1 if the sensor is alive, 0 in case of error.
     * @return (uint8_t) status : 0 if is_alive is OK.
     */
    uint8_t vl53l8cx_is_alive(
      uint8_t       *p_is_alive);

    /**
     * @brief Mandatory function used to initialize the sensor. This function must
     * be called after a power on, to load the firmware into the VL53L8CX. It takes
     * a few hundred milliseconds.
     * @return (uint8_t) status : 0 if initialization is OK.
     */

    uint8_t vl53l8cx_init();

    /**
     * @brief This function is used to change the I2C address of the sensor. If
     * multiple VL53L7 sensors are connected to the same I2C line, all other LPn
     * pins needs to be set to Low. The default sensor address is 0x52.
     * @param (uint16_t) i2c_address : New I2C address.
     * @return (uint8_t) status : 0 if new address is OK
     */

    uint8_t vl53l8cx_set_i2c_address(
      uint16_t      i2c_address);

    /**
     * @brief This function is used to get the current sensor power mode.
     * @param (uint8_t) *p_power_mode : Current power mode. The value of this
     * pointer is equal to 0 if the sensor is in low power,
     * (VL53L8CX_POWER_MODE_SLEEP), or 1 if sensor is in standard mode
     * (VL53L8CX_POWER_MODE_WAKEUP).
     * @return (uint8_t) status : 0 if power mode is OK
     */

    uint8_t vl53l8cx_get_power_mode(
      uint8_t       *p_power_mode);

    /**
     * @brief This function is used to set the sensor in Low Power mode, for
     * example if the sensor is not used during a long time. The macro
     * VL53L8CX_POWER_MODE_SLEEP can be used to enable the low power mode. When user
     * want to restart the sensor, he can use macro VL53L8CX_POWER_MODE_WAKEUP.
     * Please ensure that the device is not streaming before calling the function.
     * @param (uint8_t) power_mode : Selected power mode (VL53L8CX_POWER_MODE_SLEEP
     * or VL53L8CX_POWER_MODE_WAKEUP)
     * @return (uint8_t) status : 0 if power mode is OK, or 127 if power mode
     * requested by user is not valid.
     */

    uint8_t vl53l8cx_set_power_mode(
      uint8_t       power_mode);

    /**
     * @brief This function starts a ranging session. When the sensor streams, host
     * cannot change settings 'on-the-fly'.
     * @return (uint8_t) status : 0 if start is OK.
     */

    uint8_t vl53l8cx_start_ranging();

    /**
     * @brief This function stops the ranging session. It must be used when the
     * sensor streams, after calling vl53l8cx_start_ranging().
     * @return (uint8_t) status : 0 if stop is OK
     */

    uint8_t vl53l8cx_stop_ranging();

    /**
     * @brief This function checks if a new data is ready by polling I2C. If a new
     * data is ready, a flag will be raised.
     * @param (uint8_t) *p_isReady : Value of this pointer be updated to 0 if data
     * is not ready, or 1 if a new data is ready.
     * @return (uint8_t) status : 0 if I2C reading is OK
     */

    uint8_t vl53l8cx_check_data_ready(
      uint8_t       *p_isReady);

    /**
     * @brief This function gets the ranging data, using the selected output and the
     * resolution.
     * @param (VL53L8CX_ResultsData) *p_results : VL53L7 results structure.
     * @return (uint8_t) status : 0 data are successfully get.
     */

    uint8_t vl53l8cx_get_ranging_data(
      VL53L8CX_ResultsData    *p_results);

    /**
     * @brief This function gets the current resolution (4x4 or 8x8).
     * @param (uint8_t) *p_resolution : Value of this pointer will be equal to 16
     * for 4x4 mode, and 64 for 8x8 mode.
     * @return (uint8_t) status : 0 if resolution is OK.
     */

    uint8_t vl53l8cx_get_resolution(
      uint8_t       *p_resolution);

    /**
     * @brief This function sets a new resolution (4x4 or 8x8).
     * @param (uint8_t) resolution : Use macro VL53L8CX_RESOLUTION_4X4 or
     * VL53L8CX_RESOLUTION_8X8 to set the resolution.
     * @return (uint8_t) status : 0 if set resolution is OK.
     */

    uint8_t vl53l8cx_set_resolution(
      uint8_t                         resolution);

    /**
     * @brief This function gets the current ranging frequency in Hz. Ranging
     * frequency corresponds to the time between each measurement.
     * @param (uint8_t) *p_frequency_hz: Contains the ranging frequency in Hz.
     * @return (uint8_t) status : 0 if ranging frequency is OK.
     */

    uint8_t vl53l8cx_get_ranging_frequency_hz(
      uint8_t       *p_frequency_hz);

    /**
     * @brief This function sets a new ranging frequency in Hz. Ranging frequency
     * corresponds to the measurements frequency. This setting depends of
     * the resolution, so please select your resolution before using this function.
     * @param (uint8_t) frequency_hz : Contains the ranging frequency in Hz.
     * - For 4x4, min and max allowed values are : [1;60]
     * - For 8x8, min and max allowed values are : [1;15]
     * @return (uint8_t) status : 0 if ranging frequency is OK, or 127 if the value
     * is not correct.
     */

    uint8_t vl53l8cx_set_ranging_frequency_hz(
      uint8_t       frequency_hz);

    /**
     * @brief This function gets the current integration time in ms.
     * @param (uint32_t) *p_time_ms: Contains integration time in ms.
     * @return (uint8_t) status : 0 if integration time is OK.
     */

    uint8_t vl53l8cx_get_integration_time_ms(
      uint32_t      *p_time_ms);

    /**
     * @brief This function sets a new integration time in ms. Integration time must
     * be computed to be lower than the ranging period, for a selected resolution.
     * Please note that this function has no impact on ranging mode continuous.
     * @param (uint32_t) time_ms : Contains the integration time in ms. For all
     * resolutions and frequency, the minimum value is 2ms, and the maximum is
     * 1000ms.
     * @return (uint8_t) status : 0 if set integration time is OK.
     */

    uint8_t vl53l8cx_set_integration_time_ms(
      uint32_t      integration_time_ms);

    /**
     * @brief This function gets the current sharpener in percent. Sharpener can be
     * changed to blur more or less zones depending of the application.
     * @param (uint32_t) *p_sharpener_percent: Contains the sharpener in percent.
     * @return (uint8_t) status : 0 if get sharpener is OK.
     */

    uint8_t vl53l8cx_get_sharpener_percent(
      uint8_t       *p_sharpener_percent);

    /**
     * @brief This function sets a new sharpener value in percent. Sharpener can be
     * changed to blur more or less zones depending of the application. Min value is
     * 0 (disabled), and max is 99.
     * @param (uint32_t) sharpener_percent : Value between 0 (disabled) and 99%.
     * @return (uint8_t) status : 0 if set sharpener is OK.
     */

    uint8_t vl53l8cx_set_sharpener_percent(
      uint8_t       sharpener_percent);

    /**
     * @brief This function gets the current target order (closest or strongest).
     * @param (uint8_t) *p_target_order: Contains the target order.
     * @return (uint8_t) status : 0 if get target order is OK.
     */

    uint8_t vl53l8cx_get_target_order(
      uint8_t       *p_target_order);

    /**
     * @brief This function sets a new target order. Please use macros
     * VL53L8CX_TARGET_ORDER_STRONGEST and VL53L8CX_TARGET_ORDER_CLOSEST to define
     * the new output order. By default, the sensor is configured with the strongest
     * output.
     * @param (uint8_t) target_order : Required target order.
     * @return (uint8_t) status : 0 if set target order is OK, or 127 if target
     * order is unknown.
     */

    uint8_t vl53l8cx_set_target_order(
      uint8_t       target_order);

    /**
     * @brief This function is used to get the ranging mode. Two modes are
     * available using ULD : Continuous and autonomous. The default
     * mode is Autonomous.
     * @param (uint8_t) *p_ranging_mode : current ranging mode
     * @return (uint8_t) status : 0 if get ranging mode is OK.
     */

    uint8_t vl53l8cx_get_ranging_mode(
      uint8_t       *p_ranging_mode);

    /**
     * @brief This function is used to set the ranging mode. Two modes are
     * available using ULD : Continuous and autonomous. The default
     * mode is Autonomous.
     * @param (uint8_t) ranging_mode : Use macros VL53L8CX_RANGING_MODE_CONTINUOUS,
     * VL53L8CX_RANGING_MODE_CONTINUOUS.
     * @return (uint8_t) status : 0 if set ranging mode is OK.
     */

    uint8_t vl53l8cx_set_ranging_mode(
      uint8_t       ranging_mode);


    /**
     * @brief This function is used to enable or disable the synchronization pin. When
     * it is enabled, the sensor waits an interrupt on B1 pin to start the next
     * measurement. It is useful for multi-devices synchronization. By default the sync
     * pin is disabled.
     * @param (uint8_t) enable_sync_pin : Ste the value to 1 to enable the sync
     * pin, or 0 to disable it.
     * @return (uint8_t) status : 0 if set sync pin OK.
     */
    uint8_t vl53l8cx_set_external_sync_pin_enable(
      uint8_t       enable_sync_pin);

    /**
     * @brief This function is used to check if the synchronization pin is enabled
     * or disabled. When it is enabled, the sensor waits an interrupt on B1 pin
     * to start the next measurement. It is useful for multi-devices
     * synchronization. By default the sync pin is disabled.
     * @param (uint8_t*) p_is_sync_pin_enabled : Pointer of sync pin status. Value
     * overridden to 0 if the pin is disabled, or 1 if the pin is enabled.
     * @return (uint8_t) status : 0 if get sync pin OK.
     */
    uint8_t vl53l8cx_get_external_sync_pin_enable(
      uint8_t       *p_is_sync_pin_enabled);

    /**
     * @brief This function can be used to read 'extra data' from DCI. Using a known
     * index, the function fills the casted structure passed in argument.
     * @param (uint8_t) *data : This field can be a casted structure, or a simple
     * array. Please note that the FW only accept data of 32 bits. So field data can
     * only have a size of 32, 64, 96, 128, bits ....
     * @param (uint32_t) index : Index of required value.
     * @param (uint16_t)*data_size : This field must be the structure or array size
     * (using sizeof() function).
     * @return (uint8_t) status : 0 if OK
     */

    uint8_t vl53l8cx_dci_read_data(
      uint8_t       *data,
      uint32_t      index,
      uint16_t      data_size);

    /**
     * @brief This function can be used to write 'extra data' to DCI. The data can
     * be simple data, or casted structure.
     * @param (uint8_t) *data : This field can be a casted structure, or a simple
     * array. Please note that the FW only accept data of 32 bits. So field data can
     * only have a size of 32, 64, 96, 128, bits ..
     * @param (uint32_t) index : Index of required value.
     * @param (uint16_t)*data_size : This field must be the structure or array size
     * (using sizeof() function).
     * @return (uint8_t) status : 0 if OK
     */

    uint8_t vl53l8cx_dci_write_data(
      uint8_t       *data,
      uint32_t      index,
      uint16_t      data_size);

    /**
     * @brief This function can be used to replace 'extra data' in DCI. The data can
     * be simple data, or casted structure.
     * @param (uint8_t) *data : This field can be a casted structure, or a simple
     * array. Please note that the FW only accept data of 32 bits. So field data can
     * only have a size of 32, 64, 96, 128, bits ..
     * @param (uint32_t) index : Index of required value.
     * @param (uint16_t)*data_size : This field must be the structure or array size
     * (using sizeof() function).
     * @param (uint8_t) *new_data : Contains the new fields.
     * @param (uint16_t) new_data_size : New data size.
     * @param (uint16_t) new_data_pos : New data position into the buffer.
     * @return (uint8_t) status : 0 if OK
     */

    uint8_t vl53l8cx_dci_replace_data(
      uint8_t       *data,
      uint32_t      index,
      uint16_t      data_size,
      uint8_t       *new_data,
      uint16_t      new_data_size,
      uint16_t      new_data_pos);


    /* Thresholds Detection APIs */

    /**
     * @brief This function allows indicating if the detection thresholds are
     * enabled.
     * @param (uint8_t) *p_enabled : Set to 1 if enabled, or 0 if disable.
     * @return (uint8_t) status : 0 if OK
     */

    uint8_t vl53l8cx_get_detection_thresholds_enable(
      uint8_t       *p_enabled);

    /**
     * @brief This function allows enable the detection thresholds.
     * @param (uint8_t) enabled : Set to 1 to enable, or 0 to disable thresholds.
     * @return (uint8_t) status : 0 if programming is OK
     */

    uint8_t vl53l8cx_set_detection_thresholds_enable(
      uint8_t       enabled);

    /**
     * @brief This function allows getting the detection thresholds.
     * @param (VL53L8CX_DetectionThresholds) *p_thresholds : Array of 64 thresholds.
     * @return (uint8_t) status : 0 if programming is OK
     */

    uint8_t vl53l8cx_get_detection_thresholds(
      VL53L8CX_DetectionThresholds  *p_thresholds);

    /**
     * @brief This function allows programming the detection thresholds.
     * @param (VL53L8CX_DetectionThresholds) *p_thresholds :  Array of 64 thresholds.
     * @return (uint8_t) status : 0 if programming is OK
     */

    uint8_t vl53l8cx_set_detection_thresholds(
      VL53L8CX_DetectionThresholds  *p_thresholds);

    uint8_t vl53l8cx_get_detection_thresholds_auto_stop(
      uint8_t       *p_auto_stop);

    uint8_t vl53l8cx_set_detection_thresholds_auto_stop(
      uint8_t       p_auto_stop);

    /* Motion Indicator APIs */

    /**
     * @brief This function is used to initialized the motion indicator. By default,
     * indicator is programmed to monitor movements between 400mm and 1500mm.
     * @param (VL53L8CX_Motion_Configuration) *p_motion_config : Structure
     * containing the initialized motion configuration.
     * @param (uint8_t) resolution : Wanted resolution, defined by macros
     * VL53L8CX_RESOLUTION_4X4 or VL53L8CX_RESOLUTION_8X8.
     * @return (uint8_t) status : 0 if OK, or 127 is the resolution is unknown.
     */

    uint8_t vl53l8cx_motion_indicator_init(
      VL53L8CX_Motion_Configuration *p_motion_config,
      uint8_t       resolution);

    /**
     * @brief This function can be used to change the working distance of motion
     * indicator. By default, indicator is programmed to monitor movements between
     * 400mm and 1500mm.
     * @param (VL53L8CX_Motion_Configuration) *p_motion_config : Structure
     * containing the initialized motion configuration.
     * @param (uint16_t) distance_min_mm : Minimum distance for indicator (min value
     * 400mm, max 4000mm).
     * @param (uint16_t) distance_max_mm : Maximum distance for indicator (min value
     * 400mm, max 4000mm).
     * VL53L8CX_RESOLUTION_4X4 or VL53L8CX_RESOLUTION_8X8.
     * @return (uint8_t) status : 0 if OK, or 127 if an argument is invalid.
     */

    uint8_t vl53l8cx_motion_indicator_set_distance_motion(
      VL53L8CX_Motion_Configuration *p_motion_config,
      uint16_t      distance_min_mm,
      uint16_t      distance_max_mm);

    /**
     * @brief This function is used to update the internal motion indicator map.
     * @param (VL53L8CX_Motion_Configuration) *p_motion_config : Structure
     * containing the initialized motion configuration.
     * @param (uint8_t) resolution : Wanted SCI resolution, defined by macros
     * VL53L8CX_RESOLUTION_4X4 or VL53L8CX_RESOLUTION_8X8.
     * @return (uint8_t) status : 0 if OK, or 127 is the resolution is unknown.
     */

    uint8_t vl53l8cx_motion_indicator_set_resolution(
      VL53L8CX_Motion_Configuration *p_motion_config,
      uint8_t       resolution);

    /* XTalk APIs */

    /**
     * @brief This function starts the VL53L8CX sensor in order to calibrate Xtalk.
     * This calibration is recommended is user wants to use a coverglass.
     * @param (uint16_t) reflectance_percent : Target reflectance in percent. This
     * value is include between 1 and 99%. For a better efficiency, ST recommends a
     * 3% target reflectance.
     * @param (uint8_t) nb_samples : Nb of samples used for calibration. A higher
     * number of samples means a higher accuracy, but it increases the calibration
     * time. Minimum is 1 and maximum is 16.
     * @param (uint16_t) distance_mm : Target distance in mm. The minimum allowed
     * distance is 600mm, and maximum is 3000mm. The target must stay in Full FOV,
     * so short distance are easier for calibration.
     * @return (uint8_t) status : 0 if calibration OK, 127 if an argument has an
     * incorrect value, or 255 is something failed.
     */

    uint8_t vl53l8cx_calibrate_xtalk(
      uint16_t      reflectance_percent,
      uint8_t       nb_samples,
      uint16_t      distance_mm);

    /**
     * @brief This function gets the Xtalk buffer. The buffer is available after
     * using the function vl53l8cx_calibrate_xtalk().
     * @param (uint8_t) *p_xtalk_data : Buffer with a size defined by
     * macro VL53L8CX_XTALK_SIZE.
     * @return (uint8_t) status : 0 if buffer reading OK
     */

    uint8_t vl53l8cx_get_caldata_xtalk(
      uint8_t       *p_xtalk_data);

    /**
     * @brief This function sets the Xtalk buffer. This function can be used to
     * override default Xtalk buffer.
     * @param (uint8_t) *p_xtalk_data : Buffer with a size defined by
     * macro VL53L8CX_XTALK_SIZE.
     * @return (uint8_t) status : 0 if buffer OK
     */

    uint8_t vl53l8cx_set_caldata_xtalk(
      uint8_t       *p_xtalk_data);

    /**
     * @brief This function gets the Xtalk margin. This margin is used to increase
     * the Xtalk threshold. It can also be used to avoid false positives after the
     * Xtalk calibration. The default value is 50 kcps/spads.
     * @param (uint32_t) *p_xtalk_margin : Xtalk margin in kcps/spads.
     * @return (uint8_t) status : 0 if reading OK
     */

    uint8_t vl53l8cx_get_xtalk_margin(
      uint32_t      *p_xtalk_margin);

    /**
     * @brief This function sets the Xtalk margin. This margin is used to increase
     * the Xtalk threshold. It can also be used to avoid false positives after the
     * Xtalk calibration. The default value is 50 kcps/spads.
     * @param (uint32_t) xtalk_margin : New Xtalk margin in kcps/spads. Min value is
     * 0 kcps/spads, and max is 10.000 kcps/spads
     * @return (uint8_t) status : 0 if set margin is OK, or 127 is the margin is
     * invalid.
     */

    uint8_t vl53l8cx_set_xtalk_margin(
      uint32_t      xtalk_margin);

    /**
     * @brief Mandatory function, used to swap a buffer. The buffer size is always a
     * multiple of 4 (4, 8, 12, 16, ...).
     * @param (uint8_t*) buffer : Buffer to swap, generally uint32_t
     * @param (uint16_t) size : Buffer size to swap
     */

    void SwapBuffer(
      uint8_t     *buffer,
      uint16_t     size);

    /* Helpful APIs */
    uint8_t get_stream_count(void)
    {
      return _dev.streamcount;
    };

  protected:

    /**
     * @brief Inner function, not available outside this file. This function is used
     * to wait for an answer from VL53L8CX sensor.
     */
    uint8_t _vl53l8cx_poll_for_answer(
      uint8_t       size,
      uint8_t       pos,
      uint16_t      address,
      uint8_t       mask,
      uint8_t       expected_value);
    /**
    *  @brief Inner function, not available outside this file. This function is used to
    * wait for the MCU to boot.
    */
    uint8_t _vl53l8cx_poll_for_mcu_boot();
    /**
     * @brief Inner function, not available outside this file. This function is used
     * to set the offset data gathered from NVM.
     */
    uint8_t _vl53l8cx_send_offset_data(
      uint8_t       resolution);

    /**
     * @brief Inner function, not available outside this file. This function is used
     * to set the Xtalk data from generic configuration, or user's calibration.
     */
    uint8_t _vl53l8cx_send_xtalk_data(
      uint8_t       resolution);

    /**
     * @brief Inner function, not available outside this file. This function is used to
     * wait for an answer from VL53L7 sensor.
     */
    uint8_t _vl53l8cx_poll_for_answer_xtalk(
      uint16_t      address,
      uint8_t       expected_value);

    /**
     * @brief Inner function, not available outside this file. This function is used to
     * program the output using the macro defined into the 'platform.h' file.
     */
    uint8_t _vl53l8cx_program_output_config();

    /* Platform APIs */

    /**
     * @param (VL53L8CX_Platform*) p_platform : Pointer of VL53L8CX platform
     * structure.
     * @param (uint16_t) Address : I2C location of value to read.
     * @param (uint8_t) *p_values : Pointer of value to read.
     * @return (uint8_t) status : 0 if OK
     */

    uint8_t RdByte(
      VL53L8CX_Platform *p_platform,
      uint16_t RegisterAddress,
      uint8_t *p_value);

    /**
     * @brief Mandatory function used to write one single byte.
     * @param (VL53L8CX_Platform*) p_platform : Pointer of VL53L8CX platform
     * structure.
     * @param (uint16_t) Address : I2C location of value to read.
     * @param (uint8_t) value : Pointer of value to write.
     * @return (uint8_t) status : 0 if OK
     */

    uint8_t WrByte(
      VL53L8CX_Platform *p_platform,
      uint16_t RegisterAddress,
      uint8_t value);

    /**
     * @brief Mandatory function used to read multiples bytes.
     * @param (VL53L8CX_Platform*) p_platform : Pointer of VL53L8CX platform
     * structure.
     * @param (uint16_t) Address : I2C location of values to read.
     * @param (uint8_t) *p_values : Buffer of bytes to read.
     * @param (uint32_t) size : Size of *p_values buffer.
     * @return (uint8_t) status : 0 if OK
     */

    uint8_t RdMulti(
      VL53L8CX_Platform *p_platform,
      uint16_t RegisterAddress,
      uint8_t *p_values,
      uint32_t size);

    /**
     * @brief Mandatory function used to write multiples bytes.
     * @param (VL53L8CX_Platform*) p_platform : Pointer of VL53L8CX platform
     * structure.
     * @param (uint16_t) Address : I2C location of values to write.
     * @param (uint8_t) *p_values : Buffer of bytes to write.
     * @param (uint32_t) size : Size of *p_values buffer.
     * @return (uint8_t) status : 0 if OK
     */

    uint8_t WrMulti(
      VL53L8CX_Platform *p_platform,
      uint16_t RegisterAddress,
      uint8_t *p_values,
      uint32_t size);

    /**
     * @brief Mandatory function, used to wait during an amount of time. It must be
     * filled as it's used into the API.
     * @param (VL53L8CX_Platform*) p_platform : Pointer of VL53L8CX platform
     * structure.
     * @param (uint32_t) TimeMs : Time to wait in ms.
     * @return (uint8_t) status : 0 if wait is finished.
     */

    uint8_t WaitMs(
      VL53L8CX_Platform *p_platform,
      uint32_t TimeMs);

  protected:

    /* VL53L8CX Device */
    VL53L8CX_Configuration _dev;
    VL53L8CX_Configuration *p_dev;
};

#endif /* __VL53L8CX_CLASS_H */
