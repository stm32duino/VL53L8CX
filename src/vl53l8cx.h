/**
 ******************************************************************************
 * @file    vl53l8cx.h
 * @author  STMicroelectronics
 * @version V2.0.0
 * @date    27 June 2024
 * @brief   Abstract class of a VL53L8CX Time of Flight(TOF) sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2024 STMicroelectronics</center></h2>
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
 * DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Prevent recursive inclusion -----------------------------------------------*/
#ifndef __VL53L8CX_H
#define __VL53L8CX_H

/* Includes ------------------------------------------------------------------*/
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "platform.h"
#include "vl53l8cx_api.h"
#include "vl53l8cx_plugin_detection_thresholds.h"
#include "vl53l8cx_plugin_motion_indicator.h"
#include "vl53l8cx_plugin_xtalk.h"

/* Class Declaration ---------------------------------------------------------*/
class VL53L8CX {
  public:
    VL53L8CX(TwoWire *i2c, int lpn_pin, int i2c_rst_pin = -1);
    VL53L8CX(SPIClass *spi, int cs_pin, int lpn_pin = -1, int i2c_rst_pin = -1, uint32_t spi_speed = 5000000);
    virtual ~VL53L8CX(void);
    virtual int begin(void);
    virtual int end(void);
    virtual void on(void);
    virtual void off(void);
    virtual void i2c_reset(void);
    uint8_t is_alive(uint8_t *p_is_alive);
    uint8_t init(void);
    uint8_t set_i2c_address(uint16_t i2c_address);
    uint8_t get_power_mode(uint8_t *p_power_mode);
    uint8_t set_power_mode(uint8_t power_mode);
    uint8_t start_ranging(void);
    uint8_t stop_ranging(void);
    uint8_t check_data_ready(uint8_t *p_isReady);
    uint8_t get_ranging_data(VL53L8CX_ResultsData *p_results);
    uint8_t get_resolution(uint8_t *p_resolution);
    uint8_t set_resolution(uint8_t resolution);
    uint8_t get_ranging_frequency_hz(uint8_t *p_frequency_hz);
    uint8_t set_ranging_frequency_hz(uint8_t frequency_hz);
    uint8_t get_integration_time_ms(uint32_t *p_time_ms);
    uint8_t set_integration_time_ms(uint32_t integration_time_ms);
    uint8_t get_sharpener_percent(uint8_t *p_sharpener_percent);
    uint8_t set_sharpener_percent(uint8_t sharpener_percent);
    uint8_t get_target_order(uint8_t *p_target_order);
    uint8_t set_target_order(uint8_t target_order);
    uint8_t get_ranging_mode(uint8_t *p_ranging_mode);
    uint8_t set_ranging_mode(uint8_t ranging_mode);
    uint8_t get_external_sync_pin_enable(uint8_t *p_is_sync_pin_enabled);
    uint8_t set_external_sync_pin_enable(uint8_t enable_sync_pin);
    uint8_t get_VHV_repeat_count(uint32_t *p_repeat_count);
    uint8_t set_VHV_repeat_count(uint32_t repeat_count);
    uint8_t dci_read_data(uint8_t *data, uint32_t index, uint16_t data_size);
    uint8_t dci_write_data(uint8_t *data, uint32_t index, uint16_t data_size);
    uint8_t dci_replace_data(uint8_t *data, uint32_t index, uint16_t data_size, uint8_t *new_data, uint16_t new_data_size, uint16_t new_data_pos);
    uint8_t get_detection_thresholds_enable(uint8_t *p_enabled);
    uint8_t set_detection_thresholds_enable(uint8_t enabled);
    uint8_t get_detection_thresholds(VL53L8CX_DetectionThresholds *p_thresholds);
    uint8_t set_detection_thresholds(VL53L8CX_DetectionThresholds *p_thresholds);
    uint8_t get_detection_thresholds_auto_stop(uint8_t *p_auto_stop);
    uint8_t set_detection_thresholds_auto_stop(uint8_t auto_stop);
    uint8_t motion_indicator_init(VL53L8CX_Motion_Configuration *p_motion_config, uint8_t resolution);
    uint8_t motion_indicator_set_distance_motion(VL53L8CX_Motion_Configuration  *p_motion_config, uint16_t distance_min_mm, uint16_t distance_max_mm);
    uint8_t motion_indicator_set_resolution(VL53L8CX_Motion_Configuration *p_motion_config, uint8_t resolution);
    uint8_t calibrate_xtalk(uint16_t reflectance_percent, uint8_t nb_samples, uint16_t distance_mm);
    uint8_t get_caldata_xtalk(uint8_t *p_xtalk_data);
    uint8_t set_caldata_xtalk(uint8_t *p_xtalk_data);
    uint8_t get_xtalk_margin(uint32_t *p_xtalk_margin);
    uint8_t set_xtalk_margin(uint32_t xtalk_margin);

    /**
     * @brief Utility function to read data.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  p_values: pointer to data to be read.
     * @param  size: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint16_t RegisterAddress, uint8_t *p_values, uint32_t size)
    {
      if (dev_i2c) {
        int status = 0;
        uint8_t buffer[2];
        // Loop until the port is transmitted correctly
        do {
          dev_i2c->beginTransmission((uint8_t)((_dev.platform.address >> 1) & 0x7F));
          // Target register address for transfer
          buffer[0] = (uint8_t)(RegisterAddress >> 8);
          buffer[1] = (uint8_t)(RegisterAddress & 0xFF);
          dev_i2c->write(buffer, 2);
          status = dev_i2c->endTransmission(false);
          // Fix for some STM32 boards
          // Reinitialize the i2c bus with the default parameters
#ifdef ARDUINO_ARCH_STM32
          if (status) {
            dev_i2c->end();
            dev_i2c->begin();
          }
#endif
          // End of fix
        } while (status != 0);
        uint32_t i = 0;
        if (size > DEFAULT_I2C_BUFFER_LEN) {
          while (i < size) {
            // If still more than DEFAULT_I2C_BUFFER_LEN bytes to go, DEFAULT_I2C_BUFFER_LEN,
            // else the remaining number of bytes
            uint8_t current_read_size = (size - i > DEFAULT_I2C_BUFFER_LEN ? DEFAULT_I2C_BUFFER_LEN : size - i);
            dev_i2c->requestFrom(((uint8_t)((_dev.platform.address >> 1) & 0x7F)), current_read_size);
            while (dev_i2c->available()) {
              p_values[i] = dev_i2c->read();
              i++;
            }
          }
        } else {
          dev_i2c->requestFrom(((uint8_t)((_dev.platform.address >> 1) & 0x7F)), size);
          while (dev_i2c->available()) {
            p_values[i] = dev_i2c->read();
            i++;
          }
        }
        return i != size;
      }
      if (dev_spi) {
        uint8_t status = 0;
        uint32_t position = 0;
        uint32_t data_size = 0;
        uint16_t    temp;
        uint8_t data_write[VL53L8CX_COMMS_CHUNK_SIZE + 2];
        for (position = 0; position < size; position += VL53L8CX_COMMS_CHUNK_SIZE) {
          if (size > VL53L8CX_COMMS_CHUNK_SIZE) {
            if ((position + VL53L8CX_COMMS_CHUNK_SIZE) > size) {
              data_size = size - position;
            } else {
              data_size = VL53L8CX_COMMS_CHUNK_SIZE;
            }
          } else {
            data_size = size;
          }
          temp = RegisterAddress + position;
          data_write[0] = SPI_READ_MASK(temp) >> 8;
          data_write[1] = SPI_READ_MASK(temp) & 0xFF;
          dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
          digitalWrite(cs_pin, LOW);
          dev_spi->transfer(&data_write, 2);
          for (uint16_t i = 0; i < data_size; i++) {
            *(p_values + i + position) = dev_spi->transfer(0x00);
          }
          digitalWrite(cs_pin, HIGH);
        }
        return 0;
      }
      return 1;
    }

    /**
     * @brief Utility function to write data.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  p_values: pointer to data to be written.
     * @param  size: number of bytes to be written.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint16_t RegisterAddress, uint8_t *p_values, uint32_t size)
    {
      if (dev_i2c) {
        uint32_t i = 0;
        uint8_t buffer[2];
        while (i < size) {
          // If still more than DEFAULT_I2C_BUFFER_LEN bytes to go, DEFAULT_I2C_BUFFER_LEN,
          // else the remaining number of bytes
          size_t current_write_size = (size - i > DEFAULT_I2C_BUFFER_LEN ? DEFAULT_I2C_BUFFER_LEN : size - i);
          dev_i2c->beginTransmission((uint8_t)((_dev.platform.address >> 1) & 0x7F));
          // Target register address for transfer
          buffer[0] = (uint8_t)((RegisterAddress + i) >> 8);
          buffer[1] = (uint8_t)((RegisterAddress + i) & 0xFF);
          dev_i2c->write(buffer, 2);
          if (dev_i2c->write(p_values + i, current_write_size) == 0) {
            return 1;
          } else {
            i += current_write_size;
            if (size - i) {
              // Flush buffer and send stop bit so we have compatibility also with ESP32 platforms
              dev_i2c->endTransmission(true);
            }
          }
        }
        return dev_i2c->endTransmission(true);
      }

      if (dev_spi) {

        uint8_t status = 0;
        int32_t i = 0;
        uint32_t position = 0;
        uint32_t data_size = 0;
        uint16_t    temp;
        uint8_t data_write[VL53L8CX_COMMS_CHUNK_SIZE + 2];
        for (position = 0; position < size; position += VL53L8CX_COMMS_CHUNK_SIZE) {
          if (size > VL53L8CX_COMMS_CHUNK_SIZE) {
            if ((position + VL53L8CX_COMMS_CHUNK_SIZE) > size) {
              data_size = size - position;
            } else {
              data_size = VL53L8CX_COMMS_CHUNK_SIZE;
            }
          } else {
            data_size = size;
          }
          temp = RegisterAddress + position;
          data_write[0] = SPI_WRITE_MASK(temp) >> 8;
          data_write[1] = SPI_WRITE_MASK(temp) & 0xFF;
          for (i = 0; i < data_size; i++) {
            data_write[i + 2] = p_values[position + i];
          }
          data_size += 2;
          dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
          digitalWrite(cs_pin, LOW);
          dev_spi->transfer(&data_write, data_size);
          digitalWrite(cs_pin, HIGH);
        }
        return status;
      }
      return 1;
    }

    /**
     * @brief Utility function to wait.
     * @param  ms: milliseconds to wait.
     * @retval 0
     */
    uint8_t IO_Wait(uint32_t ms)
    {
      delay(ms);
      return 0;
    }

  private:

    /* Helper classes. */
    TwoWire *dev_i2c;
    SPIClass *dev_spi;

    /* Configuration */
    int  lpn_pin;
    int i2c_rst_pin;
    int cs_pin;
    uint32_t spi_speed;

    VL53L8CX_Configuration _dev;
    VL53L8CX_Configuration *p_dev;
};


#ifdef __cplusplus
extern "C" {
#endif

uint8_t VL53L8CX_io_write(void *handle, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);
uint8_t VL53L8CX_io_read(void *handle, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);
uint8_t VL53L8CX_io_wait(void *handle, uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __VL53L8CX_H */
