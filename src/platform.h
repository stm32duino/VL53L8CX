/**
 ******************************************************************************
 * @file    platform.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    11 November 2021
 * @brief   Header file of the platform dependent structures.
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

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>
#include <Arduino.h>
#include <Wire.h>
#include "SPI.h"
#include "platform_config.h"


#ifndef DEFAULT_I2C_BUFFER_LEN
  #ifdef ARDUINO_SAM_DUE
    /* FIXME: It seems that an I2C buffer of BUFFER_LENGTH does not work on Arduino DUE. So, we need to decrease the size */
    #define DEFAULT_I2C_BUFFER_LEN (BUFFER_LENGTH - 2)
  #else
    #ifdef BUFFER_LENGTH
      #define DEFAULT_I2C_BUFFER_LEN BUFFER_LENGTH
    #else
      #define DEFAULT_I2C_BUFFER_LEN 32
    #endif
  #endif
#endif

/**
 * @brief Structure VL53L8CX_Platform needs to be filled by the customer,
 * depending on his platform. At least, it contains the VL53L8CX I2C address.
 * Some additional fields can be added, as descriptors, or platform
 * dependencies. Anything added into this structure is visible into the platform
 * layer.
 */

typedef struct {
  uint16_t  address;

  TwoWire *dev_i2c;
  SPIClass *dev_spi;

  int cs_pin;
  uint32_t spi_speed;

  int lpn_pin;

  int i2c_rst_pin;

} VL53L8CX_Platform;

#endif  // _PLATFORM_H_
