/**
 ******************************************************************************
 * @file    platform.cpp
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    11 November 2021
 * @brief   Implementation of the platform dependent APIs.
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


#include "vl53l8cx_class.h"

#define VL53L8CX_COMMS_CHUNK_SIZE 4096
#define SPI_WRITE_MASK(x) (uint16_t)(x | 0x8000)
#define SPI_READ_MASK(x)  (uint16_t)(x & ~0x8000)


uint8_t VL53L8CX::RdByte(
  VL53L8CX_Platform *p_platform,
  uint16_t RegisterAddress,
  uint8_t *p_value)
{
  uint8_t status = RdMulti(p_platform, RegisterAddress, p_value, 1);
  return status;
}

uint8_t VL53L8CX::WrByte(
  VL53L8CX_Platform *p_platform,
  uint16_t RegisterAddress,
  uint8_t value)
{
  // Just use WrMulti but 1 byte
  uint8_t status = WrMulti(p_platform, RegisterAddress, &value, 1);
  return status;
}

uint8_t VL53L8CX::WrMulti(
  VL53L8CX_Platform *p_platform,
  uint16_t RegisterAddress,
  uint8_t *p_values,
  uint32_t size)
{
  if (p_platform->dev_i2c) {
    uint32_t i = 0;
    uint8_t buffer[2];
    while (i < size) {
      // If still more than DEFAULT_I2C_BUFFER_LEN bytes to go, DEFAULT_I2C_BUFFER_LEN,
      // else the remaining number of bytes
      size_t current_write_size = (size - i > DEFAULT_I2C_BUFFER_LEN ? DEFAULT_I2C_BUFFER_LEN : size - i);

      p_platform->dev_i2c->beginTransmission((uint8_t)((p_platform->address >> 1) & 0x7F));

      // Target register address for transfer
      buffer[0] = (uint8_t)((RegisterAddress + i) >> 8);
      buffer[1] = (uint8_t)((RegisterAddress + i) & 0xFF);

      p_platform->dev_i2c->write(buffer, 2);
      if (p_platform->dev_i2c->write(p_values + i, current_write_size) == 0) {
        return 1;
      } else {
        i += current_write_size;
        if (size - i) {

          // Flush buffer but do not send stop bit so we can keep going
          p_platform->dev_i2c->endTransmission(false);

        }
      }
    }
    return p_platform->dev_i2c->endTransmission(true);
  }

  if (p_platform->dev_spi) {

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
      p_platform->dev_spi->beginTransaction(SPISettings(p_platform->spi_speed, MSBFIRST, SPI_MODE3));
      digitalWrite(p_platform->cs_pin, LOW);
      p_platform->dev_spi->transfer(&data_write, data_size);
      digitalWrite(p_platform->cs_pin, HIGH);
    }
    return status;
  }
  return 1;
}

uint8_t VL53L8CX::RdMulti(
  VL53L8CX_Platform *p_platform,
  uint16_t RegisterAddress,
  uint8_t *p_values,
  uint32_t size)
{
  if (p_platform->dev_i2c) {
    int status = 0;
    uint8_t buffer[2];
    // Loop until the port is transmitted correctly
    do {
      p_platform->dev_i2c->beginTransmission((uint8_t)((p_platform->address >> 1) & 0x7F));

      // Target register address for transfer
      buffer[0] = (uint8_t)(RegisterAddress >> 8);
      buffer[1] = (uint8_t)(RegisterAddress & 0xFF);
      p_platform->dev_i2c->write(buffer, 2);

      status = p_platform->dev_i2c->endTransmission(false);

      // Fix for some STM32 boards
      // Reinitialize the i2c bus with the default parameters
#ifdef ARDUINO_ARCH_STM32
      if (status) {
        p_platform->dev_i2c->end();
        p_platform->dev_i2c->begin();
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
        p_platform->dev_i2c->requestFrom(((uint8_t)((p_platform->address >> 1) & 0x7F)),
                                         current_read_size);
        while (p_platform->dev_i2c->available()) {
          p_values[i] = p_platform->dev_i2c->read();
          i++;
        }
      }
    } else {
      p_platform->dev_i2c->requestFrom(((uint8_t)((p_platform->address >> 1) & 0x7F)), size);
      while (p_platform->dev_i2c->available()) {
        p_values[i] = p_platform->dev_i2c->read();
        i++;
      }
    }
    return i != size;
  }
  if (p_platform->dev_spi) {
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
      p_platform->dev_spi->beginTransaction(SPISettings(p_platform->spi_speed, MSBFIRST, SPI_MODE3));
      digitalWrite(p_platform->cs_pin, LOW);
      p_platform->dev_spi->transfer(&data_write, 2);
      for (uint16_t i = 0; i < data_size; i++) {
        *(p_values + i + position) = p_platform->dev_spi->transfer(0x00);
      }
      digitalWrite(p_platform->cs_pin, HIGH);
    }
    return 0;
  }
  return 1;
}

void VL53L8CX::SwapBuffer(
  uint8_t     *buffer,
  uint16_t     size)
{
  uint32_t i, tmp;
  /* Example of possible implementation using <string.h> */
  for (i = 0; i < size; i = i + 4) {
    tmp = (
            buffer[i] << 24)
          | (buffer[i + 1] << 16)
          | (buffer[i + 2] << 8)
          | (buffer[i + 3]);
    memcpy(&(buffer[i]), &tmp, 4);


  }

}


uint8_t VL53L8CX::WaitMs(
  VL53L8CX_Platform *p_platform,
  uint32_t TimeMs)
{
  (void)p_platform;
  delay(TimeMs);

  return 0;
}
