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


#include "platform.h"


uint8_t VL53L8CX_RdByte(
  VL53L8CX_Platform *p_platform,
  uint16_t RegisterAdress,
  uint8_t *p_value)
{
  return p_platform->Read(p_platform->handle, RegisterAdress, p_value, 1U);
}

uint8_t VL53L8CX_WrByte(
  VL53L8CX_Platform *p_platform,
  uint16_t RegisterAdress,
  uint8_t value)
{
  return p_platform->Write(p_platform->handle, RegisterAdress, &value, 1U);
}

uint8_t VL53L8CX_WrMulti(
  VL53L8CX_Platform *p_platform,
  uint16_t RegisterAdress,
  uint8_t *p_values,
  uint32_t size)
{
  return p_platform->Write(p_platform->handle, RegisterAdress, p_values, size);
}

uint8_t VL53L8CX_RdMulti(
  VL53L8CX_Platform *p_platform,
  uint16_t RegisterAdress,
  uint8_t *p_values,
  uint32_t size)
{
  return p_platform->Read(p_platform->handle, RegisterAdress, p_values, size);
}

void VL53L8CX_SwapBuffer(
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

uint8_t VL53L8CX_WaitMs(
  VL53L8CX_Platform *p_platform,
  uint32_t TimeMs)
{
  return p_platform->Wait(p_platform->handle, TimeMs);
}
