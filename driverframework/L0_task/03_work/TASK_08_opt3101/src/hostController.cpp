/*!
* \file hostController.cpp
* \author  Karthik Rajagopal <krthik@ti.com>
* \version 0.9.1
*
* \section COPYRIGHT
* TEXAS INSTRUMENTS TEXT FILE LICENSE
* Copyright (c) 2018 Texas Instruments Incorporated
* All rights reserved not granted herein.
* Limited License.
* Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive license under copyrights and patents it now or hereafter owns or controls to make, have made, use, import, offer to sell and sell ("Utilize") this software subject to the terms herein.  With respect to the foregoing patent license, such license is granted  solely to the extent that any such patent is necessary to Utilize the software alone.  The patent license shall not apply to any combinations which include this software, other than combinations with devices manufactured by or for TI ("TI Devices").  No hardware patent is licensed hereunder.
* Redistributions must preserve existing copyright notices and reproduce this license (including the above copyright notice and the disclaimer and (if applicable) source code license limitations below) in the documentation and/or other materials provided with the distribution
* Redistribution and use in binary form, without modification, are permitted provided that the following conditions are met:
* * No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any software provided in binary form.
* * any redistribution and use are licensed by TI for use only with TI Devices.
* * Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
* If software source code is provided to you, modification and redistribution of the source code are permitted provided that the following conditions are met:
* * any redistribution and use of the source code, including any resulting derivative works, are licensed by TI for use only with TI Devices.
* * any redistribution and use of any object code compiled from the source code and any resulting derivative works, are licensed by TI for use only with TI Devices.
* Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or promote products derived from this software without specific prior written permission.
* DISCLAIMER.
* THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* \section DESCRIPTION
* This file contains the hostController class methods
*/

#include "hostController.h"

#include <rtdevice.h>
#include <rtthread.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "I2cInterface.h"

hostController host;

namespace {

static I2cInterface_t g_i2c_interface = {0};
static rt_mutex_t g_i2c_mutex = RT_NULL;
static rt_base_t g_reset_pin = PIN_NONE;
static bool g_i2c_ready = false;

class ScopedMutex {
 public:
  explicit ScopedMutex(rt_mutex_t mutex) : mutex_(mutex) {
    if (mutex_) {
      rt_mutex_take(mutex_, RT_WAITING_FOREVER);
    }
  }
  ~ScopedMutex() {
    if (mutex_) {
      rt_mutex_release(mutex_);
    }
  }

 private:
  rt_mutex_t mutex_;
};

static void ensure_mutex_created() {
  if (g_i2c_mutex == RT_NULL) {
    g_i2c_mutex = rt_mutex_create("opt3101_i2c", RT_IPC_FLAG_PRIO);
    if (g_i2c_mutex == RT_NULL) {
      rt_kprintf("[OPT3101][ERR] create mutex failed\n");
    }
  }
}

static void configure_reset_pin(void) {
#ifdef WORK_TASK_OPT3101_RESET_PIN
  if ((g_reset_pin == PIN_NONE) && (WORK_TASK_OPT3101_RESET_PIN[0] != '\0')) {
    g_reset_pin = rt_pin_get(WORK_TASK_OPT3101_RESET_PIN);
    if (g_reset_pin == PIN_NONE) {
      rt_kprintf("[OPT3101][WARN] reset pin %s not found\n", WORK_TASK_OPT3101_RESET_PIN);
      return;
    }
    rt_pin_mode(g_reset_pin, PIN_MODE_OUTPUT);
    rt_pin_write(g_reset_pin, PIN_HIGH);
    rt_kprintf("[OPT3101] reset pin %s mapped to %ld\n", WORK_TASK_OPT3101_RESET_PIN, g_reset_pin);
  }
#endif
}

static void apply_i2c_speed(void) {
#ifdef WORK_TASK_OPT3101_I2C_SPEED
  if (WORK_TASK_OPT3101_I2C_SPEED > 0 && g_i2c_interface.i2c_dev != RT_NULL) {
    if (set_i2c_speed(g_i2c_interface.i2c_dev, WORK_TASK_OPT3101_I2C_SPEED) != RT_EOK) {
      rt_kprintf("[OPT3101][WARN] failed to set i2c speed to %d Hz\n", WORK_TASK_OPT3101_I2C_SPEED);
    } else {
      rt_kprintf("[OPT3101] I2C speed set to %d Hz\n", WORK_TASK_OPT3101_I2C_SPEED);
    }
  }
#endif
}

static rt_err_t ensure_i2c_ready(void) {
  if (g_i2c_ready) {
    return RT_EOK;
  }

  ensure_mutex_created();
  if (g_i2c_mutex == RT_NULL) {
    return -RT_ERROR;
  }

  ScopedMutex lock(g_i2c_mutex);

  if (g_i2c_ready) {
    return RT_EOK;
  }

  rt_err_t ret = get_i2c_interface(WORK_TASK_OPT3101_I2C_NAME, OPT3101_I2C_SLAVEADDRESS, &g_i2c_interface);
  if (ret != RT_EOK) {
    rt_kprintf("[OPT3101][ERR] get_i2c_interface %s failed (%d)\n", WORK_TASK_OPT3101_I2C_NAME, ret);
    return ret;
  }

  apply_i2c_speed();
  configure_reset_pin();

  g_i2c_ready = true;
  rt_kprintf("[OPT3101] I2C ready on %s addr 0x%02X\n", WORK_TASK_OPT3101_I2C_NAME, OPT3101_I2C_SLAVEADDRESS);
  return RT_EOK;
}

static uint32_t pack_bytes_to_u24(const uint8_t buffer[3]) {
  return (static_cast<uint32_t>(buffer[0]) << 16) |
         (static_cast<uint32_t>(buffer[1]) << 8) |
         static_cast<uint32_t>(buffer[2]);
}

}  // namespace

hostController::hostController() = default;

void hostController::writeI2C(uint8_t address, uint32_t data) {
  if (ensure_i2c_ready() != RT_EOK) {
    rt_kprintf("[OPT3101][ERR] writeI2C aborted, bus not ready\n");
    return;
  }

  uint8_t buffer[3];
  buffer[0] = static_cast<uint8_t>((data >> 16) & 0xFF);
  buffer[1] = static_cast<uint8_t>((data >> 8) & 0xFF);
  buffer[2] = static_cast<uint8_t>(data & 0xFF);

  ScopedMutex lock(g_i2c_mutex);
  if (i2c_write_reg8_mult_pack(g_i2c_interface, address, buffer, sizeof(buffer)) != RT_EOK) {
    rt_kprintf("[OPT3101][ERR] I2C write addr 0x%02X failed\n", address);
  }
}

uint32_t hostController::readI2C(uint8_t address) {
  if (ensure_i2c_ready() != RT_EOK) {
    rt_kprintf("[OPT3101][ERR] readI2C aborted, bus not ready\n");
    return 0;
  }

  uint8_t buffer[3] = {0};
  ScopedMutex lock(g_i2c_mutex);
  if (i2c_read_reg8_mult_pack(g_i2c_interface, address, buffer, sizeof(buffer)) != RT_EOK) {
    rt_kprintf("[OPT3101][ERR] I2C read addr 0x%02X failed\n", address);
    return 0;
  }

  return pack_bytes_to_u24(buffer);
}

void hostController::sleep(uint32_t timeInMilliSeconds) {
  rt_thread_mdelay(timeInMilliSeconds);
}

void hostController::sleepDataReadyCounts(uint16_t dataReadyCounts) {
  uint32_t delay = static_cast<uint32_t>(dataReadyCounts) * WORK_TASK_OPT3101_FRAME_TIME_MS;
  rt_thread_mdelay(delay);
}

void hostController::pause() {
#ifdef VERBOSE_MODE
  this->printf("[OPT3101] pause %d ms\n", WORK_TASK_OPT3101_PAUSE_DELAY_MS);
#endif
  rt_thread_mdelay(WORK_TASK_OPT3101_PAUSE_DELAY_MS);
}

void hostController::printfSetColor(uint8_t color) {
  RT_UNUSED(color);
}

void hostController::resetDevice() {
  if (ensure_i2c_ready() != RT_EOK) {
    rt_kprintf("[OPT3101][ERR] resetDevice aborted, bus not ready\n");
    return;
  }

  if (g_reset_pin != PIN_NONE) {
    rt_pin_write(g_reset_pin, PIN_LOW);
    rt_thread_mdelay(WORK_TASK_OPT3101_RESET_PULSE_MS);
    rt_pin_write(g_reset_pin, PIN_HIGH);
    rt_thread_mdelay(WORK_TASK_OPT3101_RESET_RELEASE_MS);
    return;
  }

  rt_kprintf("[OPT3101][WARN] resetDevice called but reset pin not configured\n");
}

void hostController::initialize() {
  (void)ensure_i2c_ready();
}

void hostController::printf(const char* fmt, ...) {
#ifdef VERBOSE_MODE
  va_list args;
  va_start(args, fmt);
  char buffer[256];
  int length = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (length > 0) {
    rt_kprintf("%s", buffer);
  }
#endif
}
