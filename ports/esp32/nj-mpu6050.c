/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Diez B. Roggisch
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "nj-mpu6050.h"
#include "mphalport.h"
#include "extmod/machine_i2c.h"
#include <string.h>

#define MPU6050_ADDRESS_AD0_LOW 0x68
#define MPU6050_ADDRESS_AD0_HIGH 0x69
#define MPU6050_DEFAULT_ADDRESS MPU6050_ADDRESS_AD0_LOW
#define MPU6050_RA_PWR_MGMT_1  0x6B
#define MPU6050_RA_PWR_MGMT_2  0x6C
#define MPU6050_CLOCK_PLL_XGYRO 0x01
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_SMPLRT_DIV 0x19

#define MPU6050_RA_WHO_AM_I 0x75
#define MPU6050_DEFAULT_SAMPLE_RATE 0x20
#define MPU6050_RA_GYRO_XOUT_H 0x43

STATIC int read_from_device_register_into_buffer(mp_obj_t i2c, uint16_t address, uint8_t reg, uint8_t* buf, size_t len)
{
  int res;
  res = mp_machine_soft_i2c_writeto(
    i2c,
    address,
    &reg,
    1,
    false
    );
  // error handling if the initial register communication failed
  // 1 is memaddr length
  if (res != 1) {
    // must generate STOP
    mp_machine_soft_i2c_writeto(i2c, address, NULL, 0, true);
    return res;
  }

  return mp_machine_soft_i2c_readfrom(
    i2c,
    address,
    buf,
    len,
    true
    );
}

STATIC int write_from_buffer_into_device_register(mp_obj_t i2c, uint16_t address, uint8_t reg, uint8_t* buf, size_t len)
{
  int res;
  uint8_t buf2_stack[1 + 12]; // max address size and max buffer size
  buf2_stack[0] = reg;
  memcpy(buf2_stack + 1, buf, len);
  res = mp_machine_soft_i2c_writeto(
    i2c,
    address,
    buf2_stack,
    len + 1,
    true
    );
  if(res < 0)
  {
    return res;
  }
  return 0;
}


STATIC int read_byte_from_device_register(mp_obj_t i2c, uint16_t address, uint8_t reg, uint8_t* buf)
{
  return read_from_device_register_into_buffer(
    i2c,
    address,
    reg,
    buf,
    1
    );
}


STATIC int write_byte_to_device_register(mp_obj_t i2c, uint16_t address, uint8_t reg, uint8_t value)
{
  return write_from_buffer_into_device_register(
    i2c,
    address,
    reg,
    &value,
    1
    );
 }

void newjoy_task_mpu6050(nj_task_def_t* task, uint8_t *buffer)
{
  read_from_device_register_into_buffer(
    task->i2c,
    MPU6050_DEFAULT_ADDRESS,
    MPU6050_RA_GYRO_XOUT_H,
    buffer,
    6
    );
}

int newjoy_task_setup_mpu6050(nj_task_def_t* task)
{
  int res;
  uint8_t identity;
  // first, identify ourselves. We should
  // get our own address back.
  res = read_byte_from_device_register(
    task->i2c,
    MPU6050_DEFAULT_ADDRESS,
    MPU6050_RA_WHO_AM_I,
    &identity
    );
  if(res)
  {
    return res;
  }
  if(identity != MPU6050_DEFAULT_ADDRESS)
  {
    return -1;
  }

  // disable sleep mode and select clock source
  write_byte_to_device_register(
    task->i2c,
    MPU6050_DEFAULT_ADDRESS,
    MPU6050_RA_PWR_MGMT_1,
    MPU6050_CLOCK_PLL_XGYRO
    );

  // enable all sensors
  write_byte_to_device_register(
    task->i2c,
    MPU6050_DEFAULT_ADDRESS,
    MPU6050_RA_PWR_MGMT_2,
    0
    );

  // set sampling rate
  write_byte_to_device_register(
    task->i2c,
    MPU6050_DEFAULT_ADDRESS,
    MPU6050_RA_SMPLRT_DIV,
    MPU6050_DEFAULT_SAMPLE_RATE
    );

  // enable dlpf
  write_byte_to_device_register(
    task->i2c,
    MPU6050_DEFAULT_ADDRESS,
    MPU6050_RA_CONFIG,
    1
    );

        /* # explicitly set accel/gyro range */
        /* self.set_accel_range(MPU6050_ACCEL_FS_2) */
        /* self.set_gyro_range(MPU6050_GYRO_FS_250) */

  return 0;
}

void newjoy_task_teardown_mpu6050(nj_task_def_t* task)
{
}
