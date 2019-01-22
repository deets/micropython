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
#include "madgwick-ahrs.h"
#include "nj-i2c.h"

#include <string.h>

// CALIBRATION METHOD
// #define AVERAGE_CALIBRATION
// #define NO_CALIBRATION
#define MA_CALIBRATION

// PUT CALIBRATION INTO ACC VALUES
// #define GYRO_CALIBRATION_IN_ACC

// parameters for moving average calibration
#define MA_SHIFT 16
#define MA_SECONDS 5
#define MA_N (200 * MA_SECONDS / 5) // steps per second (must be computed by hand and re-compiled if changed) times half the interval

#define MPU6050_ADDRESS_AD0_LOW 0x68
#define MPU6050_ADDRESS_AD0_HIGH 0x69
#define MPU6050_DEFAULT_ADDRESS MPU6050_ADDRESS_AD0_LOW
#define MPU6050_RA_PWR_MGMT_1  0x6B
#define MPU6050_RA_PWR_MGMT_2  0x6C
#define MPU6050_CLOCK_PLL_XGYRO 0x01
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_SMPLRT_DIV 0x19
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_WHO_AM_I 0x75
#define MPU6050_DEFAULT_SAMPLE_RATE 0x20
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_ACCEL_XOUT_H 0x3B

#define GYRO_CALIBRATION_BUFFER_SIZE 10
#define GYRO_CALIBRATION_VARIANCE 5

typedef enum {
  GYRO_250_FS=0,
  GYRO_500_FS=1,
  GYRO_1000_FS=2,
  GYRO_2000_FS=3
} mpu6050_gyro_fs_t;


typedef enum {
  ACC_2_FS=0,
  ACC_4_FS=1,
  ACC_8_FS=2,
  ACC_16_FS=3
} mpu6050_acc_fs_t;

typedef enum {
  GYRO_UNCALIBRATED,
  GYRO_CALIBRATED
} gyro_calbration_mode_t;

typedef struct {
  mpu6050_gyro_fs_t gyro_fs;
  mpu6050_acc_fs_t acc_fs;
  uint8_t input_buffer[14]; // acc + temp + gyro data
  float acc_correction;
  float gyro_correction;

  gyro_calbration_mode_t gyro_calbration_mode;

  // for average calibration
  int16_t gyro_calibration[3];
  int16_t acc_calibration[3];
  int16_t gyro_calibration_buffer[3 * GYRO_CALIBRATION_BUFFER_SIZE];
  int16_t acc_calibration_buffer[3 * GYRO_CALIBRATION_BUFFER_SIZE];
  size_t gyro_calibration_buffer_fill;

  // for ma calibration
  int32_t ma_gyro_calibration[3];
  int32_t ma_acc_calibration[3];
  int32_t ma_step;
  int32_t ma_total_steps;

  madgwick_data_t filter_data;
} mpu6050_task_data_t;


STATIC int16_t compute_variance(int16_t* buffer)
{
  int32_t accu = 0;
  for(size_t i=0; i < GYRO_CALIBRATION_BUFFER_SIZE - 1; ++i)
  {
    accu += abs(buffer[i * 3] - buffer[(i + 1) * 3]);
  }
  accu /= GYRO_CALIBRATION_BUFFER_SIZE - 1;
  return (int16_t)accu;
}


STATIC int16_t compute_average(int16_t* buffer)
{
  int32_t accu = 0;
  for(size_t i=0; i < GYRO_CALIBRATION_BUFFER_SIZE; ++i)
  {
    accu += buffer[i * 3];
  }
  accu /= GYRO_CALIBRATION_BUFFER_SIZE;
  return (int16_t)accu;
}

STATIC void compute_average_calibration(mpu6050_task_data_t* task_data, int16_t* word_access)
{
    for(size_t i=0; i < 3; ++i)
    {
      task_data->gyro_calibration_buffer[task_data->gyro_calibration_buffer_fill * 3 + i] = word_access[3 + 1 + i];  // 3 acc + 1 temp
      task_data->acc_calibration_buffer[task_data->gyro_calibration_buffer_fill * 3 + i] = word_access[i];
    }
    task_data->gyro_calibration_buffer_fill = (task_data->gyro_calibration_buffer_fill + 1) % GYRO_CALIBRATION_BUFFER_SIZE;
    // if we reach zero, we have sampled one full ring-buffer of data, so try & compute the variance, and assume calibration
    // if all of them are below a threshold
    if(compute_variance(&task_data->gyro_calibration_buffer[0]) < GYRO_CALIBRATION_VARIANCE && \
       compute_variance(&task_data->gyro_calibration_buffer[1]) < GYRO_CALIBRATION_VARIANCE && \
       compute_variance(&task_data->gyro_calibration_buffer[2]) < GYRO_CALIBRATION_VARIANCE)
    {
      task_data->gyro_calbration_mode = GYRO_CALIBRATED;
      // I piggy-back on the gyros being stable, as I presume there is also no acceleration going on then
      for(size_t i=0; i < 3; ++i)
      {
        task_data->gyro_calibration[i] = compute_average(&task_data->gyro_calibration_buffer[i]);
        task_data->acc_calibration[i] = compute_average(&task_data->acc_calibration_buffer[i]);
      }
      task_data->acc_calibration[2] -= (int16_t)(task_data->acc_correction) ; // assume level orientation!
    }
}

STATIC void compute_ma(int32_t* ma, int32_t value, int32_t n)
{
  *ma = *ma + ((value << MA_SHIFT) - *ma) / n;
}


STATIC void compute_ma_calibration(mpu6050_task_data_t* task_data, int16_t* word_access)
{
  if(++task_data->ma_step >= task_data->ma_total_steps)
  {
    task_data->gyro_calbration_mode = GYRO_CALIBRATED;
    for(size_t i=0; i < 3; ++i)
    {
      task_data->gyro_calibration[i] = task_data->ma_gyro_calibration[i] >> MA_SHIFT;
      task_data->acc_calibration[i] = task_data->ma_acc_calibration[i] >> MA_SHIFT;
    }
    task_data->acc_calibration[2] -= (int16_t)(task_data->acc_correction) ; // assume level orientation!
  }
  else
  {
    for(size_t i=0; i < 3; ++i)
    {
      compute_ma(&task_data->ma_gyro_calibration[i], word_access[3 + 1 + i], MA_N); // 3 acc + 1 temp
      compute_ma(&task_data->ma_acc_calibration[i], word_access[i], MA_N);
    }
  }
}



void newjoy_task_mpu6050(nj_task_def_t* task, uint8_t *buffer)
{
  mpu6050_task_data_t* task_data = (mpu6050_task_data_t*)task->task_data;
  read_from_device_register_into_buffer(
    task->i2c,
    task->address,
    MPU6050_RA_ACCEL_XOUT_H,
    task_data->input_buffer,
    sizeof(task_data->input_buffer)
    );
  // swap endianess
  for(size_t i=0; i < 7; ++i)
  {
    uint8_t h = task_data->input_buffer[i*2];
    task_data->input_buffer[i*2]= task_data->input_buffer[i*2 + 1];
    task_data->input_buffer[i*2 + 1] = h;
  }
  float gyro_data[3];
  float acc_data[3];

  int16_t* word_access = (int16_t*)task_data->input_buffer;

  switch(task_data->gyro_calbration_mode)
  {
  case GYRO_UNCALIBRATED:
    // when uncalibrated, no readings occur!
    gyro_data[0] = gyro_data[1] = gyro_data[2] = 0.0f;
    acc_data[0] = acc_data[1] = acc_data[2] = 0.0f;

    #ifdef AVERAGE_CALIBRATION
    compute_average_calibration(task_data, word_access);
    #endif
    #ifdef MA_CALIBRATION
    compute_ma_calibration(task_data, word_access);
    #endif
    #ifdef NO_CALIBRATION
    for(int i=0; i < 3; ++i)
    {
      task_data->gyro_calibration[i] = 0;
      task_data->acc_calibration[i] = 0;
    }
    task_data->gyro_calbration_mode = GYRO_CALIBRATED;
    #endif
    break;
  case GYRO_CALIBRATED:
    for(size_t i=0; i < 3; ++i)
    {
      gyro_data[i] = ((float)(word_access[3 + 1 + i] - task_data->gyro_calibration[i])) / task_data->gyro_correction;
      acc_data[i] = ((float)(word_access[i] - task_data->acc_calibration[i])) / task_data->acc_correction;
    }
    madgwick_ahrs_update_imu(
      &task_data->filter_data,
      gyro_data[0], gyro_data[1], gyro_data[2],
      acc_data[0], acc_data[1], acc_data[2]
      );
    float* quaternion_data = (float*)buffer;
    *quaternion_data++ = task_data->filter_data.q0;
    *quaternion_data++ = task_data->filter_data.q1;
    *quaternion_data++ = task_data->filter_data.q2;
    *quaternion_data++ = task_data->filter_data.q3;
#ifdef GYRO_CALIBRATION_IN_ACC
    *quaternion_data++ = task_data->gyro_calibration[0];
    *quaternion_data++ = task_data->gyro_calibration[1];
    *quaternion_data++ = task_data->gyro_calibration[2];
#else
    *quaternion_data++ = acc_data[0];
    *quaternion_data++ = acc_data[1];
    *quaternion_data++ = acc_data[2];
#endif
    break;
  }
}


int newjoy_task_setup_mpu6050(nj_task_def_t* task, int period)
{
  int res;
  uint8_t identity;
  // first, identify ourselves. We should
  // get our own address back.
  res = read_byte_from_device_register(
    task->i2c,
    task->address,
    MPU6050_RA_WHO_AM_I,
    &identity
    );
  if(res)
  {
    return res;
  }
  if(identity != MPU6050_DEFAULT_ADDRESS && identity != 0x72)
  {
    return -1;
  }

  mpu6050_task_data_t* task_data = task->task_data = malloc(sizeof(mpu6050_task_data_t));
  // disable sleep mode and select clock source
  write_byte_to_device_register(
    task->i2c,
    task->address,
    MPU6050_RA_PWR_MGMT_1,
    MPU6050_CLOCK_PLL_XGYRO
    );

  // enable all sensors
  write_byte_to_device_register(
    task->i2c,
    task->address,
    MPU6050_RA_PWR_MGMT_2,
    0
    );

  // set sampling rate
  write_byte_to_device_register(
    task->i2c,
    task->address,
    MPU6050_RA_SMPLRT_DIV,
    MPU6050_DEFAULT_SAMPLE_RATE
    );

  // enable dlpf
  write_byte_to_device_register(
    task->i2c,
    task->address,
    MPU6050_RA_CONFIG,
    1
    );

  // setup gyro
  task_data->gyro_fs = GYRO_1000_FS;
  uint8_t gyro_range;
  read_byte_from_device_register(
    task->i2c,
    task->address,
    MPU6050_RA_GYRO_CONFIG,
    &gyro_range
    );
  gyro_range |= ~(3 << 3);
  gyro_range |= task_data->gyro_fs << 3;
  write_byte_to_device_register(
    task->i2c,
    task->address,
    MPU6050_RA_GYRO_CONFIG,
    gyro_range
    );
  switch(task_data->gyro_fs)
  {
  case GYRO_250_FS:
    task_data->gyro_correction = 32768 / 250.0;
    break;
  case GYRO_500_FS:
    task_data->gyro_correction = 32768 / 500.0;
    break;
  case GYRO_1000_FS:
    task_data->gyro_correction = 32768 / 1000.0;
    break;
  case GYRO_2000_FS:
    task_data->gyro_correction = 32768 / 2000.0;
    break;
  }

  task_data->acc_fs = ACC_4_FS;
  uint8_t acc_range;
  read_byte_from_device_register(
    task->i2c,
    task->address,
    MPU6050_RA_ACCEL_CONFIG,
    &acc_range
    );
  acc_range |= ~(3 << 3);
  acc_range |= task_data->acc_fs << 3;
  write_byte_to_device_register(
    task->i2c,
    task->address,
    MPU6050_RA_ACCEL_CONFIG,
    acc_range
    );

  switch(task_data->acc_fs)
  {
  case ACC_2_FS:
    task_data->acc_correction = 32768.0 / 2;
    break;
  case ACC_4_FS:
    task_data->acc_correction = 32768.0 / 4;
    break;
  case ACC_8_FS:
    task_data->acc_correction = 32768.0 / 8;
    break;
  case ACC_16_FS:
    task_data->acc_correction = 32768.0 / 16;
    break;
  }

  task_data->gyro_calibration_buffer_fill = 0;
  task_data->gyro_calbration_mode = GYRO_UNCALIBRATED;
  madgwick_ahrs_init(
    &task_data->filter_data,
    1000 / period // period is in ms
    );

  task_data->ma_step = 0;
  task_data->ma_total_steps = MA_SECONDS * 1000 / period;
  for(size_t i=0; i < 3; ++i)
  {
    task_data->ma_gyro_calibration[i] = 0;
    task_data->ma_acc_calibration[i] = 0;
  }
  return 0;
}

void newjoy_task_teardown_mpu6050(nj_task_def_t* task)
{
  if(task->task_data)
  {
    free(task->task_data);
  }
}
