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

#include "nj-bmp280.h"
#include "nj-i2c.h"

#include <freertos/task.h>

#define BMP280_ID 0xD0
#define BMP280_RESET 0xE0
#define BMP280_CONTROL_MEASUREMENT 0xF4
#define BMP280_CONFIG 0xF5
#define BMP280_STATUS 0xF3

#define BMP280_FILTER_OFF 0x00
#define BMP280_FILTER_COEFF_2 0x01
#define BMP280_FILTER_COEFF_4 0x02
#define BMP280_FILTER_COEFF_8 0x03
#define BMP280_FILTER_COEFF_16 0x04
#define BMP280_PRESS_MSB 0xF7
#define BMP280_PRESS_LSB 0xF8

int newjoy_task_setup_bmp280(nj_task_def_t* task, int period)
{
  uint8_t id_register;
  read_byte_from_device_register(
    task->i2c,
    task->address,
    BMP280_ID,
    &id_register
    );
//  printf("id_register: %i\n", id_register);
//  assert(id_register == 'X');
  write_byte_to_device_register(
    task->i2c,
    task->address,
    BMP280_RESET,
    0xb6
    );

  const TickType_t delay = 50 / portTICK_PERIOD_MS;
  vTaskDelay(delay);

  uint8_t t_sb = 0b000; // 0.5 ms standby
  uint8_t filter = 0;
  uint8_t spi = 0x00; // no SPI interface
  uint8_t config_value = t_sb << 5 | filter << 2 | spi;
  write_byte_to_device_register(
    task->i2c,
    task->address,
    BMP280_CONFIG,
    config_value
    );

  uint8_t osrs_t = 0b000; // no temperature reading
  uint8_t osrs_p = 0b011; // 4 times oversampling
  uint8_t mode = 0b11; // normal mode
  uint8_t control_value = osrs_t << 5 | osrs_p << 2 | mode;
  write_byte_to_device_register(
    task->i2c,
    task->address,
    BMP280_CONTROL_MEASUREMENT,
    control_value
    );
  return 0;
}


void newjoy_task_bmp280(nj_task_def_t* task, uint8_t *buffer)
{
  uint8_t pressure_buf[3];
  read_from_device_register_into_buffer(
    task->i2c,
    task->address,
    BMP280_PRESS_MSB,
    pressure_buf,
    sizeof(pressure_buf)
    );
  // the register is in theory
  // 24 bit, but that only works
  // in the slow-reading modes. So
  // we take the upper two bytes,
  // endian-swapped, and fill up with
  // zeros to maintain a 4-sized
  // buffer utilization that helps us
  // with byte-alignment
  buffer[0] = pressure_buf[1];
  buffer[1] = pressure_buf[0];
  buffer[2] = 0;
  buffer[3] = 0;
}


void newjoy_task_teardown_bmp280(nj_task_def_t* task)
{
}
