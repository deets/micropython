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

#include <stdio.h>

#include "py/gc.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"

#include <esp_task.h>
#include <esp_timer.h>


#define NJ_MAX_TASKS 8

typedef enum
{
  NJ_TASK_MPU6050
} nj_task_type;

typedef struct {
  nj_task_type type;
  mp_obj_t i2c;
  size_t offset;
} nj_task_def_t;

esp_timer_handle_t nj_timer_handle = NULL;
int nj_timer_counter = 0;
int nj_timer_task_count = 0;
uint8_t *nj_buffer;
size_t nj_buffer_size;
size_t nj_task_count = 0;
nj_task_def_t nj_tasks[NJ_MAX_TASKS];

STATIC void newjoy_task_mpu6050(uint8_t *buffer)
{
  for(size_t i=0; i < 6; ++i)
  {
    ++buffer[i];
  }
}

STATIC void nj_task()
{
  ++nj_timer_counter;
  for(size_t i=0; i < nj_task_count; ++i)
  {
    const nj_task_def_t *current = &nj_tasks[i];
    switch(current->type)
    {
    case NJ_TASK_MPU6050:
      newjoy_task_mpu6050(nj_buffer + current->offset);
      break;
    }
  }
}


STATIC mp_obj_t newjoy_init(mp_obj_t arg, mp_obj_t buf)
{
  esp_err_t timer_err;
  mp_int_t ms = mp_obj_get_int(arg);
  if(ms <= 0)
  {
    mp_raise_ValueError("Timer Cycle must be positive integer");
  }
  if(nj_timer_handle)
  {
    mp_raise_msg(&mp_type_OSError, "newjoy already initialised, call deinit() first");
  }
  mp_buffer_info_t write_buffer;
  mp_get_buffer_raise(buf, &write_buffer, MP_BUFFER_RW);
  if(!write_buffer.len)
  {
    mp_raise_ValueError("Buffer must be large enough");
  }
  nj_buffer = (uint8_t*)write_buffer.buf;
  nj_buffer_size = write_buffer.len;
  nj_task_count = 0;

  esp_timer_create_args_t timer_args = {
    .callback = nj_task,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "nj_task"
  };

  timer_err = esp_timer_create(
    &timer_args,
    &nj_timer_handle
    );
  switch(timer_err)
  {
  case ESP_OK:
    break;
  case ESP_ERR_INVALID_ARG:
    mp_raise_msg(&mp_type_OSError, "invalid configuration");
    break;
  case ESP_ERR_INVALID_STATE:
    mp_raise_msg(&mp_type_OSError, "timer library not setup");
    break;
  case ESP_ERR_NO_MEM:
    mp_raise_msg(&mp_type_OSError, "out of memory error");
    break;
  }
  timer_err = esp_timer_start_periodic(
    nj_timer_handle,
    ms * 1000
    );
  switch(timer_err)
  {
  case ESP_OK:
    break;
  case ESP_ERR_INVALID_ARG:
    esp_timer_delete(nj_timer_handle);
    mp_raise_msg(&mp_type_OSError, "time handle invalid");
  case ESP_ERR_INVALID_STATE:
    esp_timer_delete(nj_timer_handle);
    mp_raise_msg(&mp_type_OSError, "timer already running");
    break;
  }
  return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_2(newjoy_init_obj, newjoy_init);

STATIC mp_obj_t newjoy_deinit()
{
  esp_timer_stop(nj_timer_handle);
  esp_timer_delete(nj_timer_handle);
  nj_timer_handle = NULL;
  return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(newjoy_deinit_obj, newjoy_deinit);


STATIC mp_obj_t newjoy_timer_count()
{
  return mp_obj_new_int(nj_timer_counter);
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(newjoy_timer_count_obj, newjoy_timer_count);

STATIC mp_obj_t newjoy_add_task(mp_obj_t i2c, mp_obj_t task_type, mp_obj_t buf_offset)
{
  if(nj_task_count + 1 >= NJ_MAX_TASKS)
  {
    mp_raise_ValueError("Can't register more tasks");
  }
  mp_int_t buffer_offset = mp_obj_get_int(buf_offset);
  nj_task_type type = mp_obj_get_int(task_type);
  size_t buffer_usage = 0;
  switch(type)
  {
  case NJ_TASK_MPU6050:
    buffer_usage = 6; // 6 bytes for the MPU6050
    break;
  default:
    mp_raise_ValueError("Unknown task type");
  }
  if(buffer_offset < 0 || buffer_offset + buffer_usage > nj_buffer_size)
  {
    mp_raise_ValueError("Can't place data into buffer at given offset");
  }
  nj_task_def_t task_def = {
    .i2c = i2c,
    .type = type,
    .offset = buffer_offset
  };
  nj_tasks[nj_task_count++] = task_def;
  return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_3(newjoy_add_task_obj, newjoy_add_task);

STATIC const mp_rom_map_elem_t module_globals_table_newjoy[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_newjoy) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&newjoy_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&newjoy_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_timer_count), MP_ROM_PTR(&newjoy_timer_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_add_task), MP_ROM_PTR(&newjoy_add_task_obj) },
    { MP_ROM_QSTR(MP_QSTR_TASK_MPU6050), MP_ROM_INT(NJ_TASK_MPU6050) },
};

STATIC MP_DEFINE_CONST_DICT(module_globals_newjoy, module_globals_table_newjoy);

const mp_obj_module_t mp_module_newjoy = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&module_globals_newjoy,
};
