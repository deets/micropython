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


#define NJ_TASK_PRIORITY        (ESP_TASK_PRIO_MIN + 1)
#define NJ_TASK_STACK_SIZE      (1024)
#define NJ_TASK_STACK_LEN       (NJ_TASK_STACK_SIZE / sizeof(StackType_t))
#define NJ_TASK_CORE 0 // TODO: We are bound to 1 as uP is running on core 0, see main.c

esp_timer_handle_t nj_timer_handle = NULL;

STATIC void nj_task()
{
  printf("nj_task\n");
}

STATIC mp_obj_t newjoy_init(mp_obj_t arg)
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

STATIC MP_DEFINE_CONST_FUN_OBJ_1(newjoy_init_obj, newjoy_init);

STATIC mp_obj_t newjoy_deinit()
{
  esp_timer_stop(nj_timer_handle);
  esp_timer_delete(nj_timer_handle);
  nj_timer_handle = NULL;
  return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(newjoy_deinit_obj, newjoy_deinit);


STATIC const mp_rom_map_elem_t module_globals_table_newjoy[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_newjoy) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&newjoy_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&newjoy_deinit_obj) },
};

STATIC MP_DEFINE_CONST_DICT(module_globals_newjoy, module_globals_table_newjoy);

const mp_obj_module_t mp_module_newjoy = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&module_globals_newjoy,
};
