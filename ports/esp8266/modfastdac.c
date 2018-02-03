/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Paul Sokolovsky
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
#include "drivers/dht/dht.h"
#include "uart.h"
#include "user_interface.h"
#include "mem.h"
#include "espneopixel.h"
#include "espapa102.h"
#include "modmachine.h"


STATIC mp_obj_t fastdac_init()
{
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
  PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO4_U);
  gpio_output_set(0, 0, GPIO_ID_PIN(4), 0);
  return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(fastdac_init_obj, fastdac_init);


STATIC mp_obj_t fastdac_on()
{
  gpio_output_set(1 << 4, 0, 1 << 4, 0);
  return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(fastdac_on_obj, fastdac_on);


STATIC mp_obj_t fastdac_off()
{
  gpio_output_set(0, 1 << 4, 1 << 4, 0);
  return mp_const_none;
}


STATIC MP_DEFINE_CONST_FUN_OBJ_0(fastdac_off_obj, fastdac_off);


STATIC const mp_rom_map_elem_t module_globals_table_fastdac[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_fastdac) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&fastdac_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_on), MP_ROM_PTR(&fastdac_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_off), MP_ROM_PTR(&fastdac_off_obj) },
};


STATIC MP_DEFINE_CONST_DICT(module_globals_fastdac, module_globals_table_fastdac);

const mp_obj_module_t mp_module_fastdac = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&module_globals_fastdac,
};
