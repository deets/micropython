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

#include "hw_timer.h"
#include "py/mphal.h"
#include "hspi.h"
#include "user_interface.h"


STATIC void cs_off()
{
  gpio_output_set(1 << 4, 0, 1 << 4, 0);
}

STATIC void cs_on()
{
  gpio_output_set(0, 1 << 4, 1 << 4, 0);
}


static int CS;

STATIC void fastdac_write_value(uint8_t channel, uint16_t value)
{
  uint8_t buf = 0;
  uint8_t double_gain = 0;
  uint8_t shutdown = 0;
  channel &= 1;
  uint8_t gain = 0x01 & ! double_gain;
  uint8_t shdn = 0x01 & ! shutdown;
  value &= 0xfff; // 12 bits
  uint8_t msb = (channel << 7) | (buf << 6) | (gain << 5) | (shdn << 4) | (value >> 8);
  uint8_t lsb = value & 0xff;
  cs_on();
  spi_tx8fast(HSPI, msb);
  spi_tx8fast(HSPI, lsb);
  while (spi_busy(HSPI)) {
  }
  cs_off();
}



STATIC void fastdac_timer_cb()
{
  static uint16_t value = 0;
  fastdac_write_value(0, value++);
}


STATIC int fastdac_spi_set_baudrate(int baudrate)
{
  if (baudrate == 80000000L) {
    // Special case for full speed.
    spi_init_gpio(HSPI, SPI_CLK_80MHZ_NODIV);
    spi_clock(HSPI, 0, 0);
  } else {
    uint32_t divider = 40000000L / baudrate;
    uint16_t prediv = MIN(divider, SPI_CLKDIV_PRE + 1);
    uint16_t cntdiv = (divider / prediv) * 2; // cntdiv has to be even
    baudrate = 80000000L / (prediv * cntdiv);
    spi_init_gpio(HSPI, SPI_CLK_USE_DIV);
    spi_clock(HSPI, prediv, cntdiv);
  }
  return baudrate;
}

STATIC void fastdac_spi_init()
{
  spi_tx_byte_order(HSPI, SPI_BYTE_ORDER_HIGH_TO_LOW);
  spi_rx_byte_order(HSPI, SPI_BYTE_ORDER_HIGH_TO_LOW);
  CLEAR_PERI_REG_MASK(SPI_USER(HSPI), SPI_FLASH_MODE | SPI_USR_MISO |
                      SPI_USR_ADDR | SPI_USR_COMMAND | SPI_USR_DUMMY);
  // Clear Dual or Quad lines transmission mode
  CLEAR_PERI_REG_MASK(SPI_CTRL(HSPI), SPI_QIO_MODE | SPI_DIO_MODE |
                      SPI_DOUT_MODE | SPI_QOUT_MODE);
  spi_mode(HSPI, 0, 0);
  fastdac_spi_set_baudrate(1000000);
}


STATIC mp_obj_t fastdac_init()
{
  CS = 0;
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
  PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO4_U);
  gpio_output_set(0, 0, GPIO_ID_PIN(4), 0);

  fastdac_spi_init();
  // the timer doesn't allow us to sleep
  wifi_set_sleep_type(NONE_SLEEP_T);

  hw_timer_init(FRC1_SOURCE, 1);
  hw_timer_set_func(fastdac_timer_cb);
  hw_timer_arm(1000);
  return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(fastdac_init_obj, fastdac_init);


STATIC mp_obj_t fastdac_on()
{
  cs_off();
  return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(fastdac_on_obj, fastdac_on);


STATIC mp_obj_t fastdac_off()
{
  cs_on();
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
