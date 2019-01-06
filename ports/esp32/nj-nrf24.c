#include "nj-nrf24.h"

#include "driver/spi_master.h"
#include "mphalport.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define WORK_BUFFER_SIZE 64

typedef struct {
  spi_device_handle_t spi;
  uint8_t tx_work_buffer[WORK_BUFFER_SIZE];
  uint8_t rx_work_buffer[WORK_BUFFER_SIZE];
} nrf24_data_t;

static nrf24_data_t* nrf = 0;

// hardware setup on the newjoy baseboard
#define CS 5
#define CE 19
#define MISO 22
#define MOSI 23
#define SCK 18

// global configuration settings
#define RETRIES 4
#define RETRY_PAUSE 3 // times 250us
#define CHANNEL 124 // beyond WIFI@2.4GHz
#define PAYLOAD_SIZE 32


// nRF24L01+ registers
#define CONFIG      0x00
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RX_ADDR_P0  0x0a
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define FIFO_STATUS 0x17
#define DYNPD	    0x1c

// CONFIG register
#define EN_CRC      0x08
#define CRCO        0x04
#define PWR_UP      0x02
#define PRIM_RX     0x01

// RF_SETUP register
#define POWER_0     0x00
#define POWER_1     0x02
#define POWER_2     0x04
#define POWER_3     0x06
#define SPEED_1M    0x00
#define SPEED_2M    0x08
#define SPEED_250K  0x20

// STATUS register
#define RX_DR       0x40
#define TX_DS       0x20
#define MAX_RT      0x10

// FIFO_STATUS register
#define RX_EMPTY    0x01

// constants for instructions
#define R_RX_PL_WID  0x60
#define R_RX_PAYLOAD 0x61;
#define W_TX_PAYLOAD 0xa0
#define FLUSH_TX     0xe1
#define FLUSH_RX     0xe2
#define NOP          0xff

#define START_LISTENING_TIMEOUT_US 130

#define SPI_ERROR_CHECK \
  switch(spi_res) \
  { \
  case ESP_OK: \
    res = NRF24_ERROR_OK; \
    break; \
  case ESP_ERR_INVALID_ARG: \
    res = NRF24_ERROR_INVALID_ARG; \
    break; \
  case ESP_ERR_INVALID_STATE: \
    res = NRF24_ERROR_HOST_IN_USE; \
    break; \
  case ESP_ERR_NO_MEM: \
    res = NRF24_ERROR_MALLOC; \
    break; \
  case ESP_ERR_NOT_FOUND: \
    res = NRF24_ERROR_NO_CS_SLOT; \
  default: \
    res = NRF24_ERROR_UNKNOWN; \
  } \
  if(res) \
  { \
    goto close_spi; \
  }


static void nrf24_usleep(uint32_t us)
{
  mp_hal_delay_us(us);
}


static void nrf24_ce(uint8_t value)
{
  gpio_set_level(CE, value);
}


static void nrf24_reg_write(uint8_t reg, uint8_t value)
{
  esp_err_t res;
  struct spi_transaction_t t = { 0 };
  //printf("reg_write %i %i\n", reg, value);
  t.length = 16;
  t.tx_data[0] = 0x20 | reg; // this marks the register to be written
  t.tx_data[1] = value;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(nrf->spi, &t);
  ESP_ERROR_CHECK(res);
}


static void nrf24_reg_write_bytes(uint8_t reg, const uint8_t* buf, size_t len)
{
  esp_err_t res;
  assert(len + 1 < WORK_BUFFER_SIZE);
  //printf("reg_write_bytes %i", reg);
  nrf->tx_work_buffer[0] = 0x20 | reg;
  for(size_t i=0; i < len; ++i)
  {
    nrf->tx_work_buffer[1 + i] = buf[i];
    //printf(" %i", buf[i]);
  }
  //printf("\n");
  struct spi_transaction_t t = { 0 };
  t.length = 8 + len * 8;
  t.tx_buffer = nrf->tx_work_buffer;
  t.rx_buffer = nrf->rx_work_buffer;
  t.flags = 0;
  res = spi_device_transmit(nrf->spi, &t);
  ESP_ERROR_CHECK(res);
}


static uint8_t nrf24_reg_read(uint8_t reg)
{
  esp_err_t res;
  uint8_t ret;
  struct spi_transaction_t t = { 0 };
  t.length = 16;
  t.tx_data[0] = reg;
  t.tx_data[1] = 0;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(nrf->spi, &t);
  ESP_ERROR_CHECK(res);
  ret = t.rx_data[1];
  //printf("nrf24_reg_read: %#2x: %#2x\n", reg, ret);
  return ret;
}


static void nrf24_reg_read_bytes(uint8_t reg, uint8_t* buf, size_t len)
{
  esp_err_t res;
  assert(len + 1 < WORK_BUFFER_SIZE);
  nrf->tx_work_buffer[0] = reg;
  for(size_t i=0; i < len; ++i)
  {
    nrf->tx_work_buffer[1 + i] = 0;
    nrf->rx_work_buffer[1 + i] = 0;
  }
  struct spi_transaction_t t = { 0 };
  t.length = 8 + len * 8;
  t.tx_buffer = nrf->tx_work_buffer;
  t.rx_buffer = nrf->rx_work_buffer;
  t.flags = 0;
  res = spi_device_transmit(nrf->spi, &t);
  for(size_t i=0; i < len; ++i)
  {
    buf[i] = nrf->rx_work_buffer[1 + i];
  }
  ESP_ERROR_CHECK(res);
}


void nrf24_set_power_speed(uint8_t power, uint8_t speed)
{
  uint8_t setup = nrf24_reg_read(RF_SETUP) & 0b11010001;
  nrf24_reg_write(RF_SETUP, setup | power | speed);
}


void nrf24_set_crc(uint8_t crc_length)
{
  uint8_t config = nrf24_reg_read(CONFIG) & ~(CRCO | EN_CRC);
  switch(crc_length)
  {
  case 0:
    break;
  case 1:
    config |= EN_CRC;
    break;
  case 2:
    config |= EN_CRC | CRCO;
    break;
  default:
    assert(0);
  }
  nrf24_reg_write(CONFIG, config);
}


void nrf24_set_channel(uint8_t channel)
{
  if(channel > 125)
  {
    channel = 125;
  }
  nrf24_reg_write(RF_CH, channel);
}


void nrf24_flush_rx()
{
  esp_err_t res;
  struct spi_transaction_t t = { 0 };
  t.length = 8;
  t.tx_data[0] = FLUSH_RX;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(nrf->spi, &t);
  ESP_ERROR_CHECK(res);
}


void nrf24_flush_tx()
{
  esp_err_t res;
  struct spi_transaction_t t = { 0 };
  t.length = 8;
  t.tx_data[0] = FLUSH_TX;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(nrf->spi, &t);
  ESP_ERROR_CHECK(res);
}


static void  nrf24_open_tx_pipe(const char address[5], uint8_t payload_size)
{
  uint8_t h[5];
  nrf24_reg_write_bytes(RX_ADDR_P0, (const uint8_t*)address, 5);
  nrf24_reg_read_bytes(RX_ADDR_P0, h, 5);
  /* for(size_t i=0; i < 5; ++i) */
  /* { */
  /*   printf("%#2x:", h[i]); */
  /* } */
  /* printf("\n"); */
  nrf24_reg_write_bytes(TX_ADDR, (const uint8_t*)address, 5);
  nrf24_reg_write(RX_PW_P0, payload_size);
}


static void nrf24_open_rx_pipe(uint8_t pipe_id, const char address[5], uint8_t payload_size)
{
  // I only allow the rx-pipe to be 1-5 because
  // the pipe 0 is always equal to the tx pipe's address
  // I also currently allow just for *one* rx pipe with
  // the full address. I don't need more, and the code
  // is not supposed to be generic.
  assert(1 <= pipe_id && pipe_id < 2);
  nrf24_reg_write_bytes(RX_ADDR_P0 + pipe_id, (const uint8_t*)address, 5);
  nrf24_reg_write(RX_PW_P0 + pipe_id, payload_size);
  nrf24_reg_write(EN_RXADDR, nrf24_reg_read(EN_RXADDR) | (1 << pipe_id));
}


int nrf24_setup(const char local_address[5], const char remote_address[5])
{
  int res = 0;
  esp_err_t spi_res;

  if(nrf)
  {
    res = NRF24_ERROR_ALREADY_SETUP;
    goto exit;
  }

  gpio_pad_select_gpio(CE);
  gpio_set_level(CE, 0);
  gpio_set_direction(CE, GPIO_MODE_OUTPUT);

  nrf = (nrf24_data_t*)malloc(sizeof(nrf24_data_t));
  if(!nrf)
  {
    res = NRF24_ERROR_MALLOC;
    goto exit;
  }
  memset(nrf, 0, sizeof(nrf24_data_t));

  spi_bus_config_t buscfg = {
    .miso_io_num=MISO,
    .mosi_io_num=MOSI,
    .sclk_io_num=SCK,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
  };

  spi_device_interface_config_t devcfg = {
    .clock_speed_hz=4 * 1000*1000,
    .mode=0,                                //SPI mode 0
    .spics_io_num=CS,               //CS pin
    .queue_size=1,
    .flags=0 //SPI_DEVICE_TXBIT_LSBFIRST
  };

  //Initialize the SPI bus
  spi_res = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
  SPI_ERROR_CHECK;
  spi_res = spi_bus_add_device(VSPI_HOST, &devcfg, &nrf->spi);
  SPI_ERROR_CHECK;

  // from here on, the __init__ of nrf24l01.py is lifted
  nrf24_reg_write(SETUP_AW, 0b11);
  if(nrf24_reg_read(SETUP_AW) != 0b11)
  {
    res = NRF24_ERROR_HARDWARE_NOT_RESPONDING;
    goto close_spi;
  }
  nrf24_reg_write(DYNPD, 0);
  nrf24_reg_write(SETUP_RETR, (RETRY_PAUSE << 4) | RETRIES);
  nrf24_set_power_speed(POWER_3, SPEED_2M);
  nrf24_set_crc(2);
  nrf24_reg_write(STATUS, RX_DR | TX_DS | MAX_RT);
  nrf24_set_channel(CHANNEL);
  nrf24_flush_rx();
  nrf24_flush_tx();
  nrf24_open_tx_pipe(local_address, PAYLOAD_SIZE);
  nrf24_open_rx_pipe(1, remote_address, PAYLOAD_SIZE);
  goto exit;
close_spi:
  nrf24_teardown();
exit:
  return res;
}


void nrf24_teardown()
{
  if(nrf)
  {
    spi_bus_remove_device(nrf->spi);
    spi_bus_free(VSPI_HOST);
    free(nrf);
    nrf = 0;
  }
}


void nrf24_start_listening()
{
  assert(nrf);
  nrf24_reg_write(CONFIG, nrf24_reg_read(CONFIG) | PWR_UP | PRIM_RX);
  nrf24_reg_write(STATUS, RX_DR | TX_DS | MAX_RT);
  // TODO: is this necessary?
  /* if self.pipe0_read_addr is not None: */
  /*     self.reg_write_bytes(RX_ADDR_P0, self.pipe0_read_addr) */

  nrf24_flush_rx();
  nrf24_flush_tx();
  nrf24_ce(1);
  nrf24_usleep(START_LISTENING_TIMEOUT_US);
}


void nrf24_stop_listening()
{
  assert(nrf);
  nrf24_ce(0);
  nrf24_flush_rx();
  nrf24_flush_tx();
}


int nrf24_any()
{
  assert(nrf);
  return !(nrf24_reg_read(FIFO_STATUS) & RX_EMPTY);
}
