#include "nj-nrf24.h"

#include "driver/spi_master.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  spi_device_handle_t spi;
} nrf24_data_t;

static nrf24_data_t* nrf = 0;

#define CS 5
#define CE 19
#define MISO 22
#define MOSI 23
#define SCK 18

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
    goto exit; \
  }

int nrf24_setup()
{
  int res = 0;
  esp_err_t spi_res;

  printf("nrf24_setup\n");
  if(nrf)
  {
    res = NRF24_ERROR_ALREADY_SETUP;
    goto exit;
  }
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
exit:
  return res;
}

void nrf24_teardown()
{
  printf("nrf24_teardown\n");
  if(nrf)
  {
    spi_bus_remove_device(nrf->spi);
    spi_bus_free(VSPI_HOST);
    free(nrf);
    nrf = 0;
  }
}


int nrf24_run_spoke(const char local_address[5], const char hub_address[5])
{
  int res = 0;
  struct spi_transaction_t t = { 0 };
  uint32_t cmd = 0xaa5500ff;
  t.length = 32;
  t.tx_buffer = (void*)cmd;
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  res = spi_device_transmit(nrf->spi, &t);  //Transmit!
  ESP_ERROR_CHECK(res);
//exit:
  return 0;
}
