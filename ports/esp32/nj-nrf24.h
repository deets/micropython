// Copyright: 2019, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

typedef enum {
  NRF24_ERROR_OK,
  NRF24_ERROR_ALREADY_SETUP,
  NRF24_ERROR_MALLOC,
  NRF24_ERROR_INVALID_ARG,
  NRF24_ERROR_HOST_IN_USE,
  NRF24_ERROR_NO_CS_SLOT,
  NRF24_ERROR_HARDWARE_NOT_RESPONDING,
  NRF24_ERROR_UNKNOWN
} nrf24_error_t;


int nrf24_setup(const char local_address[5], const char remote_address[5]);
void nrf24_teardown();
void nrf24_start_listening();
void nrf24_stop_listening();
int nrf24_any();
