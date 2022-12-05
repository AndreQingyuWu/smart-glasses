#include "zw101g.h"

void zw101g_init() {
  nrf_gpio_cfg_input(ZW101G_QC_PIN, NRF_GPIO_PIN_PULLUP);
}

uint8_t zw101g_read_stat() {
  return nrf_gpio_pin_read(ZW101G_QC_PIN);
}