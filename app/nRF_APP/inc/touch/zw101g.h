#ifndef _ZW101G_H_
#define _ZW101G_H_

#include <stdint.h>
#include <stdlib.h>

#include "nrf_gpio.h"
#include "nrf_log.h"


#define ZW101G_QC_PIN 19

void zw101g_init();
uint8_t zw101g_read_stat();

#endif