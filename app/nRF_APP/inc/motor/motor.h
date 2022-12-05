#ifndef _MOTION_H_
#define _MOTION_H_

#include <stdint.h>
#include <stdlib.h>

#include "nrf_gpio.h"
#include "nrf_log.h"


#define MOTOR_CTRL_PIN 20

void motor_init();
void motor_set();
void motor_clear();

#endif