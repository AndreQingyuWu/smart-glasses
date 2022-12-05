#include "motor.h"

void motor_init() {
  nrf_gpio_cfg_output(MOTOR_CTRL_PIN);
  nrf_gpio_pin_clear(MOTOR_CTRL_PIN);
}


void motor_set() {
  nrf_gpio_pin_set(MOTOR_CTRL_PIN);
}

void motor_clear() {
  nrf_gpio_pin_clear(MOTOR_CTRL_PIN);
}