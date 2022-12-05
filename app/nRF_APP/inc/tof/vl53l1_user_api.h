#ifndef VL53L1_USER_API_H_
#define VL53L1_USER_API_H_
#include <stdint.h>

#define VL53L1X_EN_PIN                6
#define VL53L1X_SCL_PIN               7
#define VL53L1X_SDA_PIN               12
#define VL53L1X_I2C_ADDR              0x29  //0x52>>1

void twi_vl53l1_init(void);

void vl53l1_device_config(void);

void vl53l1_nrf5_init(void);

//!获得距离值
uint16_t vl53l1_getvalue(void);
 
#endif /* VL53L1_USER_API_H_ */
