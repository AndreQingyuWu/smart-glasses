#ifndef BMI160_H_
#define BMI160_H_

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "app_uart.h"
#include "bmi160_defs.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"


void twi_bmi160_init(void);

void bmi160_device_config(uint8_t ACC_ODR, uint8_t GYR_ODR);

void bmi160_nrf5_init(uint8_t ACC_ODR, uint8_t GYR_ODR);

void bmi160_fifo_config(const uint16_t fifo_buffer_len);

void bmi160_read_fifo(struct bmi160_sensor_data *acc_data, uint8_t *acc_len,
                      struct bmi160_sensor_data *gyr_data, uint8_t *gyr_len,
                      uint32_t *sensor_time, uint8_t *skipped_frame_count);

int8_t read_fifo_data(void);
// original driver

int8_t bmi160_init(struct bmi160_dev *dev);

int8_t bmi160_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len,
                       const struct bmi160_dev *dev);

int8_t bmi160_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len,
                       const struct bmi160_dev *dev);

int8_t bmi160_soft_reset(struct bmi160_dev *dev);

int8_t bmi160_set_sens_conf(struct bmi160_dev *dev);

int8_t bmi160_set_power_mode(struct bmi160_dev *dev);

int8_t bmi160_get_power_mode(struct bmi160_pmu_status *pmu_status,
                             const struct bmi160_dev *dev);

int8_t bmi160_get_sensor_data(uint8_t select_sensor,
                              struct bmi160_sensor_data *accel,
                              struct bmi160_sensor_data *gyro,
                              const struct bmi160_dev *dev);

int8_t bmi160_set_int_config(struct bmi160_int_settg *int_config,
                             struct bmi160_dev *dev);

int8_t bmi160_set_step_counter(uint8_t step_cnt_enable,
                               const struct bmi160_dev *dev);

int8_t bmi160_read_step_counter(uint16_t *step_val,
                                const struct bmi160_dev *dev);

int8_t bmi160_aux_read(uint8_t reg_addr, uint8_t *aux_data, uint16_t len,
                       const struct bmi160_dev *dev);

int8_t bmi160_aux_write(uint8_t reg_addr, uint8_t *aux_data, uint16_t len,
                        const struct bmi160_dev *dev);

int8_t bmi160_aux_init(const struct bmi160_dev *dev);

int8_t bmi160_set_aux_auto_mode(uint8_t *data_addr, struct bmi160_dev *dev);

int8_t bmi160_config_aux_mode(const struct bmi160_dev *dev);

int8_t bmi160_read_aux_data_auto_mode(uint8_t *aux_data,
                                      const struct bmi160_dev *dev);

int8_t bmi160_perform_self_test(uint8_t select_sensor, struct bmi160_dev *dev);

int8_t bmi160_get_fifo_data(struct bmi160_dev const *dev);

int8_t bmi160_set_fifo_flush(const struct bmi160_dev *dev);

int8_t bmi160_set_fifo_config(uint8_t config, uint8_t enable,
                              struct bmi160_dev const *dev);

int8_t bmi160_set_fifo_down(uint8_t fifo_down, const struct bmi160_dev *dev);

int8_t bmi160_set_fifo_wm(uint8_t fifo_wm, const struct bmi160_dev *dev);

int8_t bmi160_extract_accel(struct bmi160_sensor_data *accel_data,
                            uint8_t *accel_length,
                            struct bmi160_dev const *dev);

int8_t bmi160_extract_gyro(struct bmi160_sensor_data *gyro_data,
                           uint8_t *gyro_length, struct bmi160_dev const *dev);

int8_t bmi160_extract_aux(struct bmi160_aux_data *aux_data, uint8_t *aux_len,
                          struct bmi160_dev const *dev);

int8_t bmi160_start_foc(const struct bmi160_foc_conf *foc_conf,
                        struct bmi160_offsets *offset,
                        struct bmi160_dev const *dev);

int8_t bmi160_get_offsets(struct bmi160_offsets *offset,
                          const struct bmi160_dev *dev);

int8_t bmi160_set_offsets(const struct bmi160_foc_conf *foc_conf,
                          const struct bmi160_offsets *offset,
                          struct bmi160_dev const *dev);

int8_t bmi160_set_offsets(const struct bmi160_foc_conf *foc_conf,
                          const struct bmi160_offsets *offset,
                          struct bmi160_dev const *dev);

int8_t bmi160_update_nvm(struct bmi160_dev const *dev);

int8_t bmi160_get_int_status(enum bmi160_int_status_sel int_status_sel,
                             union bmi160_int_status *int_status,
                             struct bmi160_dev const *dev);

#endif /* BMI160_H_ */

// original driver

/*

//BMI160

#define 	BMI160_LTR303_TWI_ID 1 #define 	BMI160_IIC_ID 0x68 #define
BMI160_SCL 14 #define 	BMI160_SDA
30 #define		BMI160_INT
8

//LTR303
#define				LTR303_IIC_ID                       0x29




typedef signed short      s16;
typedef signed char       s8;
typedef unsigned short    u16;
typedef signed int        s32;
typedef unsigned int      u32;





void Twi_bmi160_init(void);

unsigned short bmi_160_init(void);

void bmi160_get_data(s16 *data_xx ,s16 *data_yy ,s16 *data_zz);
void bmi160_read(s8 *Angle_D,s8 *Angle_W);


u16 LTR303ALS_Lux_Get(void);


void Low_Power_TWI_Instance_1(void);
void bmi160_init_only_step(void);
u16 read_step(void);
void Reset_bmi160(void);
void bmi160_lowpower(void);
void bmi160_step_cnt_mode(void);


#endif
*/
