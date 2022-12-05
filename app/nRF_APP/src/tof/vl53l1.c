
/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L1 Core and is dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/
#include "vl53l1_api.h"
#include "vl53l1_platform.h"
#include "vl53l1_user_api.h"
#include "vl53l1_def.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "app_util_platform.h"
#include <string.h>

static volatile bool m_xfer_done = false;
static const nrf_drv_twi_t twi_vl53l1 = NRF_DRV_TWI_INSTANCE(0);
static VL53L1_Dev_t device;
static VL53L1_RangingMeasurementData_t device_data;

static void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context) {
  switch (p_event->type) {
    case NRF_DRV_TWI_EVT_DONE:
      if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX) {
        m_xfer_done = true;
//        NRF_LOG_PRINTF("twi tx done!\n");
      } else if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX) {
        m_xfer_done = true;
  //      NRF_LOG_PRINTF("twi rx done!\n");
      }
      break;
    case NRF_DRV_TWI_EVT_ADDRESS_NACK:
      NRF_LOG_PRINTF("Error event: NACK received after sending the address.\n");
      break;
    case NRF_DRV_TWI_EVT_DATA_NACK:
      NRF_LOG_PRINTF(
          " Error event: NACK received after sending a data byte.\n");
      break;
    default:
      break;
  }
}

void twi_vl53l1_init(void) {
  uint32_t err_code;
  const nrf_drv_twi_config_t twi_vl53l1_config = {
      .scl = VL53L1X_SCL_PIN,
      .sda = VL53L1X_SDA_PIN,
      .frequency = NRF_TWI_FREQ_400K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
  };
  err_code =
      nrf_drv_twi_init(&twi_vl53l1, &twi_vl53l1_config, twi_handler, NULL);
  APP_ERROR_CHECK(err_code);
  nrf_drv_twi_enable(&twi_vl53l1);
  NRF_LOG_PRINTF( "twi vl53l1 init success.\n");
} 

void vl53l0_device_config(void) {
	int result;
	device.I2cDevAddr = VL53L1X_I2C_ADDR;
  result = VL53L1_WaitDeviceBooted(&device);
  NRF_LOG_PRINTF("VL53L1_WaitDeviceBooted :%d\n", result);
  result |= VL53L1_DataInit(&device);
  NRF_LOG_PRINTF("VL53L1_DataInit :%d\n", result);
  result |= VL53L1_StaticInit(&device);
  NRF_LOG_PRINTF("VL53L1_StaticInit :%d\n", result);
  result |= VL53L1_SetDistanceMode(&device, VL53L1_DISTANCEMODE_MEDIUM);
  NRF_LOG_PRINTF("VL53L1_SetDistanceMode :%d\n", result);
  result |= VL53L1_SetMeasurementTimingBudgetMicroSeconds(&device, 30000);//设定采样时间
  NRF_LOG_PRINTF("VL53L1_SetMeasurementTimingBudgetMicroSeconds :%d\n", result);
  result |= VL53L1_SetInterMeasurementPeriodMilliSeconds(&device, 100);
  NRF_LOG_PRINTF("VL53L1_SetInterMeasurementPeriodMilliSeconds :%d\n", result);
  result |= VL53L1_StartMeasurement(&device);
  NRF_LOG_PRINTF("VL53L1_StartMeasurement :%d\n", result);
  if (result == VL53L1_ERROR_NONE) {
    NRF_LOG_PRINTF("vl53l0 config success\n");
  } else {
    NRF_LOG_PRINTF("vl53l0 config fail\n");
  } 
}

void vl53l1_nrf5_init() {
  twi_vl53l1_init();
  vl53l0_device_config();
}

uint16_t vl53l1_getvalue(void)
{
  VL53L1_Error result;
  result = VL53L1_WaitMeasurementDataReady(&device);//简单测量
	if(result == VL53L1_ERROR_NONE) {
    NRF_LOG_PRINTF("Measurement Data Ready\n");
  } else { 
    NRF_LOG_PRINTF("Measurement Data Not Ready\n");
  }
	result = VL53L1_GetRangingMeasurementData(&device, &device_data);//简单测量
	if(result == VL53L1_ERROR_NONE) {
    NRF_LOG_PRINTF("Get Measurement Data Success\n");
  } else { 
    NRF_LOG_PRINTF("Get Measurement Data Fail\n");
  }
  VL53L1_ClearInterruptAndStartMeasurement(&device);
  return device_data.RangeMilliMeter;
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    ret_code_t err_code = VL53L1_ERROR_NONE;
    uint8_t index_msb = index >> 8;
    uint8_t index_lsb = (uint8_t)(index & 0x00ff);
    uint8_t twi_buffer[count + 2];
    memset(twi_buffer, 0, count + 2);
    if ((count + 2) > sizeof(twi_buffer)) {
        return VL53L1_ERROR_RANGE_ERROR;
    }
    memcpy(twi_buffer, &index_msb, 1);
    memcpy(twi_buffer + 1, &index_lsb, 1);
    memcpy(twi_buffer + 2, pdata, count);

    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_vl53l1, Dev->I2cDevAddr, twi_buffer, count + 2, false);
    APP_ERROR_CHECK(err_code);
    do {
        __WFE();
    } while (m_xfer_done == false);
    __SEV();
    return err_code;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    ret_code_t err_code = VL53L1_ERROR_NONE;
    uint8_t index_list[2] = {index >> 8, (uint8_t)(index & 0x00ff)};
    
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_vl53l1, Dev->I2cDevAddr, index_list, 2, true);
    APP_ERROR_CHECK(err_code);
    do {
        __WFE();
    } while (m_xfer_done == false);
    __SEV();

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&twi_vl53l1, Dev->I2cDevAddr, pdata, count);
    APP_ERROR_CHECK(err_code);
    do {
        __WFE();
    } while (m_xfer_done == false);
    __SEV();
    return err_code;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(Dev, index, &data, 1);
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
    uint8_t parse_data[2]={	(uint8_t)((uint16_t)(data & 0xff00)>>8) ,	(uint8_t)((uint16_t)data & 0x00ff)};
	return VL53L1_WriteMulti(Dev, index, parse_data, 2);
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {
    uint8_t parse_data[4]={(uint8_t)((data & 0xff000000) >> 24), (uint8_t)((data & 0x00ff0000) >> 16), 
													(uint8_t)((data & 0x0000ff00) >> 8), (uint8_t)((data & 0x000000ff))};
	return VL53L1_WriteMulti(Dev, index, parse_data, 4);
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
    uint8_t temp_data;
	VL53L1_RdByte(Dev, index , &temp_data);
	temp_data = (temp_data & AndData) | OrData;
	return VL53L1_WrByte(Dev, index, temp_data);
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data) {
    return VL53L1_ReadMulti(Dev, index, data, 1);
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data) {
    uint8_t temp_data[2];
	uint8_t* ptr_data = (uint8_t*)data;
	VL53L1_Error error;
	error = VL53L1_ReadMulti(Dev, index, temp_data, 2);
	ptr_data[0] = temp_data[1];
	ptr_data[1] = temp_data[0];
	return error;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data) {
    uint8_t temp_data[4];
	uint8_t* ptr_data = (uint8_t*)data;
	VL53L1_Error error;
	error = VL53L1_ReadMulti(Dev, index, temp_data, 4);
	ptr_data[0] = temp_data[3];
	ptr_data[1] = temp_data[2];
	ptr_data[2] = temp_data[1];
	ptr_data[3] = temp_data[0];
	return error;
}

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
  *ptick_count_ms = 0;
	return status;
}

//#define trace_print(level, ...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, \
//	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

//#define trace_i2c(...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, \
//	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
  *ptimer_freq_hz = 0;
	return status;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms){
	nrf_delay_ms(wait_ms);
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us){
	delay_us(wait_us);
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	uint32_t     polling_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;

	/* wait until value is found, timeout reached on error occurred */
	while ((status == VL53L1_ERROR_NONE)
					&& (polling_time_ms < timeout_ms)
					&& (found == 0)) 
	{
		if (status == VL53L1_ERROR_NONE)
			status = VL53L1_RdByte(pdev, index,	&byte_value);

		if ((byte_value & mask) == value)
			found = 1;

		if (status == VL53L1_ERROR_NONE	&& found == 0 && poll_delay_ms > 0)
			status = VL53L1_WaitMs(pdev, poll_delay_ms);

		/* Update polling time */
		polling_time_ms++;
	}

	if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}




