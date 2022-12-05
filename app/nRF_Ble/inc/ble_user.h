/******************************************************************************

      Copyright (C), 2001-2011, DCN Co., Ltd.

 ******************************************************************************
  File Name     : ble_user.h
  Version       : Initial Draft
  Author        : x
  Created       : 2019/12/16
  Last Modified :
  Description   : ble  user
  Function List :
  History       :
  1.Date        : 2019/12/16
    Author      : x
    Modification: Created file

******************************************************************************/
#ifndef BLE_NUS_H__
#define BLE_NUS_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_UUID_USER_SERVICE  0x0001                      /**< The UUID of the Nordic UART Service. */
#define BLE_USER_MAX_DATA_LEN (GATT_MTU_SIZE_DEFAULT - 3)  /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */


typedef struct  ble_user_s   ble_user_t;
typedef void   (*ble_user_data_handler_t) (ble_user_t * p_nus, uint8_t * p_data, uint16_t length);

typedef struct
{
    ble_user_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_user_init_t;


struct ble_user_s
{
    uint8_t                  uuid_type;               /**< UUID type for Nordic UART Service Base UUID. */
    uint16_t                 service_handle;          /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t tx_handles;              /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t rx_handles;              /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
    uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
    ble_user_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
};


void      ble_user_on_ble_evt(ble_user_t * p_nus, ble_evt_t * p_ble_evt);
uint32_t  ble_user_init(ble_user_t * p_nus  , const ble_user_init_t * p_nus_init);
uint32_t  ble_user_string_send(ble_user_t * p_nus, uint8_t * p_string, uint16_t length);

#endif 


