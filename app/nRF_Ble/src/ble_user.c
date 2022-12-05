/******************************************************************************

  Copyright (C), 2001-2011, DCN Co., Ltd.

 ******************************************************************************
  File Name     : ble_user.c
  Version       : Initial Draft
  Author        : x
  Created       : 2019/12/16
  Last Modified :
  Description   : ble  user  function.h
  Function List :
  History       :
  1.Date        : 2019/12/16
    Author      : x
    Modification: Created file

******************************************************************************/

#include "ble_user.h"
#include "sdk_common.h"
#include "app_error.h"
#include "ble_srv_common.h"

#define  BLE_USER_MAX_TX_CHAR_LEN        (20)
#define  BLE_USER_MAX_RX_CHAR_LEN        (20)
#define  BLE_UUID_USER                   0x0001
#define  BLE_UUID_USER_TX_CHARACTERISTIC 0x0002                      /**< The UUID of the TX Characteristic. */
#define  BLE_UUID_USER_RX_CHARACTERISTIC 0x0003                      /**< The UUID of the RX Characteristic. */
#define  USER_BASE_UUID   {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */


static  ble_user_t  g_ble_user;

/*****************************************************************************
 Prototype    : GetbleService
 Description  : 返回当前的用户服务信息
 Input        : None
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/16
    Author       : x
    Modification : Created function

*****************************************************************************/
ble_user_t*  GetbleService()
{
   return    &g_ble_user;
}
/*****************************************************************************
 Prototype    : on_connect
 Description  : 连接
 Input        : ble_nus_t * p_nus      
                ble_evt_t * p_ble_evt  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/16
    Author       : x
    Modification : Created function

*****************************************************************************/
static void   on_connect(ble_user_t * p_user, ble_evt_t * p_ble_evt)
{
    if(p_user && p_ble_evt)
    {
       p_user->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    }    
}
/*****************************************************************************
 Prototype    : on_disconnect
 Description  : 蓝牙断开
 Input        : ble_nus_t * p_nus      
                ble_evt_t * p_ble_evt  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/16
    Author       : x
    Modification : Created function

*****************************************************************************/
static  void   on_disconnect(ble_user_t * p_user, ble_evt_t * p_ble_evt)
{
    if(p_user&&p_ble_evt)
    {
       p_user->conn_handle = BLE_CONN_HANDLE_INVALID;
    }  
}
/*****************************************************************************
 Prototype    : on_write
 Description  : 蓝牙接收处理
 Input        : ble_nus_t * p_nus      
                ble_evt_t * p_ble_evt  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/16
    Author       : x
    Modification : Created function

*****************************************************************************/
static    void    on_write(ble_user_t * p_nus, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_nus->rx_handles.cccd_handle)&&(p_evt_write->len == 2))
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_nus->is_notification_enabled = true;
        }
        else
        {
            p_nus->is_notification_enabled = false;
        }
    }
    else if ((p_evt_write->handle == p_nus->tx_handles.value_handle)&&(p_nus->data_handler != NULL))
    {
        p_nus->data_handler(p_nus, p_evt_write->data, p_evt_write->len);
    }
    
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}
/*****************************************************************************
 Prototype    : ble_user_on_ble_evt
 Description  : 蓝牙派发回调
 Input        : ble_nus_t * p_nus      
                ble_evt_t * p_ble_evt  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/16
    Author       : x
    Modification : Created function

*****************************************************************************/
void    ble_user_on_ble_evt(ble_user_t * p_nus, ble_evt_t * p_ble_evt)
{
    if ((p_nus == NULL) || (p_ble_evt == NULL))
    { 
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_nus, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_nus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_nus, p_ble_evt);
            break;

        default:
            break;
    }       

}
/*****************************************************************************
 Prototype    : ble_gatt_tx_charater_add
 Description  : 发送属性设置
 Input        : ble_user_t * p_nus                   
                const ble_user_init_t * p_user_init  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/16
    Author       : x
    Modification : Created function

*****************************************************************************/
static   uint8_t   ble_gatt_tx_charater_add(ble_user_t * p_nus, const ble_user_init_t * p_user_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(char_md));
    memset(&attr_md, 0, sizeof(attr_md));
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_nus->uuid_type;
    ble_uuid.uuid = BLE_UUID_USER_TX_CHARACTERISTIC;


    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_USER_MAX_TX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_nus->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_nus->tx_handles);       

}
/*****************************************************************************
 Prototype    : ble_gatt_rx_charater_add
 Description  : 接收属性信设置
 Input        : ble_user_t * p_nus                   
                const ble_user_init_t * p_user_init  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/16
    Author       : x
    Modification : Created function

*****************************************************************************/
static   uint8_t   ble_gatt_rx_charater_add(ble_user_t * p_nus, const ble_user_init_t * p_user_init)
{
    ble_gatts_char_md_t  char_md;
    ble_gatts_attr_md_t  cccd_md;
    ble_uuid_t           ble_uuid;
    ble_gatts_attr_md_t  attr_md;
    ble_gatts_attr_t     attr_char_value;


    memset(&cccd_md, 0, sizeof(cccd_md));
    memset(&char_md, 0, sizeof(char_md));
    memset(&attr_md, 0, sizeof(attr_md));
    memset(&attr_char_value, 0, sizeof(attr_char_value));


    /*设置蓝牙的连接属性*/
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;


    
    ble_uuid.type = p_nus->uuid_type;
    ble_uuid.uuid = BLE_UUID_USER_RX_CHARACTERISTIC;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;
    
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_USER_MAX_RX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_nus->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_nus->rx_handles);    


}
/*****************************************************************************
 Prototype    : ble_user_init
 Description  : 用户服务初始化
 Input        : ble_nus_t * p_nus                  
                const ble_nus_init_t * p_nus_init  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/16
    Author       : x
    Modification : Created function

*****************************************************************************/
uint32_t  ble_user_init(ble_user_t * p_user , const ble_user_init_t * p_user_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;

    
    ble_uuid128_t nus_base_uuid = USER_BASE_UUID;
    
   
    if(p_user == NULL  || p_user_init  == NULL)
    {
        return   NRF_ERROR_INVALID_DATA;
    }

    p_user->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_user->data_handler            = p_user_init->data_handler;
    p_user->is_notification_enabled = false;

    /*添加服务基础值信息，添加服务类型*/
    err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &p_user->uuid_type);
    APP_ERROR_CHECK(err_code);


   
    ble_uuid.type = p_user->uuid_type;
    ble_uuid.uuid = BLE_UUID_USER;

    
    /*添加服务信息获取服务句柄*/
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_user->service_handle);
    APP_ERROR_CHECK(err_code);

    
    /*添加接收属性值信息*/
    err_code = ble_gatt_rx_charater_add(p_user, p_user_init);
    APP_ERROR_CHECK(err_code);

    
    /*添加发送属性信息*/
    err_code = ble_gatt_tx_charater_add(p_user, p_user_init);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;   

}
/*****************************************************************************
 Prototype    : ble_user_string_send
 Description  : 蓝牙服务发送函数
 Input        : ble_nus_t * p_nus   
                uint8_t * p_string  
                uint16_t length     
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        
  1.Date         : 2019/12/16
    Author       : x
    Modification : Created function

*****************************************************************************/
uint32_t  ble_user_string_send(ble_user_t * p_nus, uint8_t * p_string, uint16_t length)
{


}
