#include "glass_ble.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

#define BLE_UUID_GLASS_RX_CHARACTERISTIC 0x0005
#define BLE_UUID_GLASS_TX_CHARACTERISTIC 0x0006

#define BLE_GLASS_MAX_RX_CHAR_LEN        BLE_GLASS_MAX_DATA_LEN    
#define BLE_GLASS_MAX_TX_CHAR_LEN        BLE_GLASS_MAX_DATA_LEN    

#define GLASS_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_glass     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_glass_t * p_glass, ble_evt_t * p_ble_evt)
{
    p_glass->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_glass     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_glass_t * p_glass, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_glass->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S110 SoftDevice.
 *
 * @param[in] p_glass     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_glass_t * p_glass, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (
        (p_evt_write->handle == p_glass->rx_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_glass->is_notification_enabled = true;
        }
        else
        {
            p_glass->is_notification_enabled = false;
        }
    }
    else if (
             (p_evt_write->handle == p_glass->tx_handles.value_handle)
             &&
             (p_glass->data_handler != NULL)
            )
    {
        p_glass->data_handler(p_glass, p_evt_write->data, p_evt_write->len);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}


/**@brief Function for adding RX characteristic.
 *
 * @param[in] p_glass       Nordic UART Service structure.
 * @param[in] p_glass_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_character_add(ble_glass_t * p_glass, const ble_glass_init_t * p_glass_init)
{
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_glass->uuid_type;
    ble_uuid.uuid = BLE_UUID_GLASS_RX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_GLASS_MAX_RX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_glass->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_glass->rx_handles);
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}



static uint32_t tx_character_add(ble_glass_t * p_glass, const ble_glass_init_t * p_glass_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_glass->uuid_type;
    ble_uuid.uuid = BLE_UUID_GLASS_TX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_GLASS_MAX_TX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_glass->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_glass->tx_handles);
}

void ble_glass_on_ble_evt(ble_glass_t * p_glass, ble_evt_t * p_ble_evt)
{
    if ((p_glass == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_glass, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_glass, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_glass, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_glass_init(ble_glass_t * p_glass, const ble_glass_init_t * p_glass_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t glass_base_uuid = GLASS_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_glass);
    VERIFY_PARAM_NOT_NULL(p_glass_init);

    // Initialize the service structure.
    p_glass->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_glass->data_handler            = p_glass_init->data_handler;
    p_glass->is_notification_enabled = false;

    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&glass_base_uuid, &p_glass->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_glass->uuid_type;
    ble_uuid.uuid = BLE_UUID_GLASS_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_glass->service_handle);
    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the RX Characteristic.
    err_code = rx_character_add(p_glass, p_glass_init);
    VERIFY_SUCCESS(err_code);

    // Add the TX Characteristic.
    err_code = tx_character_add(p_glass, p_glass_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


uint32_t ble_glass_send(ble_glass_t * p_glass, uint8_t * p_data, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    VERIFY_PARAM_NOT_NULL(p_glass);

    if ((p_glass->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_glass->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_GLASS_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_glass->rx_handles.value_handle;
    hvx_params.p_data = p_data;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_glass->conn_handle, &hvx_params);
}
