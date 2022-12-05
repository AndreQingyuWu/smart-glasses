#include <stdint.h>
#include <string.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "glass_ble.h"
#include "app_util_platform.h"

#include "bmi160.h"
#include "vl53l1_user_api.h"
#include "zw101g.h"
#include "motor.h"

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Nordic_Glass"                               /**< Name of device. Will be included in the advertising data. */
#define GLASS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

const float G = 9.80665;   //m/s^2
const float PI = 3.14159;
const uint16_t RANGE_int16 = 32767;  //2^16 / 2

static ble_glass_t                        m_glass;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_GLASS_SERVICE, GLASS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_glass    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void glass_data_handler(ble_glass_t * p_glass, uint8_t * p_data, uint16_t length)
{
  NRF_LOG_PRINTF("recv ble length: %d\n", length);
    for (uint32_t i = 0; i < length; i++)
    {
      NRF_LOG_PRINTF("%d", p_data[i]);
    }
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_glass_init_t glass_init;
    
    memset(&glass_init, 0, sizeof(glass_init));

    glass_init.data_handler = glass_data_handler;
    
    err_code = ble_glass_init(&m_glass, &glass_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice 
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_glass_on_ble_evt(&m_glass, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);    
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
        
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for placing the application in low power state while waiting for events.
 */


/**@brief Application main function.
 */
int main(void) {
    uint32_t err_code;
    uint8_t acc_len = 38;  // sensor frame req  ~ 1024 / average(acc frame size
                          // in header mode) - gyr_len
    uint8_t gyr_len = 38;  // sensor frame req  ~ 1024 / average(gyr frame size
                          // in header mode) - acc_len
    uint32_t sensor_time;
    uint8_t skipped_frame_count;
    struct bmi160_sensor_data acc_data[acc_len];
    struct bmi160_sensor_data gyr_data[gyr_len];
    uint8_t index;

    int16_t acc_x = 0;
    int16_t acc_y = 0;
    int16_t acc_z = 0;
    int16_t gyr_x = 0;
    int16_t gyr_y = 0;
    int16_t gyr_z = 0;
    float r_acc_x = 0;
    float r_acc_y = 0;
    float r_acc_z = 0;
    float angle_acc_x = 0;
    float angle_acc_y = 0;
    float angle_acc_z = 0;
    float w_gyr_x = 0;
    float w_gyr_y = 0;
    float w_gyr_z = 0;
    float angle_cur_x = 0;
    float angle_cur_y = 0;
    float angle_cur_z = 0;
    float angle_pre_x = 0;
    float angle_pre_y = 0;
    float angle_pre_z = 0;
		float angle_filter_x = 0;
    float angle_filter_y = 0;
    float angle_filter_z = 0;
    int16_t angle_out_x = 0;
    int16_t angle_out_y = 0;
    int16_t angle_out_z = 0;
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    // BMI160 use example
    NRF_LOG_PRINTF("ble init\n");
    bmi160_nrf5_init(BMI160_ACCEL_ODR_25HZ, BMI160_GYRO_ODR_25HZ);
    bmi160_fifo_config(1024);
    while (true) {
	  nrf_delay_ms(380);
      if(m_conn_handle == BLE_CONN_HANDLE_INVALID) continue;
      
      index = 0; 
      acc_len = 38;
      gyr_len = 38;
      bmi160_read_fifo(acc_data, &acc_len, gyr_data, &gyr_len, &sensor_time,
                      &skipped_frame_count);
      uint16_t avail_frame = 0;
      if(MIN(acc_len, gyr_len) >= 1) {
        avail_frame = MIN(acc_len, gyr_len) - 1;
      }
      //NRF_LOG_PRINTF("[%d;%d;%d]\n", acc_len, gyr_len, avail_frame);
      for (index = 0; index < avail_frame; index++) {
        acc_x = acc_data[index].x;
        acc_y = acc_data[index].y;
        acc_z = acc_data[index].z;

        gyr_x = gyr_data[index].x;
        gyr_y = gyr_data[index].y;
        gyr_z = gyr_data[index].z;
        //差分滤波
        /*
        //ACC
        r_acc_x = acc_x * 2 * G / RANGE_int16;
        r_acc_y = acc_y * 2 * G / RANGE_int16;
        r_acc_z = acc_z * 2 * G / RANGE_int16;
        //防止除0
        if (r_acc_x == 0) {
            r_acc_x = 2 * G / RANGE_int16;
        }
        if (r_acc_y == 0) {
            r_acc_y = 2 * G / RANGE_int16;
        }
        if (r_acc_z == 0) {
            r_acc_z = 2 * G / RANGE_int16;
        }
        
        angle_acc_x = atan2(r_acc_z, r_acc_y);
        angle_acc_y = atan2(r_acc_x, r_acc_z);
        angle_acc_z = atan2(r_acc_y, r_acc_x);

        //GYR
        w_gyr_x = gyr_x * 125 / RANGE_int16 * PI / 180;
        w_gyr_y = gyr_y * 125 / RANGE_int16 * PI / 180;
        w_gyr_z = gyr_z * 125 / RANGE_int16 * PI / 180;
				
				float K1 = 0.9;
				float K2 = 1 - K1;
        angle_cur_x = K1 * (angle_pre_x + w_gyr_x * 1 / 25) + K2 * angle_acc_x;
        angle_cur_y = K1 * (angle_pre_y + w_gyr_y * 1 / 25) + K2 * angle_acc_y;
        angle_cur_z = K1 * (angle_pre_z + w_gyr_z * 1 / 25) + K2 * angle_acc_z;

        angle_filter_x = angle_pre_x + w_gyr_x * 1 / 25 - angle_acc_x;
        angle_filter_y = angle_pre_y + w_gyr_y * 1 / 25 - angle_acc_y;
        angle_filter_z = angle_pre_z + w_gyr_z * 1 / 25 - angle_acc_z;
        
        angle_pre_x = angle_cur_x;
        angle_pre_y = angle_cur_y;
        angle_pre_z = angle_cur_z;
				
        angle_out_x = 5000 * angle_filter_x;
        angle_out_y = 5000 * angle_filter_y;
        angle_out_z = 5000 * angle_filter_z;
				
        uint8_t ble_buffer[7] = {0}; 
        ble_buffer[0] = angle_out_x >> 8;
        ble_buffer[1] = angle_out_x;
        ble_buffer[2] = angle_out_y >> 8;
        ble_buffer[3] = angle_out_y;
        ble_buffer[4] = angle_out_z >> 8;
        ble_buffer[5] = angle_out_z;
        ble_buffer[6] = 0x00;
        ble_glass_send(&m_glass, ble_buffer, 7);
        */
       //采集数据用于训练
        uint8_t ble_buffer[13] = {0};
        ble_buffer[0] = acc_x >> 8;
        ble_buffer[1] = acc_x;
        ble_buffer[2] = acc_y >> 8;
        ble_buffer[3] = acc_y;
        ble_buffer[4] = acc_z >> 8;
        ble_buffer[5] = acc_z;
        ble_buffer[6] = gyr_x >> 8;
        ble_buffer[7] = gyr_x;
        ble_buffer[8] = gyr_y >> 8;
        ble_buffer[9] = gyr_y;
        ble_buffer[10] = gyr_z >> 8;
        ble_buffer[11] = gyr_z;
        ble_buffer[12] = 0x00;
        ble_glass_send(&m_glass, ble_buffer, 13);
      }
    }
}


/*
  // BMI160 use example
  NRF_LOG_PRINTF("start running\n");
  bmi160_nrf5_init(BMI160_ACCEL_ODR_400HZ, BMI160_GYRO_ODR_400HZ);
  bmi160_fifo_config(1024);
  uint8_t acc_len = 38;  // sensor frame req  ~ 1024 / average(acc frame size
                         // in header mode) - gyr_len
  uint8_t gyr_len = 38;  // sensor frame req  ~ 1024 / average(gyr frame size
                         // in header mode) - acc_len
  uint32_t sensor_time;
  uint8_t skipped_frame_count;
  struct bmi160_sensor_data acc_data[acc_len];
  struct bmi160_sensor_data gyr_data[gyr_len];
  uint8_t index;
  while (true) {
    nrf_delay_ms(100);
    index = 0;
    acc_len = 38;
    gyr_len = 38;
    bmi160_read_fifo(acc_data, &acc_len, gyr_data, &gyr_len, &sensor_time,
                     &skipped_frame_count);
    NRF_LOG_PRINTF("acc len:%d\tgyr len:%d\n", acc_len, gyr_len);
    for (index = 0; index < MIN(acc_len - 1, gyr_len - 1); index++) {
      // NRF_LOG_PRINTF("GYR_X:%d\t\tGYR_Y:%d\t\tGYR_Z:%d\t\t\n",
      // gyr_data[index].x, gyr_data[index].y, gyr_data[index].z);
      NRF_LOG_PRINTF("ACC_X:%d\t\tACC_Y:%d\t\tACC_Z:%d\t\t\n",
                     acc_data[index].x, acc_data[index].y, acc_data[index].z);
    }
    NRF_LOG_PRINTF("SENSOR TIME DATA : %u\n", sensor_time);
    NRF_LOG_PRINTF("SKIPPED FRAME COUNT : %d\n", skipped_frame_count);
  }
  */
	
  /*
  //VL53L1X use example
  NRF_LOG_PRINTF("start running\n");
  vl53l1_nrf5_init();
  uint16_t val;
	while(true) {
    nrf_delay_ms(1000);
    val = vl53l1_getvalue();
    NRF_LOG_PRINTF("MM: %d\n", val);
  }
  */
