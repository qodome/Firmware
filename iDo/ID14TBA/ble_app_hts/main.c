/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_hts_main main.c
 * @{
 * @ingroup ble_sdk_app_hts
 * @brief Health Thermometer Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Health Thermometer service
 * It also includes the sample code for Battery and Device Information services.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "ble_ido.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "app_scheduler.h"
#include "boards.h"
#include "ble_sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "app_gpiote.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "ble_debug_assert_handler.h"
#include "app_trace.h"
#include "nrf_sdm.h"
#include "app_util_platform.h"
#include "spi_master.h"
#include "nrf_delay.h"
#include "UTCtime_convert.h"
#include "ble_register_rw.h"
#include "temp_service.h"
#include "temp_date_time.h"
#include "ble_memdump.h"
#include "temp_state.h"
#include "cmd_buffer.h"
#include "ble_radio_notification.h"
#include "recorder.h"
#include "flash_helper.h"
#include "persistent.h"
#include "battery.h"
#include "pstorage.h"
#include "ble_dfu.h"
#include "dfu_app_handler.h"

//lint -e553
#ifdef SVCALL_AS_NORMAL_FUNCTION
#include "ser_phy_debug_app.h"
#endif

#define FIRMWARE_VERSION					"1.1.0(00)"
#define SOFTWARE_VERSION					"0.0.0"

#define IS_SRVC_CHANGED_CHARACT_PRESENT      0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define MANUFACTURER_NAME                    "Qodome Co., Ltd."                      /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                            "ID14TBA"                            /**< Model number. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                     MSEC_TO_UNITS(1285, UNIT_0_625_MS)       /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           0                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                 9                                          /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE              9                                          /**< Size of timer operation queues. */

// Android parameter
#define PERIPHERAL_AND_MIN_CONN_INTERVAL            MSEC_TO_UNITS(1000, UNIT_1_25_MS)   /* Minimum acceptable connection interval. */
#define PERIPHERAL_AND_MAX_CONN_INTERVAL            MSEC_TO_UNITS(1500, UNIT_1_25_MS)   /* Maximum acceptable connection interval. */
#define PERIPHERAL_AND_SLAVE_LATENCY                1                                	/* Slave latency. */
#define PERIPHERAL_AND_CONN_SUP_TIMEOUT             MSEC_TO_UNITS(8000, UNIT_10_MS)     /* Connection supervisory timeout. */
// iOS parameter
#define PERIPHERAL_IOS_MIN_CONN_INTERVAL            MSEC_TO_UNITS(500, UNIT_1_25_MS)    /* Minimum acceptable connection interval. */
#define PERIPHERAL_IOS_MAX_CONN_INTERVAL            MSEC_TO_UNITS(1000, UNIT_1_25_MS)   /* Maximum acceptable connection interval. */
#define PERIPHERAL_IOS_SLAVE_LATENCY                1                                   /* Slave latency. */
#define PERIPHERAL_IOS_CONN_SUP_TIMEOUT             MSEC_TO_UNITS(6000, UNIT_10_MS)     /* Connection supervisory timeout. */
// Parameter update
#define PERIPHERAL_FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define PERIPHERAL_NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER)
#define PERIPHERAL_MAX_CONN_PARAMS_UPDATE_COUNT     2

#define BATTERY_LEVEL_MEAS_INTERVAL          		APP_TIMER_TICKS(480000, APP_TIMER_PRESCALER)	// 480 seconds
#define WATCHDOG_INTERVAL          					APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define ADT_MONITOR_INTERVAL						APP_TIMER_TICKS(300000, APP_TIMER_PRESCALER)

#define APP_GPIOTE_MAX_USERS                 1                                          /**< Maximum number of users of the GPIOTE handler. */

#define SEC_PARAM_TIMEOUT                    30                                         /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_gap_adv_params_t                  m_adv_params;                              /**< Parameters to be passed to the stack when starting advertising. */
ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */
static ble_hts_t                             m_hts;                                     /**< Structure used to identify the health thermometer service. */
//static ble_reg_t                             m_reg;                                     /**< Structure used to identify the health thermometer service. */
static ble_time_t                            m_time;                                     /**< Structure used to identify the health thermometer service. */
static ble_dfu_t							 m_dfu;

static app_timer_id_t						m_battery_timer_id;
static app_timer_id_t						m_watchdog_timer_id;
static app_timer_id_t						m_adtmonitor_timer_id;
static dm_application_instance_t      		m_app_handle;
uint8_t advertise_temp_flag = 0;

#ifdef DEBUG_STATS
uint32_t p_ticks_max = 0;
uint32_t full_power_on_seconds = 0;
uint32_t abnormal_counts = 0;
static uint32_t * p_ticks_old;
static uint32_t * p_ticks_now;
static uint32_t * p_ticks_diff;
#endif

// YOUR_JOB: Modify these according to requirements (e.g. if other event types are to pass through
//           the scheduler).
#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE,\
                                          BLE_STACK_HANDLER_SCHED_EVT_SIZE)          /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{

    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Uncomment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
#if SVCALL_AS_NORMAL_FUNCTION
    for(;;);
#else
    NVIC_SystemReset();
#endif
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level = 0;

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

uint16_t spi_read (uint8_t addr)
{
	uint8_t temp_tx_buffer[3] = {0, 0, 0};
	uint8_t temp_rx_buffer[3] = {0, 0, 0};
    int16_t ret = 0;

    if (addr > 0x07) {
      return ret;
    }

    temp_tx_buffer[0] = ((addr & 0x07) << 3) | 0x40;
    if (addr == 0x02 || addr >= 0x04) {
      APP_ERROR_CHECK(spi_master_send_recv(SPI_MASTER_0, temp_tx_buffer, 3,  temp_rx_buffer, 3));
      ret = ((uint16_t)temp_rx_buffer[1] << 8) | temp_rx_buffer[2];
    } else {
      APP_ERROR_CHECK(spi_master_send_recv(SPI_MASTER_0, temp_tx_buffer, 2,  temp_rx_buffer, 2));
      ret = (uint16_t)temp_rx_buffer[1];
    }

    return ret;
}

void spi_write (uint8_t addr, uint16_t cmd)
{
	uint8_t temp_tx_buffer[3] = {0, 0, 0};
	uint8_t temp_rx_buffer[3] = {0, 0, 0};

    temp_tx_buffer[0] = ((addr & 0x07) << 3) & ~(0x40);
    if (addr == 0x01 || addr == 0x05) {
      temp_tx_buffer[1] =(uint8_t)(cmd & 0xFF);
      APP_ERROR_CHECK(spi_master_send_recv(SPI_MASTER_0, temp_tx_buffer, 2,  temp_rx_buffer, 2));

    } else  if (addr == 0x04 || addr == 0x06 || addr == 0x07){
      temp_tx_buffer[1] = (uint8_t)((cmd >> 8) & 0xFF);
      temp_tx_buffer[2] = (uint8_t)(cmd & 0xFF);
      APP_ERROR_CHECK(spi_master_send_recv(SPI_MASTER_0, temp_tx_buffer, 3,  temp_rx_buffer, 3));
    }
}

// Battery measurement shall be synchronized to radio off event
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_request_measure();
}

// Battery measurement shall be synchronized to radio off event
static void watchdog_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    NRF_WDT->RR[0] = 0x6E524635;  //Reload watchdog register 0
}

static void adt_monitor_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	static uint8_t timeout_cnt = 0;

	timeout_cnt++;
	if (timeout_cnt >= 12) {
		timeout_cnt = 0;
		if (((uint8_t)spi_read(0x01) & 0x60) == 0x00) {
			spi_write(0x01, 0x60);
#ifdef DEBUG_STATS
			abnormal_counts++;
#endif
		}
	}
}

static void timers_init(void)
{
	// Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create battery timer.
    APP_ERROR_CHECK(app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler));

    // Create&&start watchdog timer.
    APP_ERROR_CHECK(app_timer_create(&m_watchdog_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                watchdog_timeout_handler));
    APP_ERROR_CHECK(app_timer_start(m_watchdog_timer_id, WATCHDOG_INTERVAL, NULL));

    // Create&&start ADT7320 monitor timer.
    APP_ERROR_CHECK(app_timer_create(&m_adtmonitor_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                adt_monitor_timeout_handler));
    APP_ERROR_CHECK(app_timer_start(m_adtmonitor_timer_id, ADT_MONITOR_INTERVAL, NULL));
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    uint8_t					dev_name[21] = {0};

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    persistent_get_dev_name(dev_name);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)dev_name,
                                          strlen((char *)dev_name));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = PERIPHERAL_AND_MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = PERIPHERAL_AND_MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = PERIPHERAL_AND_SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = PERIPHERAL_AND_CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_default(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t rspdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t adv_uuids[] =
    {
        {BLE_UUID_HEALTH_THERMOMETER_SERVICE, BLE_UUID_TYPE_BLE},
    };

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    memset(&rspdata, 0, sizeof(rspdata));
    rspdata.name_type               = BLE_ADVDATA_FULL_NAME;

    err_code = ble_advdata_set(&advdata, &rspdata);
    APP_ERROR_CHECK(err_code);
}

static void advertising_init(void)
{
	advertising_default();

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                           // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}

// Set service data into advertising packet when disconnected
// due to radio signal
static void advertising_temp(int16_t temp)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t rspdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    ble_advdata_service_data_t temp_service_data;
    int32_t temp_data;
    int32_t sign = -4;

    ble_uuid_t adv_uuids[] =
    {
        {BLE_UUID_HEALTH_THERMOMETER_SERVICE, BLE_UUID_TYPE_BLE},
    };

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    advdata.service_data_count = 1;
    temp_service_data.service_uuid = BLE_UUID_HEALTH_THERMOMETER_SERVICE;
    temp_service_data.data.size = 4;
    if (temp & 0x8000) {
    	temp_data = (int32_t)temp | 0xFFFF0000;
    } else {
    	temp_data = (int32_t)temp;
    }
    temp_data = (sign << 24) | ((temp_data * 625) & 0xFFFFFF);
    temp_service_data.data.p_data = (uint8_t *)&temp_data;
    advdata.p_service_data_array = &temp_service_data;

    memset(&rspdata, 0, sizeof(rspdata));
    rspdata.name_type               = BLE_ADVDATA_FULL_NAME;

    err_code = ble_advdata_set(&advdata, &rspdata);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Health Thermometer Service events.
 *
 * @details This function will be called for all Health Thermometer Service events which are passed
 *          to the application.
 *
 * @param[in]   p_hts   Health Thermometer Service structure.
 * @param[in]   p_evt   Event received from the Health Thermometer Service.
 */
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t *p_evt)
{
	switch (p_evt->evt_type)
	{
	case BLE_HTS_EVT_INDICATION_ENABLED:
		temp_tm_start();
		break;

	case BLE_HTS_EVT_INDICATION_DISABLED:
		temp_tm_stop();
		break;

	case BLE_HTS_EVT_INDICATION_CONFIRMED:
		temp_measurement_confirm();
		break;

	case BLE_HTS_EVT_NOTIFICATION_ENABLED:
		temp_it_start();
		break;

	case BLE_HTS_EVT_NOTIFICATION_DISABLED:
		temp_it_stop();
		break;

	default:
		// No implementation needed.
		break;
	}
}

// Battery service event handler
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t *p_evt)
{
	switch (p_evt->evt_type) {
	case BLE_BAS_EVT_NOTIFICATION_ENABLED:
		// Start battery timer
		APP_ERROR_CHECK(app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL));
		break;

	case BLE_BAS_EVT_NOTIFICATION_DISABLED:
		APP_ERROR_CHECK(app_timer_stop(m_battery_timer_id));
		break;

	default:
		// No implementation needed.
		break;
	}
}

static uint8_t bin_to_ascii(uint8_t b)
{
	uint8_t a = 0;

	if (b > 15) {
		return ' ';
	}
	if (b < 10) {
		a = '0' + b;
	} else {
		a = 'a' + (b - 10);
	}
	return a;
}

static void reset_prepare(void)
{
	uint32_t err_code;

	temp_tm_stop();
	temp_it_stop();
	spi_write(0x01, 0x60);

	if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
		// Disconnect from peer.
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
	}
	err_code = ble_conn_params_stop();
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Health Thermometer, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_hts_init_t   hts_init;
    //ble_reg_init_t   reg_init;
    ble_time_init_t  time_init;
    ble_dis_init_t   dis_init;
	ble_bas_init_t   bas_init;
	ble_dfu_init_t   dfus_init;
    static uint8_t	 sn[32] = {0};
    uint8_t			*p_sn = NULL;
    uint8_t			 idx = 0;

    // Initialize Health Thermometer Service
    memset(&hts_init, 0, sizeof(hts_init));

    hts_init.evt_handler                 = on_hts_evt;

    err_code = ble_hts_init(&m_hts, &hts_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,     MODEL_NUM);
    p_sn = (uint8_t *)NRF_FICR->DEVICEADDR;
    // WARNING: do not use sprintf (code size)
    for (idx = 0; idx < 6; idx++) {
    	sn[idx * 3] = bin_to_ascii((p_sn[idx] >> 4) & 0x0F);
    	sn[idx * 3 + 1] = bin_to_ascii(p_sn[idx] & 0x0F);
    	if (idx == 5) {
    		sn[idx * 3 + 2] = 0;
    	} else {
    		sn[idx * 3 + 2] = ':';
    	}
    }
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char *)sn);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, 	FIRMWARE_VERSION);
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str,		SOFTWARE_VERSION);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // Initialize register  Service.
    //memset(&reg_init, 0, sizeof(reg_init));

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&reg_init.reg_type_attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&reg_init.reg_type_attr_md.write_perm);
    //reg_init.evt_handler          = NULL;
    //err_code = ble_reg_init(&m_reg, &reg_init);
    //APP_ERROR_CHECK(err_code);

    // Initialize date_time  Service.
    memset(&time_init, 0, sizeof(time_init));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&time_init.time_type_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&time_init.time_type_attr_md.write_perm);
    time_init.evt_handler          = NULL;
    err_code = ble_time_init(&m_time, &time_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = on_bas_evt;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    APP_ERROR_CHECK(ble_bas_init(&m_bas, &bas_init));

#ifdef DEBUG_STATS
    APP_ERROR_CHECK(ble_memdump_init());
#endif

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));
    dfus_init.evt_handler = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL; //service_error_handler - Not used as only the switch from app to DFU mode is required and not full dfu service.
    err_code = ble_dfu_init(&m_dfu, &dfus_init);
    APP_ERROR_CHECK(err_code);
    dfu_app_reset_prepare_set(reset_prepare);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;
    
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
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

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
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
    cp_init.first_conn_params_update_delay = PERIPHERAL_FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = PERIPHERAL_NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = PERIPHERAL_MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    cp_init.secondary_min_conn_interval = PERIPHERAL_IOS_MIN_CONN_INTERVAL;
    cp_init.secondary_max_conn_interval = PERIPHERAL_IOS_MAX_CONN_INTERVAL;
    cp_init.secondary_slave_latency     = PERIPHERAL_IOS_SLAVE_LATENCY;
    cp_init.secondary_conn_sup_timeout  = PERIPHERAL_IOS_CONN_SUP_TIMEOUT;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    uint8_t reason = 0;
    static uint8_t dev_name_check[32] = {0};
    uint8_t dev_name_new[32] = {0};
    uint16_t len;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            APP_ERROR_CHECK(sd_ble_gap_device_name_get(dev_name_check, &len));
            battery_request_measure();
            advertising_default();
            advertise_temp_flag = 0;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
        	// If device name get changed, save that in persistent storage
        	APP_ERROR_CHECK(sd_ble_gap_device_name_get(dev_name_new, &len));
        	if (strcmp((char *)dev_name_check, (char *)dev_name_new) != 0) {
        		persistent_set_dev_name(dev_name_new, strlen((char *)dev_name_new));
        	}
        	reason = p_ble_evt->evt.gap_evt.params.disconnected.reason;
        	if (reason != BLE_HCI_CONNECTION_TIMEOUT)
        	{
        		temp_tm_stop();
        		temp_it_stop();
        		advertise_temp_flag = 0;
        	} else {
        		advertise_temp_flag = 1;
        	}
        	m_conn_handle               = BLE_CONN_HANDLE_INVALID;
        	advertising_start();

#ifndef DEBUG_STATS
        	if(memdump_flag_get()) {
        		NVIC_SystemReset();
        	}
#endif

        	APP_ERROR_CHECK(app_timer_stop(m_battery_timer_id));

            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
                // Go to system-off mode (this function will not return; wakeup will cause a reset).
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL)
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

static void check_flash_status(void * p_event_data , uint16_t event_size)
{
	uint32_t event = 0;

	event = *(uint32_t *)p_event_data;
    flash_helper_sys_event(event);
}

/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
	static uint32_t evt = 0;

	evt = sys_evt;

    switch(sys_evt)
    {
    	case NRF_EVT_FLASH_OPERATION_SUCCESS:
    	case NRF_EVT_FLASH_OPERATION_ERROR:
    	    APP_ERROR_CHECK(app_sched_event_put((void *)&evt, sizeof(evt), check_flash_status));
    		break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	ble_memdump_t m_memdump;
	ble_hts_on_ble_evt(&m_hts, p_ble_evt);
	//ble_reg_on_ble_evt(&m_reg, p_ble_evt);
	ble_time_on_ble_evt(&m_time, p_ble_evt);
	ble_bas_on_ble_evt(&m_bas, p_ble_evt);
	on_ble_evt(p_ble_evt);
	ble_conn_params_on_ble_evt(p_ble_evt);
	dm_ble_evt_handler(p_ble_evt);
	ble_dfu_on_ble_evt(&m_dfu, p_ble_evt);

	// Backdoor
	if (memdump_flag_get()) {
		m_memdump = get_ble_memdump();
		ble_memdump_on_ble_evt(&m_memdump, p_ble_evt);
	}
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
	pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for initializing the GPIOTE handler module.

*/
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}

/*
* This function is root source for all temperatures (both measurement and intermediate)
* Intermediate temperature is reported here, measurement is reported by another routine
*/
void iDo_send_indication(ble_hts_meas_t *p) // send_temp_handler
{
	ble_hts_send_tm(&m_hts, p);
}

void iDo_send_notification(ble_hts_meas_t *p)
{

	ble_hts_send_it(&m_hts, p);
}

void iDo_advertise_temp(int16_t temp)
{
	if (advertise_temp_flag == 1) {
		advertising_temp(temp);
	}
}

// Device manager
static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           api_result_t           event_result)
{
    //uint32_t err_code;
    //bool     is_indication_enabled;

    switch(p_event->event_id)
    {
        case DM_EVT_LINK_SECURED:
            // Send a single temperature measurement if indication is enabled.
            // NOTE: For this to work, make sure ble_hts_on_ble_evt() is called before
            //       ble_bondmngr_on_ble_evt() in ble_evt_dispatch().
        	/*
            err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
            APP_ERROR_CHECK(err_code);

            if (is_indication_enabled)
            {
                //temperature_measurement_send();
            }
            */
            break;
        default:
            break;
    }

    return NRF_SUCCESS;
}

/**@brief Function for the Device Manager initialization.
 */
static void device_manager_init(void)
{
    uint32_t                err_code;
    dm_init_param_t         init_data;
    dm_application_param_t  register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    // Clear all bonded centrals if the Bonds Delete button is pushed.
    //init_data.clear_persistent_data = (nrf_gpio_pin_read(BOND_DELETE_ALL_BUTTON_ID) == 0);

    err_code = dm_init(&init_data);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.timeout      = SEC_PARAM_TIMEOUT;
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}

static void wdt_init(void)
{
	NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
	NRF_WDT->CRV = 2*32768;   // 2 seconds timeout period
	NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  //Enable reload register 0
	NRF_WDT->TASKS_START = 1;
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
	wdt_init();
	persistent_init();
    app_trace_init();
    timers_init();
    date_time_init();
    gpiote_init();
    ble_stack_init();
    temp_init(iDo_send_indication, iDo_send_notification, iDo_advertise_temp);//send_temp_handler
    scheduler_init();
    device_manager_init();
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
    temp_state_init();
    cmd_buffer_init();
    recorder_init();
    ble_radio_notification_init(APP_IRQ_PRIORITY_LOW, 0, flash_radio_notification_evt_handler_t);

    // Start execution.
    advertising_start();

    // Enter main loop.
    for (;;)
    {

#ifdef DEBUG_STATS
    	app_timer_cnt_get(p_ticks_old);
#endif

    	app_sched_execute();

#ifdef DEBUG_STATS
    	app_timer_cnt_get(p_ticks_now);
    	app_timer_cnt_diff_compute(*p_ticks_now,*p_ticks_old,p_ticks_diff);
    	if(p_ticks_max < *p_ticks_diff){
    	p_ticks_max = *p_ticks_diff;
    	}
    	full_power_on_seconds += p_ticks_max;
#endif

    	power_manage();


    }
}

/**
 * @}
 */
