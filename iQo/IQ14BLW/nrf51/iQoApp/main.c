 /** @file
 *
 * @defgroup s130_demo main.c
 * @brief Main file for Heart Rate measurement data collector. Sample Application for nRF51822 evaluation board for S130
 *
 * This file contains the source code for a sample application using S130 as collector of data from
 * up to three peer peripheral devices running Heart Rate Service (a sample application from SDK) for the nRF51822 evaluation board (PCA10001).
 * Average value of each of peer peripheral can be sent to peer central as notifications.
 */

#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "app_assert.h"
#include "board_config.h"
#include "app_timer.h"
#include "nrf_gpio.h"
#include "softdevice_handler.h"
#include "ble_memdump.h"
#include "app_scheduler.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_iqo.h"
#include "ble_iqo_c.h"
#include "ble_led.h"
#include "intermcu_spi.h"
#include "persistent.h"
#include "flash_helper.h"
#include "led_service.h"
#include "ble_radio_notification.h"
#include "app_util_platform.h"

/* Addresses of peer peripherals that are expeted to run Heart Rate Service. */
#define NUMBER_OF_PERIPHERALS                   1
#define IS_SRVC_CHANGED_CHARACT_PRESENT      	0

/* Services on Peripherals */
#define HEART_RATE_SERVICE                      0x180D
#define HEART_RATE_SERVICE_CHARACTERISTICS      0x2A37              /* HR Measurement service characteristic */
#define HEART_RATE_SERVICE_DESCRIPTOR           0x2902              /* HR Measurement service descriptor */
#define BUFFER_SIZE                             16
#define WRITE_VALUE_ENABLE_NOTIFICATIONS        0x0001              /* Enable notifications command. */
#define WRITE_VALUE_DISABLE_NOTIFICATIONS       0x0000              /* Disable notifications command. */

#define APP_ADV_INTERVAL                        MSEC_TO_UNITS(120, UNIT_0_625_MS)  /* The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS              0                                 /* The advertising NEVER timeout */

#define CENTRAL_MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)  /* Minimum acceptable connection interval. */
#define CENTRAL_MAX_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)  /* Maximum acceptable connection interval. */
#define CENTRAL_SLAVE_LATENCY                   0                                 /* Slave latency. */
#define CENTRAL_CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)   /* Connection supervisory timeout (4 seconds). */

// Android parameter
#define PERIPHERAL_AND_MIN_CONN_INTERVAL            MSEC_TO_UNITS(100, UNIT_1_25_MS)   /* Minimum acceptable connection interval. */
#define PERIPHERAL_AND_MAX_CONN_INTERVAL            MSEC_TO_UNITS(150, UNIT_1_25_MS)   /* Maximum acceptable connection interval. */
#define PERIPHERAL_AND_SLAVE_LATENCY                1                                   /* Slave latency. */
#define PERIPHERAL_AND_CONN_SUP_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /* Connection supervisory timeout. */
// iOS parameter
#define PERIPHERAL_IOS_MIN_CONN_INTERVAL            MSEC_TO_UNITS(80, UNIT_1_25_MS)    /* Minimum acceptable connection interval. */
#define PERIPHERAL_IOS_MAX_CONN_INTERVAL            MSEC_TO_UNITS(100, UNIT_1_25_MS)   /* Maximum acceptable connection interval. */
#define PERIPHERAL_IOS_SLAVE_LATENCY                1                                   /* Slave latency. */
#define PERIPHERAL_IOS_CONN_SUP_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /* Connection supervisory timeout. */
// Parameter update
#define PERIPHERAL_FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define PERIPHERAL_NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER)
#define WATCHDOG_KICK_PERIOD          				APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define PERIPHERAL_MAX_CONN_PARAMS_UPDATE_COUNT     1

#define SCAN_INTERVAL                           MSEC_TO_UNITS(100, UNIT_0_625_MS) /* Scan interval between 2.5ms to 10.24s  (100 ms).*/
#define SCAN_WINDOW                             MSEC_TO_UNITS(80, UNIT_0_625_MS)  /* Scan window between 2.5ms to 10.24s    ( 80 ms). */
#define SCAN_TIMEOUT                            0xFFFF                            /* Scan timeout between 0x0001 and 0xFFFF in seconds, 0x0000 disables timeout. */

// Application Timer
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            8                                           /**< Maximum number of simultaneously created timers. */

// Scheduler
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                4                                          /**< Maximum number of events in the scheduler queue. */

#define TARGET_UUID                     0x1809                             /**< Target device name that application is looking for. */
#define UUID16_SIZE                2                                  /**< Size of 16 bit UUID */

#define SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

/**@brief To convert ticks from milliseconds
 * @param[in] time          Number of millseconds that needs to be converted.
 * @param[in] resolution    Units to be converted.
 */
#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))

/**@brief Local function prototypes.
 */
static void advertising_start(void);

#define LOG_DEBUG(F, ...) (void)__NOP()
#define LOG_INFO(F, ...) (void)__NOP()

#define UUID16_EXTRACT(DST,SRC)                                                                  \
        do                                                                                       \
        {                                                                                        \
            (*(DST)) = (SRC)[1];                                                                 \
            (*(DST)) <<= 8;                                                                      \
            (*(DST)) |= (SRC)[0];                                                                \
        } while(0)

static ble_led_t m_led;
static ble_iqo_t m_iqo;
static ble_memdump_t m_memdump;
static ble_gap_scan_params_t m_scan_param;
static app_timer_id_t m_watchdog_timer_id;

uint16_t error_cnt = 0;
uint32_t last_error_code = 0;
uint8_t *last_error_file_name = NULL;
uint8_t flash_write_w_backup_status = 0;
uint8_t flash_write_w_backup_data[24];

uint16_t temp_notify_cnt = 0;
uint16_t acc_notify_cnt = 0;

extern ble_iqo_id_t iqo_tgt_identify;

static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)PERIPHERAL_AND_MIN_CONN_INTERVAL,     // Minimum connection
    (uint16_t)PERIPHERAL_AND_MAX_CONN_INTERVAL,     // Maximum connection
    (uint16_t)PERIPHERAL_AND_SLAVE_LATENCY,         // Slave latency
    (uint16_t)PERIPHERAL_AND_CONN_SUP_TIMEOUT       // Supervision time-out
};

/*****************************************************************************
* Functions and structures related to connection and buffers
*****************************************************************************/
#define DATA_BUFFER_SIZE                   12 /* Size of bufer that collects HTS data from one peripheral. */

#define TX_BUFFER_READY                    1 /* TX buffer empty. */
#define TX_BUFFER_BUSY                     0 /* TX buffer in use. */

static uint8_t gs_tx_buffer = TX_BUFFER_READY;

typedef struct {
    uint8_t next_entry_index;
    uint8_t value[DATA_BUFFER_SIZE];
} data_buffer_t;

typedef struct {
    uint16_t                conn_handle;
    uint16_t                descriptor_handle;
    data_buffer_t           data_buffer;
    ble_db_discovery_t      db_discovery;
    ble_iqo_c_t             iqo_c;
} peripheral_t;

peripheral_t gs_peripheral[NUMBER_OF_PERIPHERALS];

typedef struct
{
    uint16_t conn_handle;
    uint8_t  notification_enabled;
    uint8_t  cpu_request_done;
} central_t;

central_t gs_central;
static bool gs_advertising_is_running = false;

// All flash log operations shall happen in scheduler context
static void app_log_error(void * p_event_data , uint16_t event_size)
{
	uint32_t *ptr;

    ptr = (uint32_t *)p_event_data;
	persistent_record_error((uint8_t)ptr[0], ptr[1]);
}

// Application error handler
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    static uint32_t error_buf[2];

	error_cnt++;
	last_error_code = error_code;
	last_error_file_name = (uint8_t *)p_file_name;

	if ((uint8_t)error_code >= PERSISTENT_ERROR_SYS_MAX) {
		error_buf[0] = PERSISTENT_ERROR_SYS_MAX;
	} else {
		error_buf[0] = (uint8_t)error_code;
	}
	error_buf[1] = (line_num & 0x0000FFFF) | (((uint32_t)p_file_name & 0x0000FFFF) << 16);

	app_sched_event_put((void *)&(error_buf[0]), 8, app_log_error);
}

// SoftDevice
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    persistent_record_error(PERSISTENT_ERROR_DEADBEEF, (uint32_t)line_num);
	for(;;);
}

// Start scan for device
void scan_start(void)
{
    m_scan_param.active = 1;
    m_scan_param.selective = 0;
    m_scan_param.interval = SCAN_INTERVAL;
    m_scan_param.window = SCAN_WINDOW;
    m_scan_param.p_whitelist = NULL;
    m_scan_param.timeout = 0x0000;

    APP_ERROR_CHECK(sd_ble_gap_scan_start(&m_scan_param));
}

#define INVALID_DESCRIPTOR_HANDLE 0
#define ID_NOT_FOUND 0xFFFF

// Reset information about peer central
static void central_info_reset(void)
{
    memset((void*)&gs_central, 0, sizeof(central_t));
    gs_central.conn_handle = BLE_CONN_HANDLE_INVALID;
    gs_central.notification_enabled = 0;
    gs_central.cpu_request_done  = 0;
    gs_tx_buffer = TX_BUFFER_READY;
}

static void peripheral_info_reset(uint16_t peripheral_id)
{
    if (peripheral_id < NUMBER_OF_PERIPHERALS) {
        memset((void*)&(gs_peripheral[peripheral_id]), 0, sizeof(peripheral_t));
        gs_peripheral[peripheral_id].conn_handle = BLE_CONN_HANDLE_INVALID;
        gs_peripheral[peripheral_id].descriptor_handle = INVALID_DESCRIPTOR_HANDLE;
    }
}

/**@brief Function resets structure that keep information about peer peripherals.
*/
static void peripherals_info_reset(void)
{
    uint16_t peripheral_id = ID_NOT_FOUND;

    for (peripheral_id = 0; peripheral_id < NUMBER_OF_PERIPHERALS; peripheral_id++) {
        peripheral_info_reset(peripheral_id);
    }
}

// Query peripheral ID
static uint16_t peripheral_id_get(uint16_t conn_handle)
{
    uint8_t i = 0;

    for (i = 0; i < NUMBER_OF_PERIPHERALS; i++) {
        if (conn_handle == gs_peripheral[i].conn_handle) {
            return i;
        }
    }
    return ID_NOT_FOUND;
}

// Get available peripheral ID
static uint16_t peripheral_get_available_id(void)
{
    for (uint8_t i = 0; i < NUMBER_OF_PERIPHERALS; i++) {
        if (gs_peripheral[i].conn_handle == BLE_CONN_HANDLE_INVALID) {
            return i;
        }
    }
    return ID_NOT_FOUND;
}

ble_iqo_c_t *peripheral_get_iqo_c(uint16_t conn_handle)
{
    for (uint8_t i = 0; i < NUMBER_OF_PERIPHERALS; i++) {
        if (gs_peripheral[i].conn_handle == conn_handle) {
            return &(gs_peripheral[i].iqo_c);
        }
    }
    return NULL;
}

bool is_central(uint16_t conn_handle)
{
    if (conn_handle == gs_central.conn_handle) {
        return true;
    } else {
        return false;
    }
}

/*****************************************************************************
* GAP events handling
*****************************************************************************/

/**@brief Function that handles BLE connect event for peer Central
 *
 * @param[in]   p_ble_evt        Pointer to buffer filled in with an event.
*/
static void connect_peer_central(ble_evt_t *p_ble_evt)
{
    if (gs_central.conn_handle == BLE_CONN_HANDLE_INVALID) {
        gs_central.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        gs_advertising_is_running = false;
    }
}

/*****************************************************************************
* Functions related to service setup, advertisement, discovery, device connection
*****************************************************************************/
/**@brief Function steup own service to be used to notify average values to peer central.
 *
 * Function does not return error code. If any internal call failes it triggers APP_ASSERT.
*/
static void services_init(void)
{
    APP_ERROR_CHECK(ble_iqo_init(&m_iqo));
    APP_ERROR_CHECK(ble_memdump_init(&m_memdump));
    APP_ERROR_CHECK(ble_led_init(&m_led));
}

// Watchdog timeout handler
static void watchdog_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    NRF_WDT->RR[0] = 0x6E524635;  //Reload watchdog register 0
}

// Initialize timer module
static void timers_init(void)
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, false);

    // Create watchdog timer.
    APP_ERROR_CHECK(app_timer_create(&m_watchdog_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                watchdog_timeout_handler));
    APP_ERROR_CHECK(app_timer_start(m_watchdog_timer_id, WATCHDOG_KICK_PERIOD, NULL));
}

static void iqo_c_evt_handler(ble_iqo_c_t * p_iqo_c, ble_iqo_c_evt_t * p_iqo_c_evt)
{
    switch (p_iqo_c_evt->evt_type) {
        case BLE_IQO_C_EVT_DISCOVERY_TEMP_COMPLETE:
            APP_ERROR_CHECK(ble_iqo_c_temp_enable(p_iqo_c));
            break;

        case BLE_IQO_C_EVT_IT_NOTIFY:
        	temp_notify_cnt++;
            break;

        case BLE_IQO_C_EVT_DISCOVERY_ACC_COMPLETE:
            APP_ERROR_CHECK(ble_iqo_c_acc_enable(p_iqo_c));
            break;

        case BLE_IQO_C_EVT_ACC_NOTIFY:
            acc_notify_cnt++;
            break;

        default:
            break;
    }
}

static void iqo_c_init(uint16_t conn_handle)
{
    ble_iqo_c_init_t iqo_c_init_obj;
    uint16_t peripheral_id = peripheral_id_get(conn_handle);

    iqo_c_init_obj.evt_handler = iqo_c_evt_handler;

    uint32_t err_code = ble_iqo_c_init(conn_handle,
                                        (peripheral_id == ID_NOT_FOUND) ? NULL : &(gs_peripheral[peripheral_id].iqo_c),
                                        &iqo_c_init_obj);
    APP_ERROR_CHECK(err_code);
}

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    const ble_gap_evt_t *p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        	// Check scan response with full name
        	if (p_gap_evt->params.adv_report.scan_rsp == 1 && p_gap_evt->params.adv_report.data[1] == 0x09) {
        		if (memcmp(&(p_gap_evt->params.adv_report.data[2]), &(iqo_tgt_identify.id[0]), (uint32_t)(p_gap_evt->params.adv_report.data[0] - 1)) == 0 &&
        			iqo_tgt_identify.id[p_gap_evt->params.adv_report.data[0] - 1] == 0) {
            		// Stop scan
            		APP_ERROR_CHECK(sd_ble_gap_scan_stop());

            		// Initiate connection.
            		APP_ERROR_CHECK(sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
            											&m_scan_param,
            		                                    &m_connection_param));
        		}
        	}
            break;

        case BLE_GAP_EVT_CONNECTED:
            if (p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_PERIPH) {
                // Peer is central, we are peripheral
                connect_peer_central(p_ble_evt);
            } else {
                // Connected to peer peripheral
                uint16_t next_id = 0;

                next_id = peripheral_get_available_id();
                if (next_id != ID_NOT_FOUND) {
                    gs_peripheral[next_id].conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                }

                // Initialize collector
                iqo_c_init(p_ble_evt->evt.gap_evt.conn_handle);
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if (is_central(p_ble_evt->evt.gap_evt.conn_handle)) {
                // Disconnected with peer central
                // Check p_ble_evt->evt.gap_evt.params.disconnected.reason ??
                central_info_reset();
                advertising_start();
            } else {
                // Try reconnect ???
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        	sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0);
        	break;

        default:
            // No implementation needed.
            break;
    }
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    uint16_t peripheral_id;

    on_ble_evt(p_ble_evt);
    peripheral_id = peripheral_id_get(p_ble_evt->evt.common_evt.conn_handle);
    ble_db_discovery_on_ble_evt((peripheral_id == ID_NOT_FOUND) ? NULL : &(gs_peripheral[peripheral_id].db_discovery), p_ble_evt);
	ble_memdump_on_ble_evt(&m_memdump, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_iqo_c_on_ble_evt((peripheral_id == ID_NOT_FOUND) ? NULL : &(gs_peripheral[peripheral_id].iqo_c), p_ble_evt);
    ble_led_on_ble_evt(&m_led, p_ble_evt);
    ble_iqo_on_ble_evt(&m_iqo, p_ble_evt);
}

static void check_flash_status(void * p_event_data , uint16_t event_size)
{
	uint32_t event;

	event = *(uint32_t *)p_event_data;
    flash_helper_sys_event(event);
}

static void on_sys_evt(uint32_t sys_evt)
{
	static uint32_t evt= 0;

	evt = sys_evt;
    switch(sys_evt) {
    	case NRF_EVT_FLASH_OPERATION_SUCCESS:
    	case NRF_EVT_FLASH_OPERATION_ERROR:
    	    APP_ERROR_CHECK(app_sched_event_put((void *)&evt, sizeof(evt), check_flash_status));
    		break;

        default:
            break;
    }
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
    on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);

    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    APP_ERROR_CHECK(sd_ble_enable(&ble_enable_params));

    // Register with the SoftDevice handler module for BLE events.
    APP_ERROR_CHECK(softdevice_ble_evt_handler_set(ble_evt_dispatch));

    // Register with the SoftDevice handler module for BLE events.
    APP_ERROR_CHECK(softdevice_sys_evt_handler_set(sys_evt_dispatch));
}

static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void gap_params_init(void)
{
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    //uint8_t					dev_name[21];

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    /*
    persistent_get_dev_name(dev_name);
    APP_ERROR_CHECK(sd_ble_gap_device_name_set(&sec_mode,
            		(const uint8_t *)dev_name,
            		strlen((char *)dev_name)));
	*/
#define DEVICE_NAME                             "iQo"
    APP_ERROR_CHECK(sd_ble_gap_device_name_set(&sec_mode,
    				(const uint8_t *)DEVICE_NAME,
                    strlen(DEVICE_NAME)));

    APP_ERROR_CHECK(sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN));

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = PERIPHERAL_AND_MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = PERIPHERAL_AND_MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = PERIPHERAL_AND_SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = PERIPHERAL_AND_CONN_SUP_TIMEOUT;

    APP_ERROR_CHECK(sd_ble_gap_ppcp_set(&gap_conn_params));
}

static void advertising_init(void)
{
    ble_advdata_t advdata;      // flag, appearance, service
    ble_advdata_t srdata;       // full device name
    uint8_t flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t adv_uuids[] = {{BLE_UUID_IQO_SERVICE, BLE_UUID_TYPE_BLE}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    // Build and set scan response data
    memset(&srdata, 0, sizeof(srdata));
    srdata.name_type                = BLE_ADVDATA_FULL_NAME;

    APP_ERROR_CHECK(ble_advdata_set(&advdata, &srdata));
}

static void advertising_start(void)
{
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    APP_ERROR_CHECK(sd_ble_gap_adv_start(&adv_params));
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        LOG_INFO("parameter negotiation failed\n");
    } else if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED) {
        LOG_INFO("parameter negotiation success!\n");
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
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

    APP_ERROR_CHECK(ble_conn_params_init(&cp_init));
}

static void db_discovery_init(void)
{
    APP_ERROR_CHECK(ble_db_discovery_init());
}

void intermcu_spi_cb(uint8_t type, uint8_t len, uint8_t *buf)
{
    // FIXME: inter-MCU communication
}

void protect_flash(void)
{
	APP_ERROR_CHECK(sd_flash_protect(0, 0xFF800000));
}

static void wdt_init(void)
{
	NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
	NRF_WDT->CRV = 2*32768;   // 2 seconds timeout period
	NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  //Enable reload register 0
	NRF_WDT->TASKS_START = 1;
}

/* Initial configuration of peripherals and hardware before the test begins. Calling the main loop. */
int main(void)
{
	/////////////////////////////////////////////////////
	//          P r o t e c t    F l a s h             //
	/////////////////////////////////////////////////////
	protect_flash();
	persistent_init();		// Persistent storage shall be initialized before radio is turned on

    ////////////////////////////////////////////////////
	//      I n i t i a l i z e    S y s t e m        //
	////////////////////////////////////////////////////
	wdt_init();
    timers_init();
    intermcu_init(intermcu_spi_cb);
    ble_stack_init();
    scheduler_init();
    ble_radio_notification_init(APP_IRQ_PRIORITY_LOW, 0, flash_radio_notification_evt_handler_t);

    ////////////////////////////////////////////////////
    //      I n i t i a l i z e    S e r v i c e      //
    ////////////////////////////////////////////////////
    // Initialize external RTC
    led_service_init();
    APP_ERROR_CHECK(sd_ble_gap_tx_power_set(4));

    /////////////////////////////////////////////////////
    //  I n i t i a l i z e    B L E   S e r v i c e   //
    /////////////////////////////////////////////////////
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
    db_discovery_init();
    peripherals_info_reset();
    central_info_reset();

    // Finally start advertise
    advertising_start();

    for (;;) {
        app_sched_execute();
    }
}


