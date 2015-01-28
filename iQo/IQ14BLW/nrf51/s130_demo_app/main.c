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
#include "uart.h"
#include "app_timer.h"
#include "nrf_gpio.h"
#include "softdevice_handler.h"
#include "ble_memdump.h"
#include "app_scheduler.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_hts_c.h"
#include "nrf_pwm.h"
#include "ble_led.h"
#include "intermcu_spi.h"
#include "nrf_gpio.h"

/* Addresses of peer peripherals that are expeted to run Heart Rate Service. */
#define NUMBER_OF_PERIPHERALS                   1

/* Services on Peripherals */
#define HEART_RATE_SERVICE                      0x180D
#define HEART_RATE_SERVICE_CHARACTERISTICS      0x2A37              /* HR Measurement service characteristic */
#define HEART_RATE_SERVICE_DESCRIPTOR           0x2902              /* HR Measurement service descriptor */
#define BUFFER_SIZE                             16
#define WRITE_VALUE_ENABLE_NOTIFICATIONS        0x0001              /* Enable notifications command. */
#define WRITE_VALUE_DISABLE_NOTIFICATIONS       0x0000              /* Disable notifications command. */

#define DEVICE_NAME                             "iQo" 

#define APP_ADV_INTERVAL                        MSEC_TO_UNITS(120, UNIT_0_625_MS)  /* The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS              0                                 /* The advertising NEVER timeout */

#define CENTRAL_MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)  /* Minimum acceptable connection interval. */
#define CENTRAL_MAX_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)  /* Maximum acceptable connection interval. */
#define CENTRAL_SLAVE_LATENCY                   0                                 /* Slave latency. */
#define CENTRAL_CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)   /* Connection supervisory timeout (4 seconds). */

// Android parameter
#define PERIPHERAL_AND_MIN_CONN_INTERVAL            MSEC_TO_UNITS(80, UNIT_1_25_MS)   /* Minimum acceptable connection interval. */
#define PERIPHERAL_AND_MAX_CONN_INTERVAL            MSEC_TO_UNITS(100, UNIT_1_25_MS)   /* Maximum acceptable connection interval. */
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
#define PERIPHERAL_MAX_CONN_PARAMS_UPDATE_COUNT     1

#define SCAN_INTERVAL                           MSEC_TO_UNITS(100, UNIT_0_625_MS) /* Scan interval between 2.5ms to 10.24s  (100 ms).*/
#define SCAN_WINDOW                             MSEC_TO_UNITS(80, UNIT_0_625_MS)  /* Scan window between 2.5ms to 10.24s    ( 80 ms). */
#define SCAN_TIMEOUT                            0xFFFF                            /* Scan timeout between 0x0001 and 0xFFFF in seconds, 0x0000 disables timeout. */

// Application Timer
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            8                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         12                                           /**< Size of timer operation queues. */

// Scheduler
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                12                                          /**< Maximum number of events in the scheduler queue. */

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
static void board_configure(void);
void uart_logf(const char *fmt, ...);
static void advertising_start(void);

/*****************************************************************************
* Logging and printing to UART
*****************************************************************************/

/**@brief Disable logging to UART by commenting out this line.*/
#define USE_UART_LOG_INFO   /* Enable to print standard output to UART. */
#define USE_UART_LOG_DEBUG  /* Enable to print standard output to UART. */

/**@brief Macro defined to output log data on the UART or not as user information __PRINT or debug __LOG,
                    based on the USE_UART_LOGGING and USE_UART_PRINTING flag. 
                    If logging/printing is disabled, it will just yield a NOP instruction. 
*/
#ifdef USE_UART_LOG_DEBUG
    #define LOG_DEBUG(F, ...) (uart_logf(F "\r\n", ##__VA_ARGS__))
#else
    #define LOG_DEBUG(F, ...) (void)__NOP()
#endif
#ifdef USE_UART_LOG_INFO
    #define _LOG_INFO(F, ...) (uart_logf(F, ##__VA_ARGS__))
    #define LOG_INFO(F, ...) (uart_logf(F "\r\n", ##__VA_ARGS__))
#else
	#define _LOG_INFO(F, ...) (void)__NOP()
    #define LOG_INFO(F, ...) (void)__NOP()
#endif

#define UUID16_EXTRACT(DST,SRC)                                                                  \
        do                                                                                       \
        {                                                                                        \
            (*(DST)) = (SRC)[1];                                                                 \
            (*(DST)) <<= 8;                                                                      \
            (*(DST)) |= (SRC)[0];                                                                \
        } while(0)

/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct
{
    uint8_t     * p_data;                                         /**< Pointer to data. */
    uint16_t      data_len;                                       /**< Length of data. */
}data_t;

/*****************************************************************************
* Asserts handling
*****************************************************************************/

/**@brief Callback handlers for SoftDevice and application asserts. 
 */
void softdevice_assert_callback(uint32_t pc, uint16_t line_num, const uint8_t *file_name);
void app_assert_callback(uint32_t line_num, const uint8_t *file_name);

#define LED_TIME_STEP_SIZE					10
#define LED_TIME_STEP_COUNT					20
#define LED_TIME_STEP_INTERVAL				APP_TIMER_TICKS(LED_TIME_STEP_SIZE, APP_TIMER_PRESCALER)
//#define LED_STEP_SIZE						2
static app_timer_id_t led_dim_timer_id[4];
uint8_t led_current[4] = {0};
uint8_t led_target[4] = {0};
uint8_t led_turning[4] = {0};
uint8_t led_step_period[4] = {0};
uint8_t led_step_period_remaining[4] = {0};
uint8_t led_step_size[4] = {0};
uint8_t led_step_cnt[4] = {0};
uint16_t led_error_cnt = 0;
static uint8_t led_tag[4] = {0, 1, 2, 3};

static ble_memdump_t                            m_memdump;
static ble_led_t								m_led;
static ble_gap_scan_params_t                    m_scan_param;

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
    ble_hts_c_t             hts_c;
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

void led_set_light(uint8_t idx, uint8_t target)
{
	uint8_t delta;

	// Four LEDs
	if ((idx >= 4) || (led_turning[idx] != 0) || (target == led_current[idx])) {
		return;
	}

	led_target[idx] = target;
	if (target > led_current[idx]) {
		led_turning[idx] = 1;
		delta = target - led_current[idx];
	} else {
		led_turning[idx] = 0xFF;
		delta = led_current[idx] - target;
	}

	if (delta <= LED_TIME_STEP_COUNT) {
		led_step_size[idx] = 1;
		led_step_cnt[idx] = LED_TIME_STEP_COUNT;
		led_step_period[idx] = LED_TIME_STEP_COUNT / delta;
		led_step_period_remaining[idx] = led_step_period[idx];
	} else {
		led_step_size[idx] = delta / LED_TIME_STEP_COUNT;
		led_step_cnt[idx] = LED_TIME_STEP_COUNT;
		led_step_period[idx] = 1;
		led_step_period_remaining[idx] = 1;
	}

	APP_ERROR_CHECK(app_timer_start(led_dim_timer_id[idx], LED_TIME_STEP_INTERVAL, &led_tag[idx]));
}

// Application error handler
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NVIC_SystemReset();
}

/**@brief Function resets buffer that keep data collected from peripheral.
 *
 * @param[in]   peripheral_id   Pointing the buffer of which periferal will be reset.
 *
 * @return
 * @retval      Error code: NRF_SUCCESS in case of success
 *                       or NRF_ERROR_INVALID_PARAM in case of providing peripheral_id from out of range.
*/
static uint8_t buffer_reset(uint16_t peripheral_id)
{
    uint8_t i = 0;

    if (peripheral_id < NUMBER_OF_PERIPHERALS)
    {
        gs_peripheral[peripheral_id].data_buffer.next_entry_index = 0;
        for (i = 0; i < DATA_BUFFER_SIZE; i++)
        {
            gs_peripheral[peripheral_id].data_buffer.value[i] = 0;
        }
        return NRF_SUCCESS;
    }
    return NRF_ERROR_INVALID_PARAM;
}


/**@brief Function counts average value of data collected in buffor for given peripheral.
 *
 * @param[in]   peripheral_id   Pointing peripheral to select buffer that should be used to cout the average .
 *
 * @return
 * @retval      Average value of data collected in buffer. In case of providing peripheral_id from out of range it returns 0.
*/
static uint8_t average_buffer_value(uint16_t peripheral_id)
{
    uint16_t sum        = 0;
    uint8_t  average    = 0;
    uint8_t  i          = 0;

    if (peripheral_id < NUMBER_OF_PERIPHERALS)
    {
        if (gs_peripheral[peripheral_id].data_buffer.next_entry_index > 0)
        {
            for (i = 0; i < gs_peripheral[peripheral_id].data_buffer.next_entry_index; i++)
            {
                sum += (uint8_t) gs_peripheral[peripheral_id].data_buffer.value[i];
            }
            average = (uint8_t) (sum / gs_peripheral[peripheral_id].data_buffer.next_entry_index);
        }
    }
    return average;
}

/**@brief Function adds value to the buffer for given peripheral.
 *
 * @param[in]   peripheral_id   Pointing peripheral to select buffer where the value shoudl be added.
 * @param[in]   value           Value to be placed in the buffer.
 *
 * @retval      Error code: NRF_SUCCESS in case of success
 *                       or NRF_ERROR_INVALID_PARAM in case of providing peripheral_id from out of range.
*/
static uint8_t buffer_add_value(uint16_t peripheral_id, uint8_t value)
{
    uint8_t i = 0;
    
    if (peripheral_id < NUMBER_OF_PERIPHERALS)
    {
        if (gs_peripheral[peripheral_id].data_buffer.next_entry_index >= DATA_BUFFER_SIZE)
        {
            for (i = 0; i < DATA_BUFFER_SIZE - 1; i++)
            {
                gs_peripheral[peripheral_id].data_buffer.value[i] = gs_peripheral[peripheral_id].data_buffer.value[i + 1];
            }
            gs_peripheral[peripheral_id].data_buffer.value[DATA_BUFFER_SIZE - 1] = value;
        }
        else
        {
            gs_peripheral[peripheral_id].data_buffer.value[gs_peripheral[peripheral_id].data_buffer.next_entry_index] = value;
            gs_peripheral[peripheral_id].data_buffer.next_entry_index++;
        }
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }
}


/**@brief BLE related global variables used by functions.
*/
static ble_gatts_char_handles_t gs_own_char_handle;

#define INVALID_DESCRIPTOR_HANDLE 0

/**@brief Function resets structure that keep information about peer central.
*/
static void central_info_reset(void)
{
    memset((void*)&gs_central, 0, sizeof(central_t));
    gs_central.conn_handle          = BLE_CONN_HANDLE_INVALID;
    gs_central.notification_enabled = 0;
    gs_central.cpu_request_done  = 0;
    gs_tx_buffer = TX_BUFFER_READY;
}

/**@brief BLE related global variables used by functions.
*/
#define INVALID_DESCRIPTOR_HANDLE 0
#define ID_NOT_FOUND 0xFFFF

/**@brief Function resets structure that keep information about given peripherals.
 *
 * @param[in]   peripheral_id Peripherial id.
 *
 * @retval      NRF_SUCCESS if success or NRF_ERROR_INVALID_PARAM if peripheral_id out of rage
*/
static uint8_t peripheral_info_reset(uint16_t peripheral_id)
{
    if (peripheral_id < NUMBER_OF_PERIPHERALS)
    {
        memset((void*)&gs_peripheral[peripheral_id], 0, sizeof(peripheral_t));
        gs_peripheral[peripheral_id].conn_handle        = BLE_CONN_HANDLE_INVALID;
        gs_peripheral[peripheral_id].descriptor_handle = INVALID_DESCRIPTOR_HANDLE;

        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }
}

/**@brief Function resets structure that keep information about peer peripherals.
*/
static void peripherals_info_reset(void)
{
    uint16_t peripheral_id = ID_NOT_FOUND;
    
    for (peripheral_id = 0; peripheral_id < NUMBER_OF_PERIPHERALS; peripheral_id++)
    {
        if (peripheral_info_reset(peripheral_id) == NRF_ERROR_INVALID_PARAM )
        {
            LOG_DEBUG("Peripheral information not reset - given invalid peripheral id (%i).", peripheral_id);
        }
    }
}

/**@brief Function provided reference to data structured for given connection handle.
 *
 * @param[in]   conn_handle Connection handle to be checked if it belongs to peer central.
 *
 * @retval      Id of peer peripheral or ID_NOT_FOUND if given connection handle does not belong to any peer peripheral.
*/
static uint16_t peripheral_id_get(uint16_t conn_handle)
{
    uint8_t i = 0;
    
    for (i = 0; i < NUMBER_OF_PERIPHERALS; i++)
    {
        if (conn_handle == gs_peripheral[i].conn_handle)
        {
            return i;
        }
    }
    return ID_NOT_FOUND;
}

// Get available peripheral ID
static uint16_t peripheral_get_available_id(void)
{
    uint8_t i = 0;
    
    for (i = 0; i < NUMBER_OF_PERIPHERALS; i++)
    {
        if (gs_peripheral[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            return i;
        }
    }
    return ID_NOT_FOUND;
}

ble_hts_c_t *peripheral_get_hts_c(uint16_t conn_handle)
{
    uint8_t i = 0;
    
    for (i = 0; i < NUMBER_OF_PERIPHERALS; i++)
    {
        if (gs_peripheral[i].conn_handle == conn_handle)
        {
            return &(gs_peripheral[i].hts_c);
        }
    }
    return NULL;
}

/**@brief Function checks if given connection handle comes from peer central.
 *
 * @param[in]   conn_handle Connection handle to be checked if it belongs to peer central.
 *
 * @retval      true or false
*/
bool is_central(uint16_t conn_handle)
{
    if (conn_handle == gs_central.conn_handle)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*****************************************************************************
* GAP events handling
*****************************************************************************/
#define EVENT_HANDLER_NONBLOCKING 0
//static __attribute__((aligned(4))) uint8_t gs_evt_buf[sizeof(ble_evt_t) + BLE_L2CAP_MTU_DEF];
//static ble_evt_t                       *gsp_ble_evt = (ble_evt_t *) gs_evt_buf;


/**@brief Function that handles BLE connect event for peer Central
 *
 * @param[in]   p_ble_evt        Pointer to buffer filled in with an event.
*/
static void connect_peer_central(ble_evt_t *p_ble_evt)
{
    if (gs_central.conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        LOG_INFO("Central connected."); 
        gs_central.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        gs_advertising_is_running = false;
    }
    else
    {
        LOG_INFO("Central already connected. Unrecognized connection."); 
    }
}

/**@brief Function that handles BLE disconnect event
 *
 * @param[in]   p_ble_evt        Pointer to buffer filled in with an event.
*/
static __INLINE void disconnect_event_handle(ble_evt_t *p_ble_evt)
{
    //uint16_t peripheral_id = ID_NOT_FOUND;

   LOG_DEBUG("BLE_GAP_EVT_DISCONNECTED (0x%x) from connection handle 0x%x.", p_ble_evt->header.evt_id, p_ble_evt->evt.gap_evt.conn_handle);

}

/**@brief Function that handles BLE write event
 *
 * @param[in]   peripheral_id    Id of peripheral for handling this event.
 * @param[in]   p_ble_evt        Pointer to buffer filled in with an event.
*/
static __INLINE void write_event_handle(ble_evt_t *p_ble_evt)
{
    uint16_t write_event_value_handle = 0;
    uint8_t  write_data               = 0;

    write_event_value_handle = p_ble_evt->evt.gatts_evt.params.write.context.value_handle;
    write_data = p_ble_evt->evt.gatts_evt.params.write.data[0];
    /* Verify write event */
    if (write_event_value_handle != gs_own_char_handle.value_handle)
    {
        LOG_DEBUG("BLE_GATTS_EVT_WRITE handle value 0x%x != 0x%x", write_event_value_handle, gs_own_char_handle.value_handle);
        return;
    }
    if ((write_data != WRITE_VALUE_ENABLE_NOTIFICATIONS) && (write_data != WRITE_VALUE_DISABLE_NOTIFICATIONS))
    {
        LOG_INFO("Central sent improper value.");
        LOG_DEBUG("BLE_GATTS_EVT_WRITE data 0x%x out of range {0x01,0x00}", write_data, BLE_GATTS_ATTR_TYPE_DESC);
        return;
    }
    if (write_data == WRITE_VALUE_ENABLE_NOTIFICATIONS)
    {
        ble_gap_conn_params_t new_conn_params = {0};
        
        gs_central.notification_enabled = 1;
        LOG_INFO("Sending notifications enabled.");

        if (!gs_central.cpu_request_done)
        {
            uint32_t error_code = NRF_ERROR_NOT_FOUND;
            
            /* Request change of connection parameters to peer central. */
            new_conn_params.min_conn_interval = CENTRAL_MIN_CONN_INTERVAL;
            new_conn_params.max_conn_interval = CENTRAL_MAX_CONN_INTERVAL;
            new_conn_params.slave_latency     = CENTRAL_SLAVE_LATENCY;
            new_conn_params.conn_sup_timeout  = CENTRAL_CONN_SUP_TIMEOUT;
            if ((error_code = sd_ble_gap_conn_param_update(gs_central.conn_handle, &new_conn_params)) != NRF_SUCCESS)
            {
                LOG_DEBUG("(Central) Updating connection parameters failed - error code = 0x%x", error_code);
            }
            gs_central.cpu_request_done = 1; /* Already requested. */
        }
    }
    else if (write_data == WRITE_VALUE_DISABLE_NOTIFICATIONS)
    {
        gs_central.notification_enabled = 0;
        LOG_INFO("Sending notifications disabled.");
    }
}

// LED dimmer
static void led_dim_timeout_handler(void * p_context)
{
	uint8_t idx, delta;

	idx = *(uint8_t *)p_context;

	led_step_period_remaining[idx]--;
	if (led_step_period_remaining[idx] == 0) {
		if (led_turning[idx] == 1) {
			led_current[idx] += led_step_size[idx];
		} else if (led_turning[idx] == 0xFF) {
			led_current[idx] -= led_step_size[idx];
		}
		nrf_pwm_set_value(idx, led_current[idx]);

		if (led_step_cnt[idx] > 0) {
			led_step_cnt[idx]--;
		} else {
			return;
		}
		if (led_step_cnt[idx] == 0) {
			led_turning[idx] = 0;
			if (led_current[idx] != led_target[idx]) {
				led_error_cnt++;
			}
			return;
		}

		if (led_target[idx] > led_current[idx]) {
			led_turning[idx] = 1;
			delta = led_target[idx] - led_current[idx];
		} else if (led_current[idx] > led_target[idx]) {
			led_turning[idx] = 0xFF;
			delta = led_current[idx] - led_target[idx];
		}

		if (delta <= led_step_cnt[idx]) {
			led_step_size[idx] = 1;
			led_step_period[idx] = led_step_cnt[idx] / delta;
			led_step_period_remaining[idx] = led_step_period[idx];
		} else {
			led_step_size[idx] = delta / led_step_cnt[idx];
			led_step_period[idx] = 1;
			led_step_period_remaining[idx] = 1;
		}
	} else {
		led_step_cnt[idx]--;
	}

	if (led_step_cnt[idx] > 0) {
		APP_ERROR_CHECK(app_timer_start(led_dim_timer_id[idx], LED_TIME_STEP_INTERVAL, &led_tag[idx]));
	} else {
		APP_ERROR_CHECK(app_timer_stop(led_dim_timer_id[idx]));
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
    APP_ERROR_CHECK(ble_memdump_init(&m_memdump));
    APP_ERROR_CHECK(ble_led_init(&m_led));
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
	uint8_t idx;

    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    for (idx = 0; idx < 4; idx++) {
    	APP_ERROR_CHECK(app_timer_create(&(led_dim_timer_id[idx]), APP_TIMER_MODE_SINGLE_SHOT, led_dim_timeout_handler));
    }
}

static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type = p_data[index+1];

        if (field_type == type)
        {
            p_typedata->p_data = &p_data[index+2];
            p_typedata->data_len = field_length-1;
            return NRF_SUCCESS;
        }
        index += field_length+1;
    }
    return NRF_ERROR_NOT_FOUND;
}

static void hts_c_evt_handler(ble_hts_c_t * p_hts_c, ble_hts_c_evt_t * p_hts_c_evt)
{
    uint32_t err_code;

    switch (p_hts_c_evt->evt_type)
    {
        case BLE_HTS_C_EVT_DISCOVERY_COMPLETE:
            // Health Therometer Service discovered. Enable notification of Heart Rate Measurement.
            err_code = ble_hts_c_tm_idct_enable(p_hts_c);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_HTS_C_EVT_TM_INDICATION:
        {
            LOG_INFO("indication received");
            break;
        }
        default:
            break;
    }
}

static void hts_c_init(uint16_t conn_handle)
{
    ble_hts_c_init_t hts_c_init_obj;
    uint16_t peripheral_id = peripheral_id_get(conn_handle);

    hts_c_init_obj.evt_handler = hts_c_evt_handler;

    uint32_t err_code = ble_hts_c_init(conn_handle, 
                                        (peripheral_id == ID_NOT_FOUND) ? NULL : &(gs_peripheral[peripheral_id].hts_c), 
                                        &hts_c_init_obj);
    APP_ERROR_CHECK(err_code);
}

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    data_t adv_data;
    data_t type_data;
    uint32_t err_code = NRF_SUCCESS;
    uint16_t peripheral_id = 0;
    const ble_gap_evt_t   * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
            LOG_INFO("BLE_GAP_EVT_ADV_REPORT");

            // Initialize advertisement report for parsing.
            adv_data.p_data = (uint8_t *)p_gap_evt->params.adv_report.data;
            adv_data.data_len = p_gap_evt->params.adv_report.dlen;

            err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
                                        &adv_data,
                                        &type_data);
            if (err_code != NRF_SUCCESS)
            {
                // Compare short local name in case complete name does not match.
                err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
                                            &adv_data,
                                            &type_data);
            }

            // Verify if short or complete name matches target.
            if (err_code == NRF_SUCCESS)
            {
                uint16_t extracted_uuid;

                // UUIDs found, look for matching UUID
                for (uint32_t u_index = 0; u_index < (type_data.data_len/UUID16_SIZE); u_index++)
                {
                    UUID16_EXTRACT(&extracted_uuid,&type_data.p_data[u_index * UUID16_SIZE]);

                    if(extracted_uuid == TARGET_UUID)
                    {
                        // Stop scanning.
                        err_code = sd_ble_gap_scan_stop();
                        if (err_code != NRF_SUCCESS)
                        {
                            LOG_INFO("sd_ble_gap_scan_stop failed");
                        }
                        
                        // Initiate connection.
                        err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
                                                       &m_scan_param,
                                                       &m_connection_param);

                        if (err_code != NRF_SUCCESS)
                        {
                            LOG_INFO("Connection Request Failed, reason %d", err_code);
                        }
                        break;
                    }
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

                LOG_INFO("Connected to peer peripheral");
                next_id = peripheral_get_available_id();
                if (next_id != ID_NOT_FOUND) {
                    gs_peripheral[next_id].conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                } else {
                    LOG_INFO("Cannot find available slot for peripheral");
                }

                // Initialize collector
                hts_c_init(p_ble_evt->evt.gap_evt.conn_handle);
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if (is_central(p_ble_evt->evt.gap_evt.conn_handle)) {
                // Disconnected with peer central
                central_info_reset();
                if (p_ble_evt->evt.gap_evt.params.disconnected.reason == BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION)
                {
                    LOG_INFO("Central disconnected. Connection terminated remotely.");
                }
                else
                {
                    LOG_INFO("Central disconnected.");
                    if (p_ble_evt->evt.gap_evt.params.disconnected.reason == BLE_HCI_CONNECTION_TIMEOUT)
                    {
                        LOG_INFO("Reason: Timeout");
                    }
                    LOG_DEBUG("Central disconnected (reason 0x%x).", p_ble_evt->evt.gap_evt.params.disconnected.reason);
                }
                advertising_start();
            }
            else
            {
                if ((peripheral_id = peripheral_id_get(p_ble_evt->evt.gap_evt.conn_handle)) == ID_NOT_FOUND ) {
                    LOG_DEBUG("Disconnect from unknown device (0x%x)", p_ble_evt->evt.gap_evt.conn_handle);
                } else {
                    if (peripheral_info_reset(peripheral_id) == NRF_ERROR_INVALID_PARAM) {
                        LOG_DEBUG("Peripheral information not reset - given invalid peripheral id (%i).", peripheral_id);
                    }

                    switch(p_ble_evt->evt.gap_evt.params.disconnected.reason) {
                        case BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION:
                            LOG_DEBUG("(Peripheral %i) Disconnected. Conection terminated remotely.", peripheral_id);
                            break;
                        case BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION:
                            LOG_DEBUG("(Peripheral %i) Disconnected by this device.", peripheral_id);
                            break;
                        case BLE_HCI_CONNECTION_TIMEOUT:
                            LOG_DEBUG("(Peripheral %i) Disconnected due to timeout.", peripheral_id);
                            break;
                        default:
                            LOG_DEBUG("(Peripheral %i) Disconnected (reason 0x%x)", peripheral_id, p_ble_evt->evt.gap_evt.params.disconnected.reason);
                            break;
                    }
                    LOG_INFO("(Peripheral %i) Disconnected.", peripheral_id);
                }
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:

            break;

        case BLE_GATTS_EVT_TIMEOUT:
            if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL)
            {

            }
            break;

        // The following case is the key to CCCD issue
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
    ble_hts_c_on_ble_evt((peripheral_id == ID_NOT_FOUND) ? NULL : &(gs_peripheral[peripheral_id].hts_c), p_ble_evt);
    ble_led_on_ble_evt(&m_led, p_ble_evt);
}

static void on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        default:
            // No implementation needed.
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
    uint32_t err_code = 0;

    LOG_DEBUG("%s: Enabling SoftDevice...", __FUNCTION__);

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = PERIPHERAL_AND_MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = PERIPHERAL_AND_MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = PERIPHERAL_AND_SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = PERIPHERAL_AND_CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);

    APP_ERROR_CHECK(err_code);
}

static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;      // flag, appearance, service
    ble_advdata_t srdata;       // full device name
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    // FIXME: battery service is the place holder for Smart Lamp
    ble_uuid_t adv_uuids[] =
       {
           {BLE_UUID_LED_SERVICE, BLE_UUID_TYPE_BLE},
       };

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

    err_code = ble_advdata_set(&advdata, &srdata);
    APP_ERROR_CHECK(err_code);
}

static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
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

/*
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
*/

static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init();
    APP_ERROR_CHECK(err_code);
}

static void scan_start(void)
{
    uint32_t              err_code;

    // No devices in whitelist, hence non selective performed.
    m_scan_param.active       = 0;            // Active scanning set.
    m_scan_param.selective    = 0;            // Selective scanning not set.
    m_scan_param.interval     = SCAN_INTERVAL;// Scan interval.
    m_scan_param.window       = SCAN_WINDOW;  // Scan window.
    m_scan_param.p_whitelist  = NULL;         // No whitelist provided.
    m_scan_param.timeout      = 0x0000;       // No timeout.

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    APP_ERROR_CHECK(err_code);
}

/*
void intermcu_spi_cb(uint8_t type, uint8_t len, uint8_t *buf)
{
	if (type == 0x01) {
		led_set_light(0, (uint32_t)buf[0]);
		led_set_light(1, (uint32_t)buf[1]);
		led_set_light(2, (uint32_t)buf[2]);
	}
}
*/

/* Initial configuration of peripherals and hardware before the test begins. Calling the main loop. */
int main(void)
{
	nrf_gpio_cfg_input(12, GPIO_PIN_CNF_PULL_Disabled);

    // Initialize peripheral
    board_configure();
	timers_init();
    //intermcu_init(intermcu_spi_cb);
	ble_stack_init();
    scheduler_init();
    services_init();
    gap_params_init();
    advertising_init();
    conn_params_init(); 

    // Initialize central
    db_discovery_init();

    peripherals_info_reset();
    central_info_reset();

    LOG_INFO("#########################");
    LOG_INFO("# S130 Demo application #");
    LOG_INFO("#########################");

    advertising_start();
    scan_start(); 

    nrf_pwm_init(18, 14, 15, 13, PWM_MODE_LED_255);
    nrf_pwm_set_value(0, 0);
    nrf_pwm_set_value(1, 0);
    nrf_pwm_set_value(2, 0);
    nrf_pwm_set_value(3, 0);

    //do_work();
    for (;;) {
        app_sched_execute();
        //power_manage();
    }
}

/**@brief Assert callback handler for SoftDevice asserts. */
void softdevice_assert_callback(uint32_t pc, uint16_t line_num, const uint8_t *file_name)
{
    LOG_DEBUG("softdevice_assert_callback");

    while (1);
}

/**@brief Assert callback handler for application asserts. */
void app_assert_callback(uint32_t line_num, const uint8_t *file_name)
{
     LOG_DEBUG("app_assert_callback");

    while (1);
}

/**@brief Hardware configuration. Specify which board you are using.
 */
static void board_configure(void)
{
#if (BOARD_TYPE == BOARD_PCA10001)
        NRF_GPIO->DIRSET = 0x0000F0FF;
        NRF_GPIO->OUTCLR = 0x0000F0FF;
    #else
        NRF_GPIO->DIRSET = 0xF0FF0000;
        NRF_GPIO->OUTCLR = 0xF0FF0000;
#endif
    
    uart_init(UART_BAUD_19K2);

    NRF_GPIO->PIN_CNF[BUTTON_0] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)
                                | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
                                | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
                                | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);

    NRF_GPIO->PIN_CNF[BUTTON_1] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)
                                | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
                                | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
                                | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);
    
#if (BOARD_TYPE == BOARD_PCA0001)
     LOG_DEBUG("%s: Hardware initiated. Usign PCA10001 (Ev. kit)", __FUNCTION__);
#else
     LOG_DEBUG("%s: Hardware initiated. Usign PCA10005 (Dev. kit)", __FUNCTION__);
#endif
}

/**@brief Logging function, used for formated output on the UART.
 */
void uart_logf(const char *fmt, ...)
{
    uint16_t i = 0;
    uint8_t logf_buf[200];
    va_list args;
    va_start(args, fmt);    
    i = vsnprintf((char*)logf_buf, sizeof(logf_buf) - 1, fmt, args);
    logf_buf[i] = 0x00; /* Make sure its zero terminated */
    uart_write_buf((uint8_t*)logf_buf, i);
    va_end(args);
}
