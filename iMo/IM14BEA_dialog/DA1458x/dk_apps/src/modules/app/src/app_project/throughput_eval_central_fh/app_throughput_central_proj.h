/**
****************************************************************************************
*
* @file app_template_proj.h
*
* @brief Template application header file.
*
* Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
*
* <bluetooth.support@diasemi.com> and contributors.
*
****************************************************************************************
*/

#ifndef APP_TEMPLATE_PROJ_H_
#define APP_TEMPLATE_PROJ_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief 
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwble_config.h"
#include "app_task.h"                  // application task
#include "gapc_task.h"                 // gap functions and messages
#include "gapm_task.h"                 // gap functions and messages
#include "app.h"                       // application definitions
#include "co_error.h"                  // error code definitions
#include "smpc_task.h"                 // error code definitions
 


#if BLE_STREAMDATA_HOST
#include "app_streamdatah.h"                  
#include "app_streamdatah_task.h"             
#endif

enum stream_action
{
    APP_STREAM_STOP = 0,
    APP_STREAM_START
    
};



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

struct app_uart_test_req
{
    uint8_t value;
};
struct app_uart_test_rsp
{
    uint8_t value;
};

struct app_ready_ind
{
	int x;
};

struct app_scan_req
{
	uint8_t x;
};

struct app_ext_scan_cmp_ind ///PP app_scan_rsp
{
	uint8_t x;
};

struct app_ext_adv_report_ind
{
    uint8_t addr_type;
    uint8_t addr[6];
};

struct app_ext_connect_cmd
{
    uint8_t addr_type;
    uint8_t addr[6];
    uint16_t intv;
    
};

struct app_ext_connect_ind
{
    uint8_t state;
    uint8_t addr_type;
    uint8_t conn_handle;
    uint8_t addr[6];
};

struct app_ext_disconnect_cmd
{
    uint8_t conn_handle;
};

struct app_ext_disconnect_ind
{
    uint8_t conn_handle;
};

struct app_ext_transmit_cmd
{//APP_EXT_TRANSMIT_CMD
    uint8_t type;
    uint8_t action; // 1 = start / 0 = stop
    uint8_t conn_handle;

};

struct app_ext_stats_report
{//APP_EXT_STATS_REPORT
    uint8_t type;
    uint8_t conn_handle;
    uint16_t timeperiod;
    uint16_t packets_tx;
    uint16_t bytes_tx;

    uint16_t packets_rx;
    uint16_t bytes_rx;
  
    uint16_t packets_errors;
};
/*
 * DEFINES
 ****************************************************************************************
 */

/****************************************************************************
Define device name. Used in Advertise string
*****************************************************************************/

#define APP_DEVICE_NAME "DA14580 TEMPL"

/**
 * Default Advertising data
 * --------------------------------------------------------------------------------------
 * x02 - Length
 * x01 - Flags
 * x06 - LE General Discoverable Mode + BR/EDR Not Supported
 * --------------------------------------------------------------------------------------
 * x03 - Length
 * x03 - Complete list of 16-bit UUIDs available
 * x09\x18 - Health Thermometer Service UUID
 *   or
 * x00\xFF - Nebulization Service UUID
 * --------------------------------------------------------------------------------------
 */

#define APP_ADV_DATA        "\x07\x01\x03\x18"
#define APP_ADV_DATA_LEN    (0)

/**
 * Default Scan response data
 * --------------------------------------------------------------------------------------
 * x09                             - Length
 * xFF                             - Vendor specific advertising type
 * x00\x60\x52\x57\x2D\x42\x4C\x45 - "RW-BLE"
 * --------------------------------------------------------------------------------------
 */
#define APP_SCNRSP_DATA         "\x02\xFF\x00"
#define APP_SCNRSP_DATA_LENGTH  (0)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles reception of uart test message from external processor application. Responds with APP_EXT_TEST_RSP
 * to confirm that channel is alive. 
 ****************************************************************************************
 */
int app_uart_test_req_handler(ke_msg_id_t const msgid,
                                           const struct app_uart_test_req *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Handles reception of scan command from external processor application. Starts scanning for peripherals
 ****************************************************************************************
 */

int app_ext_scan_cmd_handler(ke_msg_id_t const msgid,
	                           const void *param,
	                           ke_task_id_t const dest_id,
	                           ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Handles reception of connect command from external processor application. Connects to peripherals
 ****************************************************************************************
 */
int app_ext_connect_cmd_handler(ke_msg_id_t const msgid,
                             const struct app_ext_connect_cmd *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);


/**
 ****************************************************************************************
 * @brief Handles reception of disconnect command from external processor application. Disconnects from peripherals
 ****************************************************************************************
 */

int app_ext_disconnect_cmd_handler(ke_msg_id_t const msgid,
                             const struct app_ext_disconnect_cmd *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Handles connection timer expiration. Connection request timedout. Cancel connection procedure.
 ****************************************************************************************
 */

int app_conn_timer_handler(ke_msg_id_t const msgid,
									void *param,
									ke_task_id_t const dest_id,
									ke_task_id_t const src_id);
                                    
/**
 ****************************************************************************************
 * @brief Handles reception of start streaming command from external processor application. Starts/stops transmition proccess
 ****************************************************************************************
 */                                    
                                    
int app_ext_transmit_cmd_handler(ke_msg_id_t const msgid,
                             const struct app_ext_transmit_cmd *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Sends streaming counters to external processor application. 
 ****************************************************************************************
 */

void app_ext_stats_report_send(void);

/**
 ****************************************************************************************
 * @brief Throughput test Function initialise application level buffers.
 ****************************************************************************************
*/

void test_pkt_init (void);

/**
 ****************************************************************************************
 * @brief Generate messages and provide to stream queue.
 *
 * @return void.
 ****************************************************************************************
*/

void test_pkt_gen (void);      

/**
 ****************************************************************************************
 * @brief Send Inquiry (devices discovery) request to GAP task.
 *
 * @return void.
 ****************************************************************************************
 */

void app_start_scanning(void);

/// @} APP

#endif //APP_TEMPLATE_PROJ_H_
