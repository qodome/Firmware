/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

#include "ble_conn_params.h"
#include <stdlib.h>
#include "nordic_common.h"
#include "app_timer.h"
#include "ble_srv_common.h"
#include "app_util.h"


static ble_conn_params_init_t m_conn_params_config;     /**< Configuration as specified by the application. */
static ble_gap_conn_params_t  m_ios_conn_params;  		// iOS connection parameter
static ble_gap_conn_params_t  m_and_conn_params;		// Android connection parameter
static uint8_t                m_update_count;           /**< Number of Connection Parameter Update messages that has currently been sent. */
static uint16_t               m_conn_handle;            /**< Current connection handle. */
static ble_gap_conn_params_t  m_current_conn_params;    /**< Connection parameters received in the most recent Connect event. */
static app_timer_id_t         m_conn_params_timer_id;   /**< Connection parameters timer. */
static app_timer_id_t		  m_fast_conn_params_timer_id;
uint8_t m_conn_param_negotiation_done = 0;

static void fast_conn_params_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    ble_gap_conn_params_t  requested_conn_params;

	if (m_conn_param_negotiation_done == 1) {
		if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
			requested_conn_params.min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS);
			requested_conn_params.max_conn_interval = MSEC_TO_UNITS(150, UNIT_1_25_MS);
			requested_conn_params.slave_latency = 0;
			requested_conn_params.conn_sup_timeout = MSEC_TO_UNITS(2000, UNIT_10_MS);
	        // Parameters are not ok, send connection parameters update request.
	        APP_ERROR_CHECK(sd_ble_gap_conn_param_update(m_conn_handle, &requested_conn_params));
		}
	} else {
		APP_ERROR_CHECK(app_timer_start(m_fast_conn_params_timer_id, 1000, NULL));
	}
}

static void __update_timeout_handler(void * p_event_data , uint16_t event_size)
{
    ble_gap_conn_params_t  requested_conn_params;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        // Check if we have reached the maximum number of attempts
        m_update_count++;
        if (m_update_count <= m_conn_params_config.max_conn_params_update_count) {
            if (m_update_count == 1) {
            	requested_conn_params.min_conn_interval = m_ios_conn_params.min_conn_interval;
            	requested_conn_params.max_conn_interval = m_ios_conn_params.max_conn_interval;
            	requested_conn_params.slave_latency = m_ios_conn_params.slave_latency;
            	requested_conn_params.conn_sup_timeout = m_ios_conn_params.conn_sup_timeout;
            } else {
            	requested_conn_params.min_conn_interval = m_and_conn_params.min_conn_interval;
            	requested_conn_params.max_conn_interval = m_and_conn_params.max_conn_interval;
            	requested_conn_params.slave_latency = m_and_conn_params.slave_latency;
            	requested_conn_params.conn_sup_timeout = m_and_conn_params.conn_sup_timeout;
            }

            // Parameters are not ok, send connection parameters update request.
            APP_ERROR_CHECK(sd_ble_gap_conn_param_update(m_conn_handle, &requested_conn_params));
        }
    }
}

static void update_timeout_handler(void * p_context)
{
    APP_ERROR_CHECK(app_sched_event_put(NULL, 0, __update_timeout_handler));
}

// Enable fast conn_parameter for temperature record read
void ble_conn_enable_fast_read(void)
{
	ble_gap_conn_params_t  requested_conn_params;

	if (m_conn_param_negotiation_done == 1) {
		if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
			requested_conn_params.min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS);
			requested_conn_params.max_conn_interval = MSEC_TO_UNITS(150, UNIT_1_25_MS);
			requested_conn_params.slave_latency = 0;
			requested_conn_params.conn_sup_timeout = MSEC_TO_UNITS(2000, UNIT_10_MS);
			// Parameters are not ok, send connection parameters update request.
			APP_ERROR_CHECK(sd_ble_gap_conn_param_update(m_conn_handle, &requested_conn_params));
		}
	} else {
		APP_ERROR_CHECK(app_timer_start(m_fast_conn_params_timer_id, 1000, NULL));
	}
}

uint32_t ble_conn_params_init(const ble_conn_params_init_t * p_init)
{
    uint32_t err_code;

    m_conn_params_config = *p_init;

    // Fetch the connection params from stack
    err_code = sd_ble_gap_ppcp_get(&m_ios_conn_params);
    if (err_code != NRF_SUCCESS)
    {
    	return err_code;
    }

    m_and_conn_params.min_conn_interval = m_conn_params_config.secondary_min_conn_interval;
    m_and_conn_params.max_conn_interval = m_conn_params_config.secondary_max_conn_interval;
    m_and_conn_params.slave_latency = m_conn_params_config.secondary_slave_latency;
    m_and_conn_params.conn_sup_timeout = m_conn_params_config.secondary_conn_sup_timeout;

    m_conn_handle  = BLE_CONN_HANDLE_INVALID;
    m_update_count = 0;

    err_code = app_timer_create(&m_fast_conn_params_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                fast_conn_params_handler);
    if (err_code != NRF_SUCCESS) {
    	return err_code;
    }

    return app_timer_create(&m_conn_params_timer_id,
                            APP_TIMER_MODE_SINGLE_SHOT,
                            update_timeout_handler);
}


uint32_t ble_conn_params_stop(void)
{
    APP_ERROR_CHECK(app_timer_stop(m_conn_params_timer_id));
    APP_ERROR_CHECK(app_timer_stop(m_fast_conn_params_timer_id));

    return NRF_SUCCESS;
}


static void conn_params_negotiation(void)
{
    if (m_update_count <= 1) {
    	if (m_update_count == 0) {
    		// First connection parameter shall be updated immediately
    		APP_ERROR_CHECK(app_sched_event_put(NULL, 0, __update_timeout_handler));
    	} else {
    		APP_ERROR_CHECK(app_timer_start(m_conn_params_timer_id, m_conn_params_config.next_conn_params_update_delay, NULL));
    	}
    } else {
        m_conn_param_negotiation_done = 1;
    }

	// Notify the application that the procedure has succeeded
	if (m_conn_params_config.evt_handler != NULL) {
		ble_conn_params_evt_t evt;

		evt.evt_type = BLE_CONN_PARAMS_EVT_SUCCEEDED;
		evt.conn_interval = m_current_conn_params.max_conn_interval;
		evt.slave_latency = m_current_conn_params.slave_latency;
		m_conn_params_config.evt_handler(&evt);
	}
}

static void on_connect(ble_evt_t * p_ble_evt)
{
    // Save connection parameters
    m_conn_handle         = p_ble_evt->evt.gap_evt.conn_handle;
    m_current_conn_params = p_ble_evt->evt.gap_evt.params.connected.conn_params;
    m_update_count        = 0;  // Connection parameter negotiation should re-start every connection

    conn_params_negotiation();
}

static void on_disconnect(ble_evt_t * p_ble_evt)
{
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    APP_ERROR_CHECK(app_timer_stop(m_conn_params_timer_id));
    APP_ERROR_CHECK(app_timer_stop(m_fast_conn_params_timer_id));
    m_conn_param_negotiation_done = 0;
    m_update_count = 0;
}

static void on_conn_params_update(ble_evt_t * p_ble_evt)
{
    // Copy the parameters
    m_current_conn_params = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params;

    conn_params_negotiation();
}


void ble_conn_params_on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ble_evt);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            on_conn_params_update(p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}
