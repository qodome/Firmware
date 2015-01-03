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
#include "ble_hci.h"
#include "app_timer.h"
#include "ble_srv_common.h"
#include "app_util.h"


static ble_conn_params_init_t m_conn_params_config;     /**< Configuration as specified by the application. */
static ble_gap_conn_params_t  m_ios_conn_params;  		// iOS connection parameter
static ble_gap_conn_params_t  m_and_conn_params;		// Android connection parameter
static ble_gap_conn_params_t  m_requested_conn_params;  // Requested parameter
static uint8_t                m_update_count;           /**< Number of Connection Parameter Update messages that has currently been sent. */
static uint16_t               m_conn_handle;            /**< Current connection handle. */
static ble_gap_conn_params_t  m_current_conn_params;    /**< Connection parameters received in the most recent Connect event. */
static app_timer_id_t         m_conn_params_timer_id;   /**< Connection parameters timer. */
static app_timer_id_t		  m_fast_conn_params_timer_id;
static uint8_t				  m_enabled_fast_parameter;
uint8_t m_conn_param_negotiation_done = 0;

extern void uart_logf(const char *fmt, ...);

static bool is_conn_params_ok(ble_gap_conn_params_t * p_conn_params, ble_gap_conn_params_t * p_target)
{
    // Check if interval is within the acceptable range.
    // NOTE: Using max_conn_interval in the received event data because this contains
    //       the client's connection interval.
	if ((p_conn_params->max_conn_interval >= p_target->min_conn_interval)
			&& (p_conn_params->max_conn_interval <= p_target->max_conn_interval)) {
		return true;
	} else {
		return false;
	}
}

static void fast_conn_params_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

	if (m_conn_param_negotiation_done == 1) {
		if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
			m_enabled_fast_parameter = 1;
			m_requested_conn_params.min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS);
			m_requested_conn_params.max_conn_interval = MSEC_TO_UNITS(150, UNIT_1_25_MS);
			m_requested_conn_params.slave_latency = 0;
			m_requested_conn_params.conn_sup_timeout = MSEC_TO_UNITS(2000, UNIT_10_MS);
	        // Parameters are not ok, send connection parameters update request.
	        APP_ERROR_CHECK(sd_ble_gap_conn_param_update(m_conn_handle, &m_requested_conn_params));
		}
	} else {
		APP_ERROR_CHECK(app_timer_start(m_fast_conn_params_timer_id, 1000, NULL));
	}
}

static void update_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        // Check if we have reached the maximum number of attempts
        m_update_count++;
        if (m_update_count <= m_conn_params_config.max_conn_params_update_count) {
            uint32_t err_code;

            if (m_update_count == 1) {
            	m_requested_conn_params.min_conn_interval = m_ios_conn_params.min_conn_interval;
            	m_requested_conn_params.max_conn_interval = m_ios_conn_params.max_conn_interval;
            	m_requested_conn_params.slave_latency = m_ios_conn_params.slave_latency;
            	m_requested_conn_params.conn_sup_timeout = m_ios_conn_params.conn_sup_timeout;
            } else {
            	m_requested_conn_params.min_conn_interval = m_and_conn_params.min_conn_interval;
            	m_requested_conn_params.max_conn_interval = m_and_conn_params.max_conn_interval;
            	m_requested_conn_params.slave_latency = m_and_conn_params.slave_latency;
            	m_requested_conn_params.conn_sup_timeout = m_and_conn_params.conn_sup_timeout;
            }

            // Parameters are not ok, send connection parameters update request.
            err_code = sd_ble_gap_conn_param_update(m_conn_handle, &m_requested_conn_params);
            if ((err_code != NRF_SUCCESS) && (m_conn_params_config.error_handler != NULL)) {
                m_conn_params_config.error_handler(err_code);
            }
        } else {
            m_update_count = 0;

            // Notify the application that the procedure has failed
            if (m_conn_params_config.evt_handler != NULL) {
                ble_conn_params_evt_t evt;

                evt.evt_type = BLE_CONN_PARAMS_EVT_FAILED;
                m_conn_params_config.evt_handler(&evt);
            }

            m_conn_param_negotiation_done = 1;
        }
    }
}

// Enable fast conn_parameter for temperature record read
void ble_conn_enable_fast_read(void)
{
	if (m_conn_param_negotiation_done == 1) {
		if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
			m_enabled_fast_parameter = 1;
			m_requested_conn_params.min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS);
			m_requested_conn_params.max_conn_interval = MSEC_TO_UNITS(150, UNIT_1_25_MS);
			m_requested_conn_params.slave_latency = 0;
			m_requested_conn_params.conn_sup_timeout = MSEC_TO_UNITS(2000, UNIT_10_MS);
			// Parameters are not ok, send connection parameters update request.
			APP_ERROR_CHECK(sd_ble_gap_conn_param_update(m_conn_handle, &m_requested_conn_params));
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
    m_enabled_fast_parameter = 0;

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
    return app_timer_stop(m_conn_params_timer_id);
}


static void conn_params_negotiation(void)
{
    uint32_t err_code;
    uint32_t timeout_ticks;

    if (m_update_count <= 1) {
    	if (m_update_count == 0) {
    		// First connection parameter update
    		timeout_ticks = m_conn_params_config.first_conn_params_update_delay;
    	} else {
    		timeout_ticks = m_conn_params_config.next_conn_params_update_delay;
    	}
        err_code = app_timer_start(m_conn_params_timer_id, timeout_ticks, NULL);
        if ((err_code != NRF_SUCCESS) && (m_conn_params_config.error_handler != NULL)) {
            m_conn_params_config.error_handler(err_code);
        }
    }

	// Notify the application that the procedure has succeeded
	if (m_conn_params_config.evt_handler != NULL) {
		ble_conn_params_evt_t evt;

		evt.evt_type = BLE_CONN_PARAMS_EVT_SUCCEEDED;
		evt.conn_interval = m_current_conn_params.max_conn_interval;
		m_conn_params_config.evt_handler(&evt);
	}
}


static void on_connect(ble_evt_t * p_ble_evt)
{
    // Save connection parameters
    m_conn_handle         = p_ble_evt->evt.gap_evt.conn_handle;
    m_current_conn_params = p_ble_evt->evt.gap_evt.params.connected.conn_params;
    m_update_count        = 0;  // Connection parameter negotiation should re-start every connection
    m_enabled_fast_parameter = 0;

    //init_preferred_conn_params();

#ifdef CONN_PARAM_UPDATE_ON_CCCD
    // Check if we shall handle negotiation on connect
    if (m_conn_params_config.start_on_notify_cccd_handle == BLE_GATT_HANDLE_INVALID)
    {
        conn_params_negotiation();
    }
#else
    conn_params_negotiation();
#endif
}

static void on_disconnect(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    m_enabled_fast_parameter = 0;

    // Stop timer if running
    m_update_count = 0; // Connection parameters updates should happen during every connection

    err_code = app_timer_stop(m_conn_params_timer_id);
    if ((err_code != NRF_SUCCESS) && (m_conn_params_config.error_handler != NULL))
    {
        m_conn_params_config.error_handler(err_code);
    }

    app_timer_stop(m_fast_conn_params_timer_id);

    m_conn_param_negotiation_done = 0;
}

#ifdef CONN_PARAM_UPDATE_ON_CCCD
static void on_write(ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // Check if this the correct CCCD
    if (
        (p_evt_write->handle == m_conn_params_config.start_on_notify_cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        // Check if this is a 'start notification'
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            // Do connection parameter negotiation if necessary
            conn_params_negotiation();
        }
        else
        {
            uint32_t err_code;

            // Stop timer if running
            err_code = app_timer_stop(m_conn_params_timer_id);
            if ((err_code != NRF_SUCCESS) && (m_conn_params_config.error_handler != NULL))
            {
                m_conn_params_config.error_handler(err_code);
            }
        }
    }
}
#endif

static void on_conn_params_update(ble_evt_t * p_ble_evt)
{
	if (m_enabled_fast_parameter != 0) {
		// Faster conn_parameter does not go through negotiation process
		return;
	}

    // Copy the parameters
    m_current_conn_params = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params;

    conn_params_negotiation(&m_ios_conn_params);
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

#ifdef CONN_PARAM_UPDATE_ON_CCCD
        case BLE_GATTS_EVT_WRITE:
            on_write(p_ble_evt);
            break;
#endif

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            on_conn_params_update(p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}
