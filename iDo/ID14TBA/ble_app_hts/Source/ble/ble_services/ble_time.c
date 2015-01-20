/*
 * date_time.c
 *
 *  Created on: 2014-10-31
 *      Author: Administrator
 */

/* Copyright (c) 2012 Nordic Semiconductor. All Rigtime Reserved.
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

/* Attention!
 *  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#include "ble_time.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_error.h"
#include "UTCtime_convert.h"
#include "temp_service.h"
#include "app_timer.h"
#include "nrf51.h"

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define TEMP_DATE_TIME_SAMPLE_PERIOD         APP_TIMER_TICKS(800, APP_TIMER_PRESCALER) /**< temperature measurement interval (ticks). */

static app_timer_id_t temp_date_time_id;

static uint8_t last_st1_h = 0;
uint32_t second_now = 0;
uint32_t second_relative = 0;
uint8_t data_time_initial_time_set = 0;

#ifdef DEBUG_STATS
uint32_t power_on_seconds = 0;
#endif

static void recorder_rtc_tick(void * p_context);


/**@brief Function for returning the current value of the RTC1 counter.
 *
 * @return     Current value of the RTC1 counter.
 */

static __INLINE uint32_t rtc1_counter_get(void)
{
	return NRF_RTC1->COUNTER;
}


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_time       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_time_t * p_time, ble_evt_t * p_ble_evt)
{
	p_time->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_time       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_time_t * p_time, ble_evt_t * p_ble_evt)
{
	UNUSED_PARAMETER(p_ble_evt);
	p_time->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_time       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_time_t * p_time, ble_evt_t * p_ble_evt)
{
	ble_date_time_t time;

	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if (p_evt_write->handle == p_time->time_handles.value_handle)
	{
		ble_date_time_decode(&time, &(p_evt_write->data[0]));
		if(time.month <= 12 && time.day <= 31 && time.hours < 24 && time.minutes < 60 && time.seconds < 60){
			data_time_initial_time_set = 1;
			second_now = osal_ConvertUTCSecs(&time);
		}
	}
}

static void on_rw_authorize_request(ble_time_t * p_time, ble_gatts_evt_t * p_gatts_evt)
{
	ble_date_time_t tm;
	uint8_t result_buf[7] = {0};

	ble_gatts_evt_rw_authorize_request_t * p_auth_req = &p_gatts_evt->params.authorize_request;

	if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_READ)
	{
		if (p_auth_req->request.read.handle == p_time->time_handles.value_handle)
		{
			ble_gatts_rw_authorize_reply_params_t   auth_params;
			ble_gatts_rw_authorize_reply_params_t * p_auth_params = &auth_params;

			memset((void *)&tm, 0, sizeof(tm));
			if (data_time_initial_time_set != 0) {
				osal_ConvertUTCTime(&tm, second_now);
			}

			memset((void *)&auth_params, 0, sizeof(auth_params));
			auth_params.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
			auth_params.params.read.p_data = result_buf;
			auth_params.params.read.len    = ble_date_time_encode(&tm, result_buf);
			auth_params.params.read.update = 1;

			APP_ERROR_CHECK(sd_ble_gatts_rw_authorize_reply(p_time->conn_handle, p_auth_params));
		}
	}
}

void ble_time_on_ble_evt(ble_time_t * p_time, ble_evt_t * p_ble_evt)
{
	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		on_connect(p_time, p_ble_evt);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnect(p_time, p_ble_evt);
		break;

	case BLE_GATTS_EVT_WRITE:
		on_write(p_time, p_ble_evt);
		break;

	case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
		on_rw_authorize_request(p_time, &p_ble_evt->evt.gatts_evt);
		break;

	default:
		// No implementation needed.
		break;
	}
}

/**@brief Function for adding Temperature Type characteristics.
 *
 * @param[in]   p_time        Health Thermometer Service structure.
 * @param[in]   p_time_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t time_value_char_add(ble_time_t * p_time, const ble_time_init_t * p_time_init)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;

	uint8_t             init_time[7] = {0};

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read  = 1;
	char_md.char_props.write  = 1;
	char_md.p_char_user_desc = NULL;
	char_md.p_char_pf        = NULL;
	char_md.p_user_desc_md   = NULL;
	char_md.p_cccd_md        = NULL;
	char_md.p_sccd_md        = NULL;

	BLE_UUID_BLE_ASSIGN(ble_uuid, 0x2A08);

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.read_perm  = p_time_init->time_type_attr_md.read_perm;
	attr_md.write_perm = p_time_init->time_type_attr_md.write_perm;
	attr_md.rd_auth    = 1;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 0;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = 7;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = 7;
	attr_char_value.p_value   = init_time;

	return sd_ble_gatts_characteristic_add(p_time->service_handle,
			&char_md,
			&attr_char_value,
			&p_time->time_handles);
}

uint32_t ble_time_init(ble_time_t * p_time, const ble_time_init_t * p_time_init)
{
	uint32_t   err_code;
	ble_uuid_t ble_uuid;

	// Initialize service structure
	p_time->evt_handler = p_time_init->evt_handler;
	p_time->conn_handle = BLE_CONN_HANDLE_INVALID;

	// Add service
	BLE_UUID_BLE_ASSIGN(ble_uuid, 0x1805);// BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HEALTH_THERMOMETER_SERVICE);

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_time->service_handle);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	// Add measurement characteristic
	err_code = time_value_char_add(p_time, p_time_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	return NRF_SUCCESS;
}



void date_time_init(void)//send_temp_callback callback
{
	uint32_t err_code;
	// Create timers ..
	err_code = app_timer_create(&temp_date_time_id,
			APP_TIMER_MODE_REPEATED,
			recorder_rtc_tick);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(temp_date_time_id,TEMP_DATE_TIME_SAMPLE_PERIOD, NULL);
	APP_ERROR_CHECK(err_code);
}

static void recorder_rtc_tick(void * p_context)
{
	uint32_t sleepTimeRegister = 0;

	sleepTimeRegister = rtc1_counter_get();
	if (((uint8_t *)&sleepTimeRegister)[1] & 0x80) {
		if (last_st1_h != 0x80) {
			last_st1_h = 0x80;
			second_now++;
			second_relative++;

#ifdef DEBUG_STATS
			power_on_seconds++;
#endif

		}
	} else {
		if (last_st1_h == 0x80) {
			last_st1_h = 0x00;
			second_now++;
			second_relative++;

#ifdef DEBUG_STATS
			power_on_seconds++;
#endif

		}
	}
}

uint32_t date_time_get_relative(void)
{
	return second_now;
}

uint32_t date_time_get_wall(void)
{
	return second_now;
}

void date_time_set_wall(uint32_t t)
{
	data_time_initial_time_set = 1;
	second_now = t;
}

uint8_t date_time_initialized(void)
{
	return data_time_initial_time_set;
}
