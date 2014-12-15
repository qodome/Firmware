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

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

#include "ble_ido.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "ble_date_time.h"
#include "UTCtime_convert.h"
#include "temp_date_time.h"
#include "app_error.h"
#include "temp_service.h"
#include "recorder.h"
#include "ble_conn_params.h"
#include "ble_memdump.h"
#include "app_scheduler.h"

#define OPCODE_LENGTH 1                                                    /**< Length of opcode inside Health Thermometer Measurement packet. */
#define HANDLE_LENGTH 2                                                    /**< Length of handle inside Health Thermometer Measurement packet. */
#define MAX_HTM_LEN   (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)  /**< Maximum size of a transmitted Health Thermometer Measurement. */

#define INTERVAL_RANGE_MIN				1			// minimum: 1 second
#define INTERVAL_RANGE_MAX				255			// maximum: 255 seconds

// Health Thermometer Measurement flag bits
#define HTS_MEAS_FLAG_TEMP_UNITS_BIT (0x01 << 0)  /**< Temperature Units flag. */
#define HTS_MEAS_FLAG_TIME_STAMP_BIT (0x01 << 1)  /**< Time Stamp flag. */
#define HTS_MEAS_FLAG_TEMP_TYPE_BIT  (0x01 << 2)  /**< Temperature Type flag. */

bool       m_is_notification_enabled = false;                     /**< Variable to indicate whether the notification is enabled by the peer.*/
bool       m_is_indication_enabled = false;                     /**< Variable to indicate whether the notification is enabled by the peer.*/
uint8_t    enable_fast_read = 0;

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_hts       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_hts_t * p_hts, ble_evt_t * p_ble_evt)
{
    p_hts->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    enable_fast_read = 0;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_hts       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_hts_t * p_hts, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_hts->conn_handle = BLE_CONN_HANDLE_INVALID;
    m_is_notification_enabled = false;					// FIXME: check if notify/indication disabled after BLE reconnection
    m_is_indication_enabled = false;
    enable_fast_read = 0;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_hts       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_hts_t * p_hts, ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if ((p_evt_write->handle == p_hts->meas_handle.cccd_handle) && (p_evt_write->len == 2)) {
		if (p_hts->evt_handler != NULL) {
			ble_hts_evt_t  evt;
			m_is_indication_enabled = ble_srv_is_indication_enabled(p_evt_write->data);

			if (m_is_indication_enabled) {
				evt.evt_type = BLE_HTS_EVT_INDICATION_ENABLED;
			} else {
				evt.evt_type = BLE_HTS_EVT_INDICATION_DISABLED;
			}
			p_hts->evt_handler(p_hts, &evt);
		}
	} else if ((p_evt_write->handle == p_hts->intermediate_handle.cccd_handle) && (p_evt_write->len == 2)) {
		if (p_hts->evt_handler != NULL) {
			ble_hts_evt_t  evt;
			m_is_notification_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

			if (m_is_notification_enabled) {
				evt.evt_type = BLE_HTS_EVT_NOTIFICATION_ENABLED;
			} else {
				evt.evt_type = BLE_HTS_EVT_NOTIFICATION_DISABLED;
			}
			p_hts->evt_handler(p_hts, &evt);
		}
	} else if ((p_evt_write->handle == p_hts->unix_time_handle.value_handle) && (p_evt_write->len == 7)) {
		ble_date_time_t time;

		ble_date_time_decode(&time, p_evt_write->data);
		if(time.month<12 && time.day<31 && time.hours <24 && time.minutes<60 && time.seconds<60){
			date_time_set(osal_ConvertUTCSecs(&time));
		}
	} else if ((p_evt_write->handle == p_hts->intermediate_handle.value_handle) && (p_evt_write->len == 12)) {
		ble_date_time_t  time;

		if (memcmp(p_evt_write->data, "_QoDoMe_2014", 12) == 0) {
			APP_ERROR_CHECK(ble_memdump_init());
		} else {
			ble_date_time_decode(&time, p_evt_write->data + 5);
			recorder_set_read_base_ts(&time);
		}
	}
}

static void _it_read_enable_fast_conn(void * p_event_data , uint16_t event_size)
{
	if (enable_fast_read == 0) {
		enable_fast_read = 1;
		ble_conn_enable_fast_read();
	}
}

// Handle unix time read
static void on_rw_authorize_request(ble_hts_t * p_hts, ble_gatts_evt_t * p_gatts_evt)
{
	ble_date_time_t tm;
	uint8_t result_buf[7] = {0};
	uint8_t temp_record[13] = {0};
	ble_gatts_rw_authorize_reply_params_t   auth_params;
	ble_gatts_rw_authorize_reply_params_t * p_auth_params = &auth_params;
	memset((void *)&auth_params, 0, sizeof(auth_params));

	ble_gatts_evt_rw_authorize_request_t * p_auth_req = &p_gatts_evt->params.authorize_request;

    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_READ) {
        if (p_auth_req->request.read.handle == p_hts->unix_time_handle.value_handle) {
        	memset((void *)&tm, 0, sizeof(tm));
        	if (flag_time_stamp_get()) {
        		osal_ConvertUTCTime(&tm, date_time_get());
        	}

        	auth_params.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
        	auth_params.params.read.p_data = result_buf;
        	auth_params.params.read.len    = ble_date_time_encode(&tm, result_buf);
        	auth_params.params.read.update = 1;

        	APP_ERROR_CHECK(sd_ble_gatts_rw_authorize_reply(p_hts->conn_handle, p_auth_params));
        } else if (p_auth_req->request.read.handle == p_hts->intermediate_handle.value_handle) {
        	ble_date_time_t tc;
        	int16_t temp_raw = 0;
        	int32_t sign = -4;
        	int32_t t32 = 0;
        	int8_t rssi = 0;

        	app_sched_event_put(NULL, 0, _it_read_enable_fast_conn);

        	memset((uint8_t *)&tc, 0, sizeof(tc));
        	temp_raw = recorder_get_temperature(&tc, &rssi);
        	ble_date_time_encode(&tc, result_buf);
            if (temp_raw & 0x8000) {
                t32 = (int32_t)temp_raw | 0xFFFF0000;
            } else {
                t32 = temp_raw;
            }
            t32 = (sign << 24) | ((t32 * 625) & 0xFFFFFF);

        	temp_record[0] = 0x02;
        	memcpy((temp_record + 1), (uint8_t *)&t32, 4);
        	memcpy((temp_record + 5), result_buf, 7);
        	temp_record[12] = (uint8_t)rssi;

        	auth_params.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
        	auth_params.params.read.p_data = temp_record;
        	auth_params.params.read.len    = 13;
        	auth_params.params.read.update = 1;

        	APP_ERROR_CHECK(sd_ble_gatts_rw_authorize_reply(p_hts->conn_handle, p_auth_params));
        }
    } else if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) {
        if (p_auth_req->request.write.handle == p_hts->meas_interval_handle.value_handle) {
        	uint16_t new_interval = 0;
        	uint16_t len = 2;

        	auth_params.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;

        	if (p_auth_req->request.write.len != 2) {
        		auth_params.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_INVALID_ATT_VAL_LENGTH;
        	} else {
        		new_interval = ((uint16_t)p_auth_req->request.write.data[1]) << 8 | ((uint16_t)p_auth_req->request.write.data[0]);
        		if ((new_interval <= INTERVAL_RANGE_MAX) && (new_interval >= INTERVAL_RANGE_MIN)) {
        			auth_params.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
        			temp_service_set_tm_intreval(new_interval);
        			sd_ble_gatts_value_set(p_hts->meas_interval_handle.value_handle, 0, &len, p_auth_req->request.write.data);
        		} else {
        			auth_params.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_INVALID_ATT_VAL_LENGTH;
        		}
        	}
        	APP_ERROR_CHECK(sd_ble_gatts_rw_authorize_reply(p_hts->conn_handle, p_auth_params));
        }
    }
}

/**@brief Function for handling the HVC event.
 *
 * @details Handles HVC events from the BLE stack.
 *
 * @param[in]   p_hts       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_hvc(ble_hts_t * p_hts, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_hvc_t * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

    if (p_hvc->handle == p_hts->meas_handle.value_handle)
    {
        ble_hts_evt_t evt;

        evt.evt_type = BLE_HTS_EVT_INDICATION_CONFIRMED;
        p_hts->evt_handler(p_hts, &evt);
    }
}


void ble_hts_on_ble_evt(ble_hts_t * p_hts, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_hts, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_hts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_hts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        	on_rw_authorize_request(p_hts, &p_ble_evt->evt.gatts_evt);
        	break;

        case BLE_GATTS_EVT_HVC:
            on_hvc(p_hts, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for encoding a Health Thermometer Measurement.
 *
 * @param[in]   p_hts              Health Thermometer Service structure.
 * @param[in]   p_hts_meas         Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t hts_measurement_encode(ble_hts_t      * p_hts,
                                      ble_hts_meas_t * p_hts_meas,
                                      uint8_t        * p_encoded_buffer)
{
    uint8_t  flags = 0;
    uint8_t  len   = 1;
    uint32_t encoded_temp;

    // Flags field
    if (p_hts_meas->temp_in_fahr_units)
    {
        flags |= HTS_MEAS_FLAG_TEMP_UNITS_BIT;
    }
    if (p_hts_meas->time_stamp_present)
    {
        flags |= HTS_MEAS_FLAG_TIME_STAMP_BIT;
    }

    // Temperature Measurement Value field
    if (p_hts_meas->temp_in_fahr_units)
    {
        flags |= HTS_MEAS_FLAG_TEMP_UNITS_BIT;

        encoded_temp = ((p_hts_meas->temp_in_fahr.exponent << 24) & 0xFF000000) |
                       ((p_hts_meas->temp_in_fahr.mantissa <<  0) & 0x00FFFFFF);
    }
    else
    {
        encoded_temp = ((p_hts_meas->temp_in_celcius.exponent << 24) & 0xFF000000) |
                       ((p_hts_meas->temp_in_celcius.mantissa <<  0) & 0x00FFFFFF);
    }
    len += uint32_encode(encoded_temp, &p_encoded_buffer[len]);

    // Time Stamp field
    if (p_hts_meas->time_stamp_present)
    {
        flags |= HTS_MEAS_FLAG_TIME_STAMP_BIT;
        len   += ble_date_time_encode(&p_hts_meas->time_stamp, &p_encoded_buffer[len]);
    }

    // Temperature Type field
    if (p_hts_meas->temp_type_present)
    {
        flags                  |= HTS_MEAS_FLAG_TEMP_TYPE_BIT;
        p_encoded_buffer[len++] = p_hts_meas->temp_type;
    }

    // Flags field
    p_encoded_buffer[0] = flags;

    return len;
}

// Temperature measurement
static uint32_t hts_measurement_char_add(ble_hts_t * p_hts)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_hts_meas_t      initial_htm;
    uint8_t             encoded_htm[MAX_HTM_LEN];

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.indicate = 1;
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = &cccd_md;
    char_md.p_sccd_md           = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_TEMPERATURE_MEASUREMENT_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    memset(&initial_htm, 0, sizeof(initial_htm));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = hts_measurement_encode(p_hts, &initial_htm, encoded_htm);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_HTM_LEN;
    attr_char_value.p_value   = encoded_htm;

    return sd_ble_gatts_characteristic_add(p_hts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_hts->meas_handle);
}

// Intermediate temperature
static uint32_t hts_intermediate_temp_char_add(ble_hts_t * p_hts)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	ble_hts_meas_t      initial_htm;
	uint8_t             encoded_htm[MAX_HTM_LEN];

	memset(&cccd_md, 0, sizeof(cccd_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.indicate = 1;
	char_md.char_props.read 	= 1;
	char_md.char_props.write 	= 1;
	char_md.p_char_user_desc    = NULL;
	char_md.p_char_pf           = NULL;
	char_md.p_user_desc_md      = NULL;
	char_md.p_cccd_md           = &cccd_md;
	char_md.p_sccd_md           = NULL;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_INTERMEDIATE_TEMPERATURE_CHAR);

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.rd_auth    = 1;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));
	memset(&initial_htm, 0, sizeof(initial_htm));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = hts_measurement_encode(p_hts, &initial_htm, encoded_htm);
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = MAX_HTM_LEN;
	attr_char_value.p_value   = encoded_htm;

    return sd_ble_gatts_characteristic_add(p_hts->service_handle,
	                                       &char_md,
	                                       &attr_char_value,
	                                       &p_hts->intermediate_handle);
}

// Measurement interval
static uint32_t hts_meas_interval_char_add(ble_hts_t * p_hts)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	uint16_t            interval_sec;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read = 1;
	char_md.char_props.write = 1;
	char_md.p_char_user_desc    = NULL;
	char_md.p_char_pf           = NULL;
	char_md.p_user_desc_md      = NULL;
	char_md.p_cccd_md           = NULL;
	char_md.p_sccd_md           = NULL;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_MEASUREMENT_INTERVAL_CHAR);

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 1;
	attr_md.vlen       = 0;

	memset(&attr_char_value, 0, sizeof(attr_char_value));
	interval_sec = temp_service_get_tm_interval();

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = 2;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = 2;
	attr_char_value.p_value   = (uint8_t *)&interval_sec;

    return sd_ble_gatts_characteristic_add(p_hts->service_handle,
	                                       &char_md,
	                                       &attr_char_value,
	                                       &p_hts->meas_interval_handle);
}

// Interval range
static uint32_t hts_interval_range_char_add(ble_hts_t * p_hts)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	uint16_t            interval_range[2];

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read = 1;
	char_md.p_char_user_desc    = NULL;
	char_md.p_char_pf           = NULL;
	char_md.p_user_desc_md      = NULL;
	char_md.p_cccd_md           = NULL;
	char_md.p_sccd_md           = NULL;

	BLE_UUID_BLE_ASSIGN(ble_uuid, 0x2906);	// GATT_VALID_RANGE_UUID

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 0;

	memset(&attr_char_value, 0, sizeof(attr_char_value));
	interval_range[0] = INTERVAL_RANGE_MIN;
	interval_range[1] = INTERVAL_RANGE_MAX;

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = 4;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = 4;
	attr_char_value.p_value   = (uint8_t *)interval_range;

    return sd_ble_gatts_characteristic_add(p_hts->service_handle,
	                                       &char_md,
	                                       &attr_char_value,
	                                       &p_hts->interval_range_handle);
}

// Unix time
static uint32_t hts_unix_time_char_add(ble_hts_t * p_hts)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	uint8_t     		time_now[7] = {0};

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read = 1;
	char_md.char_props.write = 1;
	char_md.p_char_user_desc    = NULL;
	char_md.p_char_pf           = NULL;
	char_md.p_user_desc_md      = NULL;
	char_md.p_cccd_md           = NULL;
	char_md.p_sccd_md           = NULL;

	BLE_UUID_BLE_ASSIGN(ble_uuid, 0xAAAA);	// legacy characteristic inherited from CC2541 iDo

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.rd_auth    = 1;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 0;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = 7;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = 7;
	attr_char_value.p_value   = time_now;

    return sd_ble_gatts_characteristic_add(p_hts->service_handle,
	                                       &char_md,
	                                       &attr_char_value,
	                                       &p_hts->unix_time_handle);
}

uint32_t ble_hts_init(ble_hts_t * p_hts, const ble_hts_init_t * p_hts_init)
{
	uint32_t   err_code;
	ble_uuid_t ble_uuid;

	// Initialize service structure
	p_hts->evt_handler = p_hts_init->evt_handler;
	p_hts->conn_handle = BLE_CONN_HANDLE_INVALID;

	// Add service
	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HEALTH_THERMOMETER_SERVICE);

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_hts->service_handle);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	// Add measurement characteristic
	err_code = hts_measurement_char_add(p_hts);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	// Add intermediate temperature characteristic
	err_code = hts_intermediate_temp_char_add(p_hts);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	// Add measurement interval characteristic
	err_code = hts_meas_interval_char_add(p_hts);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	// Add interval range characteristic
	err_code = hts_interval_range_char_add(p_hts);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	// Add unix time characteristic
	err_code = hts_unix_time_char_add(p_hts);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	return NRF_SUCCESS;
}

uint32_t ble_hts_send_tm(ble_hts_t * p_hts, ble_hts_meas_t * p_hts_meas)
{
    uint32_t err_code;

    // Send value if connected
    if (p_hts->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_hts_meas[MAX_HTM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = hts_measurement_encode(p_hts, p_hts_meas, encoded_hts_meas);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_hts->meas_handle.value_handle;
        hvx_params.type   = BLE_GATT_HVX_INDICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hts_meas;

        err_code = sd_ble_gatts_hvx(p_hts->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_hts_send_it(ble_hts_t * p_hts, ble_hts_meas_t * p_hts_meas)
{
    uint32_t err_code;
    // Send value if connected and notifying
    if (p_hts->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_hts_meas[MAX_HTM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = hts_measurement_encode(p_hts, p_hts_meas, encoded_hts_meas);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_hts->intermediate_handle.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hts_meas;

        err_code = sd_ble_gatts_hvx(p_hts->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_hts_is_indication_enabled(ble_hts_t * p_hts, bool * p_indication_enabled)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    uint16_t len = BLE_CCCD_VALUE_LEN;

    err_code = sd_ble_gatts_value_get(p_hts->meas_handle.cccd_handle,
                                      0,
                                      &len,
                                      cccd_value_buf);
    if (err_code == NRF_SUCCESS)
    {
        *p_indication_enabled = ble_srv_is_indication_enabled(cccd_value_buf);
    }
    return err_code;
}

uint32_t ble_hts_is_notification_enabled(ble_hts_t * p_hts, bool * p_notification_enabled)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    uint16_t len = BLE_CCCD_VALUE_LEN;

    err_code = sd_ble_gatts_value_get(p_hts->intermediate_handle.cccd_handle,
                                      0,
                                      &len,
                                      cccd_value_buf);
    if (err_code == NRF_SUCCESS)
    {
        *p_notification_enabled = ble_srv_is_notification_enabled(cccd_value_buf);
    }
    return err_code;
}

