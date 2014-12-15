/*
 * timer_scheduler.c
 *
 *  Created on: 2014-10-30
 *      Author: Administrator
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
#include "ble_hts.h"
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
#include "pstorage.h"
#include "app_trace.h"
#include "nrf_sdm.h"
#include "app_util_platform.h"
#include "spi_master.h"
#include "nrf_delay.h"
#include "UTCtime_convert.h"
#include "ble_register_rw.h"
#include "temp_service.h"
#include "temp_date_time.h"
#include "temp_state.h"
#include "cmd_buffer.h"
#include "recorder.h"

#define APP_TIMER_PRESCALER			0
#define IC_SPS_SAMPLE_PERIOD		APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)
#define IT_SAMPLE_PERIOD_FAST		APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define IT_SAMPLE_PERIOD_SLOW		APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

#define MEASUREMENT_INTERVAL_DEFAULT	5	//FIXME		// default interval 30 seconds

static app_timer_id_t	m_sps_timer_id, m_it_timer_id, m_tm_timer_id;
bool m_tm_enabled = false;
bool m_it_enabled = false;
static uint32_t m_it_timeout_ticks = IT_SAMPLE_PERIOD_FAST;
uint16_t m_tm_interval = MEASUREMENT_INTERVAL_DEFAULT;
uint32_t m_tm_interval_ticks = APP_TIMER_TICKS(MEASUREMENT_INTERVAL_DEFAULT * 1000, APP_TIMER_PRESCALER);

#ifdef DEBUG_STATS
uint32_t adt_sample_sps_counts = 0;
uint32_t temp_tm_counts_30s= 0;
#endif

static cmd_buffer_t *lastReadBuffer = NULL;

iDo_send_indication_callback send_indication = NULL;
iDo_send_notification_callback send_notification = NULL;
iDo_advertise_callback send_advertise = NULL;

uint16_t temp_service_get_tm_interval()
{
	return m_tm_interval;
}

void temp_service_set_tm_intreval(uint16_t interval)
{
	m_tm_interval = interval;
	m_tm_interval_ticks = APP_TIMER_TICKS((uint32_t)m_tm_interval * 1000, APP_TIMER_PRESCALER);
	if (m_tm_enabled == true) {
		APP_ERROR_CHECK(app_timer_stop(m_tm_timer_id));
		APP_ERROR_CHECK(app_timer_start(m_tm_timer_id, m_tm_interval_ticks, NULL));
	}
}

void temp_tm_start(void)
{
	uint32_t err_code;

	m_tm_enabled = true;

	err_code = app_timer_start(m_it_timer_id, m_it_timeout_ticks, NULL);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(m_tm_timer_id, m_tm_interval_ticks, NULL);
	APP_ERROR_CHECK(err_code);
}

void temp_it_start(void)
{
	uint32_t err_code;

	m_it_enabled = true;

	err_code = app_timer_start(m_it_timer_id, m_it_timeout_ticks, NULL);
	APP_ERROR_CHECK(err_code);
}

void temp_tm_stop(void)
{
	m_tm_enabled = false;

	if (m_it_enabled == true) {
		APP_ERROR_CHECK(app_timer_stop(m_tm_timer_id));
	} else {
		APP_ERROR_CHECK(app_timer_stop(m_tm_timer_id));
		APP_ERROR_CHECK(app_timer_stop(m_it_timer_id));
	}
}

void temp_it_stop(void)
{
	m_it_enabled = false;

	if (m_tm_enabled == false) {
		APP_ERROR_CHECK(app_timer_stop(m_it_timer_id));
	}
}

uint8_t temp_advertise_temp(void)
{
	if (m_tm_enabled == true || m_it_enabled == true) {
		return 1;
	} else {
		return 0;
	}
}

// Do send intermediate temperature update
static void __do_send_tm(int16_t temp_raw, uint8_t flag, ble_date_time_t *ptc)
{
	ble_hts_meas_t   adt_meas;
	int32_t			 temp_meas_v = 0;

	// signed 16 bits to signed 32 bits
	if (temp_raw & 0x8000) {
		temp_meas_v = 0xFFFF0000 | (int32_t)temp_raw;
	} else {
		temp_meas_v = (int32_t)temp_raw;
	}

	memset((void *) &adt_meas, 0, sizeof(adt_meas));

	// Temperature value
	adt_meas.temp_in_fahr_units = false;
	adt_meas.temp_in_celcius.exponent = -4;
	adt_meas.temp_in_celcius.mantissa = temp_meas_v * 625;

	// Temperature type
	if(flag & TYPE_VALID_FLAG){
		adt_meas.temp_type_present = 1;
		adt_meas.temp_type = 2;
	}

	if (flag & TIME_VALID_FLAG) {
		adt_meas.time_stamp_present = true;
		memcpy((void *)&(adt_meas.time_stamp), (void *)ptc, sizeof(ble_date_time_t));
	}

	if (send_indication != NULL) {
		send_indication(&adt_meas);
	}
}

static void __temp_tm_timeout_handler(void * p_event_data , uint16_t event_size)
{
	int16_t temp = 0;
	uint8_t validFlag = 0;
	ble_date_time_t tc;

	if (temp_state_mesaurement_ready()) {
		cmd_buffer_push_temp(temp_state_get_last_filtered_temp());
	}

	memset((void *)&tc, 0, sizeof(tc));

	lastReadBuffer = CBGetNextBufferForTX(&temp, &validFlag, &tc);
	if (lastReadBuffer != NULL) {
		__do_send_tm(temp, validFlag, &tc);
	}

#ifdef DEBUG_STATS
	temp_tm_counts_30s ++;
#endif
}

static void temp_tm_timeout_handler(void *p_context)
{
	uint32_t err_code;

	err_code=app_sched_event_put(NULL, 0, __temp_tm_timeout_handler);
	APP_ERROR_CHECK(err_code);
}

static void temp_it_timeout_handler(void  *p_context)
{
	uint32_t err_code;

	spi_write(0x01,0x40);

	err_code = app_timer_start(m_sps_timer_id, IC_SPS_SAMPLE_PERIOD, NULL);
	APP_ERROR_CHECK(err_code);
}

// Do send intermediate temperature update
static void __do_send_it(int16_t temp_raw)
{
	ble_hts_meas_t   adt_meas;
	uint8_t          flag_type = 0;
	int32_t			 temp_meas_v = 0;

	// signed 16 bits to signed 32 bits
	if (temp_raw & 0x8000) {
		temp_meas_v = 0xFFFF0000 | (int32_t)temp_raw;
	} else {
		temp_meas_v = (int32_t)temp_raw;
	}

	memset((void *) &adt_meas, 0, sizeof(adt_meas));

	// Temperature value
	adt_meas.temp_in_fahr_units = false;
	adt_meas.temp_in_celcius.exponent = -4;
	adt_meas.temp_in_celcius.mantissa = temp_meas_v * 625;

	// Temperature type
	flag_type = temp_state_is_attached();
	if (flag_type) {
		adt_meas.temp_type_present = 1;
		adt_meas.temp_type = 2;
	}

	if (send_notification != NULL) {
		send_notification(&adt_meas);
	}
}

// Function scheduled background
static void __sps_timeout_handler(void * p_event_data , uint16_t event_size)
{
	int16_t result = 0;
	static int16_t lastTemp = 0;
	static uint32_t staleCount = 0;

	spi_write(0x01, 0x60);
	result = spi_read(0x02);
    if (result & 0x8000) {
        // negative
    	result = result >> 3;
    	result &= 0x1FFF;
    	result |= 0xE000;
    } else {
    	result = result >> 3;
    	result &= 0x1FFF;
    }

	if (((result - lastTemp) >= 2) || ((lastTemp - result) >= 2)) {
		if (staleCount >= 10) {
			staleCount = 0;
			m_it_timeout_ticks = IT_SAMPLE_PERIOD_FAST;

			if (m_it_enabled == true || m_tm_enabled == true) {
				APP_ERROR_CHECK(app_timer_stop(m_it_timer_id));
				APP_ERROR_CHECK(app_timer_start(m_it_timer_id, m_it_timeout_ticks, NULL));
			}
		}
		staleCount = 0;
	} else {
		staleCount++;
		if (staleCount == 10) {
			m_it_timeout_ticks = IT_SAMPLE_PERIOD_SLOW;

			if (m_it_enabled == true || m_tm_enabled == true) {
				APP_ERROR_CHECK(app_timer_stop(m_it_timer_id));
				APP_ERROR_CHECK(app_timer_start(m_it_timer_id, m_it_timeout_ticks, NULL));
			}
		}
	}
	lastTemp = result;

	temp_state_update(result);
	recorder_add_temperature(result);

	if ((m_it_enabled == true && m_tm_enabled == false) ||
			(m_it_enabled == true && (!temp_state_mesaurement_ready()))) {
		__do_send_it(result);
	}

	if (send_advertise != 0) {
		send_advertise(result);
	}

#ifdef DEBUG_STATS
	adt_sample_sps_counts++;
#endif
}

static void sps_timeout_handler(void * p_context)
{
	APP_ERROR_CHECK(app_sched_event_put(NULL, 0, __sps_timeout_handler));
}

void temp_init(iDo_send_indication_callback indication_callback,
				iDo_send_notification_callback notification_callback,
				iDo_advertise_callback advertise_callback)
{
	uint32_t err_code;

	send_indication = indication_callback;
	send_notification = notification_callback;
	send_advertise = advertise_callback;

	//Structure for SPI master configuration, initialized by default values.
	spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;

    //Configure SPI master.
    spi_config.SPI_Pin_SCK = 15;
    spi_config.SPI_Pin_MISO = 14;
    spi_config.SPI_Pin_MOSI = 17;
    spi_config.SPI_Pin_SS = 18;
    spi_config.SPI_CONFIG_ORDER = SPI_CONFIG_ORDER_MsbFirst;
    spi_config.SPI_CONFIG_CPOL = SPI_CONFIG_CPOL_ActiveLow;
    spi_config.SPI_CONFIG_CPHA = SPI_CONFIG_CPHA_Trailing;
    //spi_config.SPI_Freq = SPI_FREQUENCY_FREQUENCY_K125;

    //Initialize SPI master.
    APP_ERROR_CHECK(spi_master_open(SPI_MASTER_0, &spi_config));

	spi_write(0x01, 0x60);

    // Create timer for 2s or 10s notify
    err_code = app_timer_create(&m_it_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                temp_it_timeout_handler);//temp_sps_start
    APP_ERROR_CHECK(err_code);

    // Create timer for 100ms ADT7320 1SPS
    err_code = app_timer_create(&m_sps_timer_id,
    		                    APP_TIMER_MODE_SINGLE_SHOT, /**< The timer will expire only once.  APP_TIMER_MODE_SINGLE_SHOT */
    			                sps_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // create timer for 30s indication
    err_code = app_timer_create(&m_tm_timer_id,
    		                    APP_TIMER_MODE_REPEATED,
    		                    temp_tm_timeout_handler);
    APP_ERROR_CHECK(err_code);

    m_tm_enabled = false;
    m_it_enabled = false;
}

//	get sample from cmd_buffer && send
void temp_measurement_confirm(void)
{
	int16_t temp = 0;
	uint8_t validFlag = 0;
	ble_date_time_t tc;

    if (lastReadBuffer != NULL) {
    	cmd_put_buffer(lastReadBuffer);
    	lastReadBuffer = CBGetNextBufferForTX(&temp, &validFlag, &tc);
    	if (lastReadBuffer != NULL) {
    		__do_send_tm(temp, validFlag, &tc);
    	}
    }
}

