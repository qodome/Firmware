/*
 * Application logic to talk with sensors via I2C
 */

#include "sensor_service.h"
#include "twi_master.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble_sensor.h"
#include "nrf_delay.h"

#define ISL_ADDRESS 0x88

#define APP_TIMER_PRESCALER					0
#define ISL_SAMPLE_PERIOD					APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

static app_timer_id_t m_isl_timer_id;
extern ble_sensor_t m_sensor;

static void isl_timeout_handler(void* p_context)
{
	uint8_t test[4] = {0};

	ble_sensor_send_isl(&m_sensor, test);
}

void sensor_service_init(void)
{
	/*
	 * Hardware sensor currently does not work due to footprint error on PCB
	uint8_t chipId = 0;

    if (!twi_master_init()) {
        APP_ERROR_CHECK(NRF_ERROR_BUSY);
    } else {
    	if (!ss_isl_read(0x00, &chipId) || (chipId != 0xB0)) {
    		APP_ERROR_CHECK(NRF_ERROR_NOT_FOUND);
    	}
    }
    */

    APP_ERROR_CHECK(app_timer_create(&m_isl_timer_id, APP_TIMER_MODE_REPEATED, isl_timeout_handler));
}

bool ss_isl_read(uint8_t regAddr, uint8_t *cBuf)
{
    bool transfer_succeeded;
    transfer_succeeded  = twi_master_transfer(ISL_ADDRESS, &regAddr, 1, TWI_DONT_ISSUE_STOP);
    transfer_succeeded &= twi_master_transfer((ISL_ADDRESS | TWI_READ_BIT), cBuf, 1, TWI_ISSUE_STOP);
    return transfer_succeeded;
}

bool ss_isl_write(uint8_t regAddr, uint8_t val)
{
    uint8_t wData[2];

    wData[0] = regAddr;
    wData[1] = val;
    return twi_master_transfer(ISL_ADDRESS, wData, 2, TWI_ISSUE_STOP);
}

void ss_isl_start(void)
{
	APP_ERROR_CHECK(app_timer_start(m_isl_timer_id, ISL_SAMPLE_PERIOD, NULL));
}

void ss_isl_stop(void)
{
	APP_ERROR_CHECK(app_timer_stop(m_isl_timer_id));
}
