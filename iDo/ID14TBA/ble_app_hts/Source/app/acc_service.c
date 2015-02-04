/*
 * ADXL362 ACC Service
 */

#include <stdint.h>
#include <string.h>
#include "softdevice_handler.h"
#include "app_util_platform.h"
#include "acc_service.h"
#include "spi_master.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "nrf_gpio.h"
#include "persistent.h"

//#define ACC_INTERRUPT		1

static app_timer_id_t m_acc_timer_id;
extern ble_acc_t *acc_ptr;
#ifdef ACC_INTERRUPT
static app_gpiote_user_id_t m_gpio_uid;
#endif

uint8_t acc_data[20] = {0};
uint16_t acc_cnt = 0;
uint8_t acc_data_enabled = 0;
uint8_t acc_sample_cnt = 0;
uint16_t acc_int_cnt = 0;
uint16_t acc_act_cnt = 0;
uint16_t acc_inact_cnt = 0;
uint8_t acc_working_mode = 1;
uint8_t acc_status;

uint8_t acc_service_spi_read(uint8_t addr)
{
	uint8_t temp_tx_buffer[3] = {0, 0, 0};
	uint8_t temp_rx_buffer[3] = {0, 0, 0};

    temp_tx_buffer[0] = 0x0B;
    temp_tx_buffer[1] = addr;
    APP_ERROR_CHECK(spi_master_send_recv(SPI_MASTER_1, temp_tx_buffer, temp_rx_buffer, 3));

    return temp_rx_buffer[2];
}

void acc_service_spi_write(uint8_t addr, uint8_t cmd)
{
	uint8_t temp_tx_buffer[3] = {0, 0, 0};
	uint8_t temp_rx_buffer[3] = {0, 0, 0};

    temp_tx_buffer[0] = 0x0A;
    temp_tx_buffer[1] = addr;
    temp_tx_buffer[2] = cmd;
    APP_ERROR_CHECK(spi_master_send_recv(SPI_MASTER_1, temp_tx_buffer, temp_rx_buffer, 3));
}

void acc_service_start(void)
{

}

void acc_service_stop(void)
{

}

void acc_fifo_read(uint8_t *buf, uint8_t len)
{
	uint8_t tx_buf[20] = {0}, rx_buf[20] = {0};
	uint8_t idx;

	tx_buf[0] = 0x0D;
	 APP_ERROR_CHECK(spi_master_send_recv(SPI_MASTER_1, tx_buf, rx_buf, (len + 1)));
	for (idx = 0; idx < len; idx++) {
		buf[idx] = rx_buf[idx + 1];
	}
}

#ifdef ACC_INTERRUPT
static void acc_timer_start_trigger(void * p_event_data , uint16_t event_size)
{
	APP_ERROR_CHECK(app_timer_start(m_acc_timer_id, APP_TIMER_TICKS(5000, 0), NULL));
}

static void acc_timer_stop_trigger(void * p_event_data , uint16_t event_size)
{
	APP_ERROR_CHECK(app_timer_stop(m_acc_timer_id));
}

void int1_gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
    if (event_pins_low_to_high & 0x100) {
        acc_int_cnt++;
        acc_status = acc_service_spi_read(0x0B);
        if (acc_status & 0x10) {
            acc_act_cnt++;
            app_sched_event_put(NULL, 0, acc_timer_start_trigger);
        }
        if (acc_status & 0x20) {
            acc_inact_cnt++;
            app_sched_event_put(NULL, 0, acc_timer_stop_trigger);
        }
    }
}
#endif

static void acc_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    uint16_t acc_fifo_sample_cnt;
    uint8_t pkt_cnt;

    if (acc_service_spi_read(0x00) != 0xAD) {
    	persistent_record_error(PERSISTENT_ERROR_ADXL362, 1);
    	return;
    }

    if (acc_working_mode == 0) {
        acc_fifo_sample_cnt = ((uint16_t)(acc_service_spi_read(0x0D) & 0x03) << 8) | (uint16_t)acc_service_spi_read(0x0C);
        // Get 3 samples into one pkt, three packts each timeout period
        if (acc_fifo_sample_cnt > 0) {
            pkt_cnt = 0;
            while ((pkt_cnt <= 4) && (acc_fifo_sample_cnt > 0)) {
                if (acc_fifo_sample_cnt >= (9 - acc_sample_cnt)) {
                    acc_fifo_read(&(acc_data[2 + acc_sample_cnt * 2]), 2 * (9 - acc_sample_cnt));
                    acc_fifo_sample_cnt -= (9 - acc_sample_cnt);
                    acc_sample_cnt = 0;

                    // Send value if connected and notifying
                    if (acc_data_enabled == 1 && acc_ptr->conn_handle != BLE_CONN_HANDLE_INVALID) {
                        uint16_t               hvx_len;
                        ble_gatts_hvx_params_t hvx_params;

                        hvx_len = 20;

                        memset(&hvx_params, 0, sizeof(hvx_params));
                        *(uint16_t *)acc_data = acc_cnt;
                        acc_cnt++;

                        hvx_params.handle = acc_ptr->acc_data_handle.value_handle;
                        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
                        hvx_params.offset = 0;
                        hvx_params.p_len  = &hvx_len;
                        hvx_params.p_data = acc_data;

                        sd_ble_gatts_hvx(acc_ptr->conn_handle, &hvx_params);
                        pkt_cnt++;
                    }
                } else {
                    acc_fifo_read(&(acc_data[2 + acc_sample_cnt * 2]), 2 * acc_fifo_sample_cnt);
                    acc_sample_cnt += acc_fifo_sample_cnt;
                    acc_fifo_sample_cnt = 0;
                }
            }
        }
    } else {
#ifdef ACC_INTERRUPT
        // Send value if connected and notifying
        if (acc_data_enabled == 1 && acc_ptr->conn_handle != BLE_CONN_HANDLE_INVALID) {
            uint16_t               hvx_len;
            ble_gatts_hvx_params_t hvx_params;

            hvx_len = 1;

            memset(&hvx_params, 0, sizeof(hvx_params));
            acc_data[0] = acc_status;

            hvx_params.handle = acc_ptr->acc_data_handle.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = &hvx_len;
            hvx_params.p_data = acc_data;

            sd_ble_gatts_hvx(acc_ptr->conn_handle, &hvx_params);
        }
#else
        acc_status = acc_service_spi_read(0x0B);
        if (acc_status & 0x40) {
        	uint16_t               hvx_len;
        	ble_gatts_hvx_params_t hvx_params;

        	hvx_len = 1;

        	memset(&hvx_params, 0, sizeof(hvx_params));
        	acc_data[0] = acc_status;

        	hvx_params.handle = acc_ptr->acc_data_handle.value_handle;
        	hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        	hvx_params.offset = 0;
        	hvx_params.p_len  = &hvx_len;
        	hvx_params.p_data = acc_data;

        	sd_ble_gatts_hvx(acc_ptr->conn_handle, &hvx_params);
        }
#endif
    }
}

void acc_init_timer_io_spi(void)
{
	spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;
#ifdef ACC_INTERRUPT
    uint32_t int1_low_to_high_bitmask = 0x00000100; // Bitmask to be notified of transition from low to high for GPIO 8
    uint32_t high_to_low_bitmask = 0x00000000; // don't care
#endif

    //Configure SPI master.
	spi_config.SPI_Freq = SPI_FREQUENCY_FREQUENCY_K125;
    spi_config.SPI_Pin_SCK = 13;
    spi_config.SPI_Pin_MISO = 11;
    spi_config.SPI_Pin_MOSI = 12;
    spi_config.SPI_Pin_SS = 10;
    spi_config.SPI_CONFIG_ORDER = SPI_CONFIG_ORDER_MsbFirst;
    spi_config.SPI_CONFIG_CPOL = SPI_CONFIG_CPOL_ActiveHigh;
    spi_config.SPI_CONFIG_CPHA = SPI_CONFIG_CPHA_Leading;
    //spi_config.SPI_Freq = SPI_FREQUENCY_FREQUENCY_K125;

    //Initialize SPI master.
    APP_ERROR_CHECK(spi_master_open(SPI_MASTER_1, &spi_config));


    // Create ACC timer.
    APP_ERROR_CHECK(app_timer_create(&m_acc_timer_id,
                                    APP_TIMER_MODE_REPEATED,
                                    acc_timeout_handler));

    if (acc_service_spi_read(0x00) != 0xAD) {
    	persistent_record_error(PERSISTENT_ERROR_ADXL362, 0);
    	return;
    }

    acc_service_spi_write(0x2D, 0x00);
    if (acc_working_mode == 0) {
        acc_service_spi_write(0x28, 0x02);
        acc_service_spi_write(0x2C, 0x91);
        acc_service_spi_write(0x2D, 0x22);

        APP_ERROR_CHECK(app_timer_start(m_acc_timer_id, APP_TIMER_TICKS(200, 0), NULL));
    } else {
        acc_service_spi_write(0x20, 0x96);  // ACT threshold: 150 mg
        acc_service_spi_write(0x21, 0x00);
        acc_service_spi_write(0x23, 0x96);  // INACT threshold: 150 mg
        acc_service_spi_write(0x24, 0x00);
        acc_service_spi_write(0x25, 0x3C);  // INACT timeout: 10 sec
        acc_service_spi_write(0x26, 0x00);
        acc_service_spi_write(0x27, 0x3F);
        acc_service_spi_write(0x2A, 0x30);
        acc_service_spi_write(0x2D, 0x0A);

#ifdef ACC_INTERRUPT
        nrf_gpio_pin_dir_set(8, NRF_GPIO_PIN_DIR_INPUT);

        APP_ERROR_CHECK(app_gpiote_user_register(&m_gpio_uid, int1_low_to_high_bitmask, high_to_low_bitmask, int1_gpiote_event_handler));
        APP_ERROR_CHECK(app_gpiote_user_enable(m_gpio_uid));
#else
        APP_ERROR_CHECK(app_timer_start(m_acc_timer_id, APP_TIMER_TICKS(5000, 0), NULL));
#endif
    }
}

