#include <string.h>
#include "intermcu_spi.h"
#include "spi_slave.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "nrf_delay.h"

// SPI pin configurations
#define SPIS_MISO_PIN       7
#define SPIS_MOSI_PIN       6
#define SPIS_SCK_PIN        5
#define SPIS_CSN_PIN        2

#define MAX_BUF_SIZE        20
#define TX_BUF_SIZE         MAX_BUF_SIZE        /**< SPI TX buffer size. */      
#define RX_BUF_SIZE         MAX_BUF_SIZE        /**< SPI RX buffer size. */      
#define DEF_CHARACTER       0xAAu               /**< SPI default character. Character clocked out in case of an ignored transaction. */      
#define ORC_CHARACTER       0x55u               /**< SPI over-read character. Character clocked out after an over-read of the transmit buffer. */      

#define NRF51_TO_RT5350_MAIL_IO				4
#define RT5350_TO_NRF51_MAIL_IO				3
#define RT5350_RESET_CONTROL_IO				1

#define NRF_CMD_POLL			0x00
#define NRF_CMD_SEND_IP			0x01

enum intermcu_state {
	IO_INVALID = 0,
	IO_INIT,
	IO_UP1,
	IO_DOWN1,
	IO_UP2,
	IO_CONNECTED,
	IO_HEADER_IN_PROGRESS,
	IO_HEADER_RECEIVED,
	IO_WAIT_FOR_PAYLOAD,
	IO_PAYLOAD_IN_PROGRESS,
	IO_PAYLOAD_RECEIVED,
	IO_WAIT_PEER_IDLE,
};

struct intermcu_header {
	uint8_t type;
	uint8_t len;
};

static uint8_t m_tx_buf[TX_BUF_SIZE];   /**< SPI TX buffer. */      
static uint8_t m_rx_buf[RX_BUF_SIZE];   /**< SPI RX buffer. */          
static intermcu_spi_recv_cb intermcu_recv_cb = NULL;

static app_timer_id_t intermcu_timer;
enum intermcu_state intermcu_init_state = IO_INVALID;
uint32_t init_state_cnt = 0;
struct intermcu_header magic_header;

uint16_t intermcu_abnormal_cnt1 = 0;
uint16_t intermcu_abnormal_cnt2 = 0;

static uint8_t intermcu_notify_type = NRF_IDLE_NOTIFY;
static uint8_t intermcu_notify_len = 0;
static uint8_t intermcu_notify_buf[19] = {0};

uint8_t ip_addr_received = 0;
uint8_t ip[15] = {0};

/*
 * FIXME: run callback in background thread!
 */
static void spi_slave_event_handle(spi_slave_evt_t event)
{
    if (event.evt_type == SPI_SLAVE_XFER_DONE)
    {
        if (intermcu_init_state == IO_HEADER_IN_PROGRESS) {
        	intermcu_init_state = IO_HEADER_RECEIVED;
        } else if (intermcu_init_state == IO_PAYLOAD_IN_PROGRESS) {
        	intermcu_init_state = IO_PAYLOAD_RECEIVED;
        } else {
        	intermcu_abnormal_cnt1++;
        	intermcu_init_state = IO_INVALID;
        	nrf_gpio_pin_clear(NRF51_TO_RT5350_MAIL_IO);
        	init_state_cnt = 0;
        }
    }
}

void intermcu_notify(uint8_t type, uint8_t *buf, uint8_t len)
{
	intermcu_notify_type = type;
	intermcu_notify_len = len;
	for (uint8_t idx = 0; idx < 19; idx++) {
		intermcu_notify_buf[idx] = 0;
	}
	memcpy(intermcu_notify_buf, buf, len);
}

// Initialize IO
static void rino_init(void)
{
	nrf_gpio_pin_clear(NRF51_TO_RT5350_MAIL_IO);
	nrf_gpio_cfg_output(NRF51_TO_RT5350_MAIL_IO);
	nrf_gpio_pin_clear(NRF51_TO_RT5350_MAIL_IO);
}

static void roni_init(void)
{
	nrf_gpio_cfg_input(RT5350_TO_NRF51_MAIL_IO, NRF_GPIO_PIN_PULLDOWN);
	// To avoid IO collision, configure RINO as input and wait
	// until RT5350 signal us
	nrf_gpio_cfg_input(NRF51_TO_RT5350_MAIL_IO, NRF_GPIO_PIN_PULLDOWN);
}

void trigger_rt5350_reset(void)
{
	nrf_gpio_pin_clear(RT5350_RESET_CONTROL_IO);
	nrf_delay_us(5000);
	nrf_gpio_pin_set(RT5350_RESET_CONTROL_IO);
}

static void rt5350_reset_ctrl_init(void)
{
	nrf_gpio_pin_set(RT5350_RESET_CONTROL_IO);
	nrf_gpio_cfg_output(RT5350_RESET_CONTROL_IO);
	nrf_gpio_pin_set(RT5350_RESET_CONTROL_IO);
}

// When timer time happens, toggle GPIO to raise interrupt
static void intermcu_timeout_handler(void * p_context)
{
	uint8_t buf[20];
	uint8_t idx;

	if (intermcu_init_state < IO_CONNECTED) {
		if (intermcu_init_state == IO_INVALID) {
			if (nrf_gpio_pin_read(RT5350_TO_NRF51_MAIL_IO) == 0) {
				intermcu_init_state = IO_INIT;
			}
		} else if (intermcu_init_state == IO_INIT) {
			if (nrf_gpio_pin_read(RT5350_TO_NRF51_MAIL_IO) == 1) {
				intermcu_init_state = IO_UP1;
				init_state_cnt = 0;
			}
		} else if (intermcu_init_state == IO_UP1) {
			if (nrf_gpio_pin_read(RT5350_TO_NRF51_MAIL_IO) == 1) {
				init_state_cnt++;
			} else {
				if ((init_state_cnt >= 5) && (init_state_cnt <= 20)) {
					intermcu_init_state = IO_DOWN1;
				} else {
					intermcu_init_state = IO_INIT;
				}
				init_state_cnt = 0;
			}
		} else if (intermcu_init_state == IO_DOWN1) {
			if (nrf_gpio_pin_read(RT5350_TO_NRF51_MAIL_IO) == 0) {
				init_state_cnt++;
			} else {
				if ((init_state_cnt >= 5) && (init_state_cnt <= 20)) {
					intermcu_init_state = IO_UP2;
				} else {
					intermcu_init_state = IO_INIT;
				}
				init_state_cnt = 0;
			}
		} else if (intermcu_init_state == IO_UP2) {
			if (nrf_gpio_pin_read(RT5350_TO_NRF51_MAIL_IO) == 1) {
				init_state_cnt++;
			} else {
				if ((init_state_cnt >= 5) && (init_state_cnt <= 20)) {
					intermcu_init_state = IO_CONNECTED;
					rino_init();
				} else {
					intermcu_init_state = IO_INIT;
				}
				init_state_cnt = 0;
			}
		} else {
			intermcu_init_state = IO_INVALID;
			init_state_cnt = 0;
		}
	} else {
		// Handle commands from RT5350
		if (intermcu_init_state == IO_CONNECTED) {
			if (nrf_gpio_pin_read(RT5350_TO_NRF51_MAIL_IO) == 1) {
				// Prepare SPI transaction
				APP_ERROR_CHECK(intermcu_spi_transaction(m_tx_buf, 2));

				// Setup flag to tell RT5350 we are ready!
				nrf_gpio_pin_set(NRF51_TO_RT5350_MAIL_IO);

				intermcu_init_state = IO_HEADER_IN_PROGRESS;

			}
		} else if (intermcu_init_state == IO_HEADER_RECEIVED) {
			nrf_gpio_pin_clear(NRF51_TO_RT5350_MAIL_IO);
			magic_header.type = m_rx_buf[0];
			magic_header.len = m_rx_buf[1];
			intermcu_init_state = IO_WAIT_FOR_PAYLOAD;
		} else if (intermcu_init_state == IO_WAIT_FOR_PAYLOAD) {
			if (nrf_gpio_pin_read(RT5350_TO_NRF51_MAIL_IO) == 1) {
				// Check command type
				if (magic_header.type == NRF_CMD_POLL) {
					buf[0] = intermcu_notify_type;
					intermcu_notify_type = NRF_IDLE_NOTIFY;
					for (uint8_t idx = 0; idx < 19; idx++) {
						buf[1 + idx] = intermcu_notify_buf[idx];
						intermcu_notify_buf[idx] = 0;
					}
					intermcu_notify_len = 0;
					APP_ERROR_CHECK(intermcu_spi_transaction(buf, magic_header.len));
				} else {
					// FIXME: RT5350 has command sent to us
					APP_ERROR_CHECK(intermcu_spi_transaction(buf, magic_header.len));
				}

				// Setup flag to tell RT5350 we are ready!
				nrf_gpio_pin_set(NRF51_TO_RT5350_MAIL_IO);

				intermcu_init_state = IO_PAYLOAD_IN_PROGRESS;
			}
		} else if (intermcu_init_state == IO_PAYLOAD_RECEIVED) {
			if (magic_header.type == NRF_CMD_SEND_IP) {
				ip_addr_received = 1;
				for (idx = 0; idx < magic_header.len; idx++) {
					ip[idx] = m_rx_buf[idx];
				}
			}
	        if (intermcu_recv_cb != NULL) {
	            (*intermcu_recv_cb)(magic_header.type, magic_header.len, m_rx_buf);
	        }
			nrf_gpio_pin_clear(NRF51_TO_RT5350_MAIL_IO);
			intermcu_init_state = IO_WAIT_PEER_IDLE;
			init_state_cnt = 0;
		} else if (intermcu_init_state == IO_WAIT_PEER_IDLE) {
			if (nrf_gpio_pin_read(RT5350_TO_NRF51_MAIL_IO) == 0) {
				intermcu_init_state = IO_CONNECTED;
				init_state_cnt = 0;
			} else {
				init_state_cnt++;
				if (init_state_cnt > 20) {
		        	intermcu_abnormal_cnt2++;
		        	intermcu_init_state = IO_INVALID;
		        	nrf_gpio_pin_clear(NRF51_TO_RT5350_MAIL_IO);
		        	init_state_cnt = 0;
				}
			}
		}
	}
}

static void intermcu_spi_init(void)
{
    uint32_t           err_code;
    spi_slave_config_t spi_slave_config;
        
    err_code = spi_slave_evt_handler_register(spi_slave_event_handle);
    APP_ERROR_CHECK(err_code);    

    spi_slave_config.pin_miso         = SPIS_MISO_PIN;
    spi_slave_config.pin_mosi         = SPIS_MOSI_PIN;
    spi_slave_config.pin_sck          = SPIS_SCK_PIN;
    spi_slave_config.pin_csn          = SPIS_CSN_PIN;
    spi_slave_config.mode             = SPI_MODE_1;
    spi_slave_config.bit_order        = SPIM_MSB_FIRST;
    spi_slave_config.def_tx_character = DEF_CHARACTER;
    spi_slave_config.orc_tx_character = ORC_CHARACTER;
    
    err_code = spi_slave_init(&spi_slave_config);
    APP_ERROR_CHECK(err_code);

    return; 
}

/*
 * Initialize inter-MCU communication interface
 */
void intermcu_init(intermcu_spi_recv_cb spi_recv_cb)
{
	// Enable RT5350
	rt5350_reset_ctrl_init();

	trigger_rt5350_reset();

	// SPI interface
	intermcu_spi_init();

	// MAIL IO, configured both to input
	roni_init();

	// Timer
    APP_ERROR_CHECK(app_timer_create(&intermcu_timer, APP_TIMER_MODE_REPEATED, intermcu_timeout_handler));
    APP_ERROR_CHECK(app_timer_start(intermcu_timer, APP_TIMER_TICKS(100, 0), NULL));

    intermcu_recv_cb = spi_recv_cb;
}

/*
 * Do one bulk read/write SPI operations
 */
uint32_t intermcu_spi_transaction(uint8_t *tx_buffer, uint16_t tx_len)
{
    uint16_t idx;

    if (tx_len > MAX_BUF_SIZE) {
        return NRF_ERROR_INVALID_PARAM;
    }

    for (idx = 0; idx < tx_len; idx++) {
        m_tx_buf[idx] = tx_buffer[idx];
    }

    //Set buffers.
    return spi_slave_buffers_set(m_tx_buf, m_rx_buf, tx_len, tx_len);
}

uint8_t intermcu_get_wifi_status(uint8_t *ip_addr_buffer)
{
	uint8_t idx;

	if (intermcu_init_state < IO_CONNECTED) {
		return 0;
	} else {
		if (ip_addr_received > 0) {
			// Fillin IP address
			for (idx = 0; idx < 15; idx++) {
				ip_addr_buffer[idx] = ip[idx];
			}
			return 2;
		} else {
			return 1;
		}
	}
}
