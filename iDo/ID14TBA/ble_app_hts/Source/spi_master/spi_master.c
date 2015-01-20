/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

#include <string.h>
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "nrf51_bitfields.h"
#include "spi_master.h"
#include "nrf_delay.h"

#define SPI_USE_HARDWARE

#ifdef SPI_USE_HARDWARE

typedef struct
{
	NRF_SPI_Type * p_nrf_spi;   /**< A pointer to the NRF SPI master */
	IRQn_Type irq_type;         /**< A type of NVIC IRQn */

	uint8_t * p_tx_buffer;      /**< A pointer to TX buffer. */
	uint16_t tx_length;         /**< A length of TX buffer. */
	uint16_t tx_index;          /**< A index of the current element in the TX buffer. */

	uint8_t * p_rx_buffer;      /**< A pointer to RX buffer. */
	uint16_t rx_length;         /**< A length RX buffer. */
	uint16_t rx_index;          /**< A index of the current element in the RX buffer. */

	uint16_t max_length;        /**< Max length (Max of the TX and RX length). */
	uint16_t bytes_count;
	uint8_t pin_slave_select;   /**< A pin for Slave Select. */

	spi_master_event_handler_t callback_event_handler;  /**< A handler for event callback function. */

	spi_master_state_t state;   /**< A state of an instance of SPI master. */
	bool started_flag;
	bool disable_all_irq;
} spi_master_instance_t;

#define _static static

_static volatile spi_master_instance_t m_spi_master_instances[SPI_MASTER_HW_ENABLED_COUNT];

/* Function prototypes */
static __INLINE volatile spi_master_instance_t * spi_master_get_instance(
		const spi_master_hw_instance_t spi_master_hw_instance);

/**
 * @breif Function for getting an instance of SPI master.
 */
static __INLINE volatile spi_master_instance_t * spi_master_get_instance(
		const spi_master_hw_instance_t spi_master_hw_instance)
{
	if (spi_master_hw_instance < SPI_MASTER_HW_ENABLED_COUNT)
	{
		return &(m_spi_master_instances[(uint8_t)spi_master_hw_instance]);
	}
	return NULL;
}

/**
 * @brief Function for initializing instance of SPI master by default values.
 */
static __INLINE void spi_master_init_hw_instance(NRF_SPI_Type *p_nrf_spi,
				volatile spi_master_instance_t *p_spi_instance)
{
	APP_ERROR_CHECK_BOOL(p_spi_instance != NULL);

	memset((void *)p_spi_instance, 0, sizeof(spi_master_instance_t));

	p_spi_instance->p_nrf_spi = p_nrf_spi;
	p_spi_instance->state           = SPI_MASTER_STATE_DISABLED;
}

uint32_t spi_master_open(const spi_master_hw_instance_t    spi_master_hw_instance,
		spi_master_config_t const * const p_spi_master_config)
{
	uint8_t idx;

	/*
	 * Hardware SPI
	 */
	/* Check against null */
	if (p_spi_master_config == NULL)
	{
		return NRF_ERROR_NULL;
	}

	volatile spi_master_instance_t * p_spi_instance = spi_master_get_instance(
			spi_master_hw_instance);
	APP_ERROR_CHECK_BOOL(p_spi_instance != NULL);

	switch (spi_master_hw_instance)
	{
	case SPI_MASTER_0:
		spi_master_init_hw_instance(NRF_SPI0, p_spi_instance);
		break;

	case SPI_MASTER_1:
		spi_master_init_hw_instance(NRF_SPI1, p_spi_instance);
		break;

	default:
		break;
	}

	//A Slave select must be set as high before setting it as output,
	//because during connect it to the pin it causes glitches.
	nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_SS);
	nrf_gpio_cfg_output(p_spi_master_config->SPI_Pin_SS);
	nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_SS);

	//Configure GPIO
	//Clear SCK
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_SCK);
	nrf_gpio_cfg_output(p_spi_master_config->SPI_Pin_SCK);
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_SCK);

	//Clear MOSI
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_MOSI);
	nrf_gpio_cfg_output(p_spi_master_config->SPI_Pin_MOSI);
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_MOSI);

	// Perform ADT7320 interface reset before moving on
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_SS);
	for (idx = 0; idx < 34; idx++) {
		nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_SCK);
		nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_MOSI);
		nrf_delay_us(50);
		nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_SCK);
		nrf_delay_us(50);
	}
	nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_SS);
	// reset end

	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_SCK);
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_MOSI);
	nrf_gpio_cfg_input(p_spi_master_config->SPI_Pin_MISO, NRF_GPIO_PIN_NOPULL);
	p_spi_instance->pin_slave_select = p_spi_master_config->SPI_Pin_SS;

	/* Configure SPI hardware */
	p_spi_instance->p_nrf_spi->PSELSCK  = p_spi_master_config->SPI_Pin_SCK;
	p_spi_instance->p_nrf_spi->PSELMOSI = p_spi_master_config->SPI_Pin_MOSI;
	p_spi_instance->p_nrf_spi->PSELMISO = p_spi_master_config->SPI_Pin_MISO;

	p_spi_instance->p_nrf_spi->FREQUENCY = p_spi_master_config->SPI_Freq;

	p_spi_instance->p_nrf_spi->CONFIG =
			(uint32_t)(p_spi_master_config->SPI_CONFIG_CPHA << SPI_CONFIG_CPHA_Pos) |
			(p_spi_master_config->SPI_CONFIG_CPOL << SPI_CONFIG_CPOL_Pos) |
			(p_spi_master_config->SPI_CONFIG_ORDER << SPI_CONFIG_ORDER_Pos);

	/* Clear waiting interrupts and events */
	p_spi_instance->p_nrf_spi->EVENTS_READY = 0;

	//APP_ERROR_CHECK(sd_nvic_ClearPendingIRQ(p_spi_instance->irq_type));
	//APP_ERROR_CHECK(sd_nvic_SetPriority(p_spi_instance->irq_type, p_spi_master_config->SPI_PriorityIRQ));

	/* Clear event handler */
	p_spi_instance->callback_event_handler = NULL;

	/* Enable interrupt */
	//p_spi_instance->p_nrf_spi->INTENSET = (SPI_INTENSET_READY_Set << SPI_INTENCLR_READY_Pos);
	//APP_ERROR_CHECK(sd_nvic_EnableIRQ(p_spi_instance->irq_type));

	/* Enable SPI hardware */
	p_spi_instance->p_nrf_spi->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);

	/* Change state to IDLE */
	p_spi_instance->state = SPI_MASTER_STATE_IDLE;

	return NRF_SUCCESS;
}

void spi_master_close(const spi_master_hw_instance_t spi_master_hw_instance)
{
	volatile spi_master_instance_t * p_spi_instance = spi_master_get_instance(
			spi_master_hw_instance);
	APP_ERROR_CHECK_BOOL(p_spi_instance != NULL);

	/* Disable interrupt */
	//APP_ERROR_CHECK(sd_nvic_ClearPendingIRQ(p_spi_instance->irq_type));
	//APP_ERROR_CHECK(sd_nvic_DisableIRQ(p_spi_instance->irq_type));

	p_spi_instance->p_nrf_spi->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);

	/* Disconnect pin slave select */
	nrf_gpio_pin_clear(p_spi_instance->pin_slave_select);
	p_spi_instance->pin_slave_select = (uint8_t)0xFF;

	/* Disconnect pins from SPI hardware */
	p_spi_instance->p_nrf_spi->PSELSCK  = (uint32_t)SPI_PIN_DISCONNECTED;
	p_spi_instance->p_nrf_spi->PSELMOSI = (uint32_t)SPI_PIN_DISCONNECTED;
	p_spi_instance->p_nrf_spi->PSELMISO = (uint32_t)SPI_PIN_DISCONNECTED;

	/* Reset to default values */
	spi_master_init_hw_instance(NULL, p_spi_instance);
}

uint32_t spi_master_send_recv(const spi_master_hw_instance_t spi_master_hw_instance,
		uint8_t * p_tx_buf, uint8_t * p_rx_buf, const uint16_t buf_len)
{
	volatile spi_master_instance_t * p_spi_instance = spi_master_get_instance(
			spi_master_hw_instance);
	APP_ERROR_CHECK_BOOL(p_spi_instance != NULL);

	uint32_t err_code = NRF_SUCCESS;
	uint32_t idx;

	if (p_spi_instance->state != SPI_MASTER_STATE_IDLE) {
		return NRF_ERROR_BUSY;
	}

	p_spi_instance->state = SPI_MASTER_STATE_BUSY;

	nrf_gpio_pin_clear(p_spi_instance->pin_slave_select);
	nrf_delay_us(10);

	for (idx = 0; idx < buf_len; idx++) {
		p_spi_instance->p_nrf_spi->TXD = p_tx_buf[idx];
		while (p_spi_instance->p_nrf_spi->EVENTS_READY != 1);
		p_rx_buf[idx] = p_spi_instance->p_nrf_spi->RXD;
		p_spi_instance->p_nrf_spi->EVENTS_READY = 0;
	}

	p_spi_instance->state = SPI_MASTER_STATE_IDLE;
	nrf_gpio_pin_set(p_spi_instance->pin_slave_select);

	return err_code;
}

#else

#define SS		18
#define SCK		15
#define MOSI	17
#define MISO	14

uint32_t spi_master_open(const spi_master_hw_instance_t    spi_master_hw_instance,
		spi_master_config_t const * const p_spi_master_config)
{
	uint8_t idx = 0;

	//A Slave select must be set as high before setting it as output,
	//because during connect it to the pin it causes glitches.
	nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_SS);
	nrf_gpio_cfg_output(p_spi_master_config->SPI_Pin_SS);
	nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_SS);

	//Configure GPIO
	//Clear SCK
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_SCK);
	nrf_gpio_cfg_output(p_spi_master_config->SPI_Pin_SCK);
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_SCK);

	//Clear MOSI
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_MOSI);
	nrf_gpio_cfg_output(p_spi_master_config->SPI_Pin_MOSI);
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_MOSI);

	// Perform ADT7320 interface reset before moving on
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_SS);
	for (idx = 0; idx < 34; idx++) {
		nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_SCK);
		nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_MOSI);
		nrf_delay_us(50);
		nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_SCK);
		nrf_delay_us(50);
	}
	nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_SS);
	// reset end

	nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_SS);
	nrf_gpio_pin_set(p_spi_master_config->SPI_Pin_SCK);
	nrf_gpio_pin_clear(p_spi_master_config->SPI_Pin_MOSI);
	nrf_gpio_cfg_input(p_spi_master_config->SPI_Pin_MISO, NRF_GPIO_PIN_NOPULL);

	return NRF_SUCCESS;
}

uint32_t spi_master_send_recv(const spi_master_hw_instance_t spi_master_hw_instance,
		uint8_t * p_tx_buf, const uint16_t tx_buf_len,
		uint8_t * p_rx_buf, const uint16_t rx_buf_len)
{
	uint16_t idx = 0;
	uint8_t i = 0, inData = 0;

	nrf_gpio_pin_set(SS);
	nrf_gpio_cfg_output(SS);
	nrf_gpio_pin_set(SS);

	//Configure GPIO
	//Clear SCK
	nrf_gpio_pin_set(SCK);
	nrf_gpio_cfg_output(SCK);
	nrf_gpio_pin_set(SCK);

	//Clear MOSI
	nrf_gpio_pin_clear(MOSI);
	nrf_gpio_cfg_output(MOSI);
	nrf_gpio_pin_clear(MOSI);

	nrf_gpio_cfg_input(MISO, NRF_GPIO_PIN_NOPULL);

	nrf_gpio_pin_clear(SS);
	nrf_delay_us(50);

	for (idx = 0; idx < tx_buf_len; idx++) {
		inData = 0;
		for(i=0; i<8; i++)
		{
			nrf_gpio_pin_clear(SCK);
			if ((p_tx_buf[idx]<<i) & 0x80) {
				nrf_gpio_pin_set(MOSI);
			} else {
				nrf_gpio_pin_clear(MOSI);
			}
			nrf_delay_us(200);
			nrf_gpio_pin_set(SCK);
			inData <<= 1;
			inData |= nrf_gpio_pin_read(MISO);
			nrf_delay_us(200);
		}
		p_rx_buf[idx] = inData;
	}

	nrf_delay_us(50);
	nrf_gpio_pin_set(SS);

	return NRF_SUCCESS;
}

#endif
