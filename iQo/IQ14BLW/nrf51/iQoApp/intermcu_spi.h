/*
 * The SPI interface between nRF51822 and RT5350
 */
#ifndef __INTERMCU_SPI__
#define __INTERMCU_SPI__

#include <stdint.h>
#include <stdbool.h>

typedef void (*intermcu_spi_recv_cb)(uint8_t type, uint8_t len, uint8_t *buf);

/*
 * Initialize inter-MCU SPI interface
 */
void intermcu_init(intermcu_spi_recv_cb spi_recv_cb);

/*
 * Do one bulk read/write SPI operations
 */
uint32_t intermcu_spi_transaction(uint8_t *tx_buffer, uint16_t tx_len);

#endif
