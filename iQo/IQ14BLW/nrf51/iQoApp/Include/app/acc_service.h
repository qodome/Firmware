/*
 * ADXL362 driver
 */

#ifndef __ACC_SERVICE__
#define __ACC_SERVICE__

#include "ble_acc.h"

void acc_service_start(void);
void acc_service_stop(void);
uint8_t acc_service_spi_read (uint8_t addr);
void acc_service_spi_write (uint8_t addr, uint8_t cmd);
void acc_init_timer_io_spi(void);

#endif /* __ACC_SERVICE__ */
