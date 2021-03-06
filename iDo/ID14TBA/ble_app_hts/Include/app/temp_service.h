/*
 * timer_scheduler.h
 *
 *  Created on: 2014-10-30
 *      Author: Administrator
 */

#ifndef TIMER_SCHEDULER_H_
#define TIMER_SCHEDULER_H_

#include "ble_ido.h"

void temp_init_timer_spi(void);

uint16_t temp_service_get_tm_interval();
void temp_service_set_tm_intreval(uint16_t interval);
void temp_tm_start(void);
void temp_it_start(void);
void temp_tm_stop(void);
void temp_it_stop(void);
void temp_measurement_confirm(void);

extern uint16_t spi_read (uint8_t addr);
extern void spi_write (uint8_t addr, uint16_t cmd);

void iDo_indicate_measurement_ready(void);

#endif /* TIMER_SCHEDULER_H_ */
