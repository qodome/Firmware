/*
 * Application logic to talk with sensors via I2C
 */

#ifndef __SENSOR_SERVICE__
#define __SENSOR_SERVICE__

#include "nrf_types.h"
#include <stdbool.h>

void sensor_service_init(void);

bool ss_isl_read(uint8_t reg, uint8_t *cBuf);
bool ss_isl_write(uint8_t reg, uint8_t val);

void ss_isl_start(void);
void ss_isl_stop(void);

#endif
