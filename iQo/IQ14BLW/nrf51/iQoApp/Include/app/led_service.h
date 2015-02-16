/*
 * LED service
 */

#ifndef __LED_SERVICE__
#define __LED_SERVICE__

#include "nrf_types.h"

void led_service_init(void);
void led_set_light(uint8_t idx, uint8_t target);

#endif
