/*
 * Keep user persistent data in 0x3BC00 - 0x3C000
 */
#ifndef __PERSISTENT__
#define __PERSISTENT__

#include <stdint.h>
#include <stdlib.h>

// Max length of device name: 20 (excluding tailing null)
void persistent_set_dev_name(uint8_t *buf, uint16_t len);

// Caller provide dev name buffer, at least 21 bytes
void persistent_get_dev_name(uint8_t *buf);

#endif
