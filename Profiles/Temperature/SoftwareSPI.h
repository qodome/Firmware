/*
 * Software SPI interface to work with pressure sensor
 */
#ifndef __SOFTWARE_SPI__
#define __SOFTWARE_SPI__

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"

#define CS_HIGH()   (P0_2 = 1)
#define CS_LOW()    (P0_2 = 0)
#define SCK_HIGH()  (P0_5 = 1)
#define SCK_LOW()   (P0_5 = 0)
#define MO_HIGH()  (P0_3 = 1)
#define MO_LOW()   (P0_3 = 0)

#define READ_MI() ((P0 & BV(4)) >> 4)

void softwareSPIInit (void);

uint16 spi_read (uint8 addr);

void spi_write (uint8 addr, uint16 val);

#endif
