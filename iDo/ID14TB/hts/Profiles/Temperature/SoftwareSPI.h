/*
 * Software SPI interface to work with pressure sensor
 */
#ifndef __SOFTWARE_SPI__
#define __SOFTWARE_SPI__

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"

void softwareSPIInit (void);

uint16 spi_read (uint8 addr);

void spi_write (uint8 addr, uint16 val);

void SPI_intf_reset (void);

#endif
