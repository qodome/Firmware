/**
 ****************************************************************************************
 *
 * @file periph_setup.h
 *
 * @brief Peripherals setup header file. 
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "global_io.h"
#include "arch.h"

#include "i2c_eeprom.h"
/*
 * DEFINES
 ****************************************************************************************
 */

/****************************************************************************************/ 
/* i2c eeprom configuration                                                             */
/****************************************************************************************/ 
#define I2C_EEPROM_SIZE   0x20000         // EEPROM size in bytes
#define I2C_EEPROM_PAGE   256             // EEPROM's page size in bytes
#define I2C_SPEED_MODE    I2C_FAST        // 1: standard mode (100 kbits/s), 2: fast mode (400 kbits/s)
#define I2C_ADDRESS_MODE  I2C_7BIT_ADDR   // 0: 7-bit addressing, 1: 10-bit addressing
#define I2C_ADDRESS_SIZE  I2C_2BYTES_ADDR // 0: 8-bit memory address, 1: 16-bit memory address, 3: 24-bit memory address

/****************************************************************************************/ 
/* SPI FLASH configuration                                                             */
/****************************************************************************************/


// SPI Flash Manufacturer and ID
#define W25X10CL_MANF_DEV_ID (0xEF10)     // W25X10CL Manufacturer and ID
#define W25X20CL_MANF_DEV_ID (0xEF11)     // W25X10CL Manufacturer and ID

// SPI Flash options
#define W25X10CL_SIZE 131072              // SPI Flash memory size in bytes
#define W25X20CL_SIZE 262144              // SPI Flash memory size in bytes
#define W25X10CL_PAGE 256                 // SPI Flash memory page size in bytes
#define W25X20CL_PAGE 256                 // SPI Flash memory page size in bytes
#define SPI_FLASH_DEFAULT_SIZE  131072    // SPI Flash memory size in bytes
#define SPI_FLASH_DEFAULT_PAGE  256
#define SPI_SECTOR_SIZE         4096

/*
 * DEFINES
 ****************************************************************************************
 */

/* Enable WKUPCT. Required by wkupct_quadec driver. */
#define WKUP_ENABLED

#if defined(HW_CONFIG_BASIC_DK)
    #define GPIO_ALERT_LED_PORT     GPIO_PORT_1
    #define GPIO_ALERT_LED_PIN      GPIO_PIN_0

    #define USE_PUSH_BUTTON         0
    #define GPIO_BUTTON_PORT        GPIO_PORT_0
    #define GPIO_BUTTON_PIN         GPIO_PIN_6

    #define USE_BAT_LEVEL_ALERT     0
    #define GPIO_BAT_LED_PORT       GPIO_PORT_1
    #define GPIO_BAT_LED_PIN        GPIO_PIN_2
#elif defined(HW_CONFIG_PRO_DK)
    #define GPIO_ALERT_LED_PORT     GPIO_PORT_1
    #define GPIO_ALERT_LED_PIN      GPIO_PIN_0

    #define USE_PUSH_BUTTON         1
    #define GPIO_BUTTON_PORT        GPIO_PORT_1
    #define GPIO_BUTTON_PIN         GPIO_PIN_1

    #define USE_BAT_LEVEL_ALERT     0
    #define GPIO_BAT_LED_PORT       GPIO_PORT_1
    #define GPIO_BAT_LED_PIN        GPIO_PIN_2
#else
    #define GPIO_ALERT_LED_PORT     GPIO_PORT_0
    #define GPIO_ALERT_LED_PIN      GPIO_PIN_7

    #define USE_PUSH_BUTTON         1
    #define GPIO_BUTTON_PORT        GPIO_PORT_1
    #define GPIO_BUTTON_PIN         GPIO_PIN_1

    #define USE_BAT_LEVEL_ALERT     1
    #define GPIO_BAT_LED_PORT       GPIO_PORT_1
    #define GPIO_BAT_LED_PIN        GPIO_PIN_0
#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
void periph_init(void);

void GPIO_reservations(void);

