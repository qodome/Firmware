/**
 ****************************************************************************************
 *
 * @file periph_setup.c
 *
 * @brief Peripherals setup and initialization. 
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
#include "rwip_config.h"             // SW configuration
#include "periph_setup.h"            // peripheral configuration
#include "global_io.h"
#include "gpio.h"
#include "uart.h"                    // UART initialization


/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 *
 * @return void
 ****************************************************************************************
 */

#if DEVELOPMENT_DEBUG

void GPIO_reservations(void)
{
#if BLE_INTEGRATED_HOST_GTL
    // UART
    RESERVE_GPIO( UART1_TX, UART1_TX_GPIO_PORT,  UART1_TX_GPIO_PIN, PID_UART1_TX);
    RESERVE_GPIO( UART1_RX, UART1_RX_GPIO_PORT,  UART1_RX_GPIO_PIN, PID_UART1_RX);    

    RESERVE_GPIO( UART1_RTS, UART1_RTSN_GPIO_PORT, UART1_RTSN_GPIO_PIN, PID_UART1_RTSN);
    RESERVE_GPIO( UART1_CTS, UART1_CTSN_GPIO_PORT, UART1_CTSN_GPIO_PIN, PID_UART1_CTSN);    
#endif
#ifdef CFG_WKUP_EXT_PROCESSOR		
    // external MCU wakeup GPIO
    RESERVE_GPIO( EXT_WAKEUP_GPIO, EXT_WAKEUP_PORT, EXT_WAKEUP_PIN, PID_GPIO );    
#endif
}
#endif //DEVELOPMENT__NO_OTP

/**
 ****************************************************************************************
 * @brief Map port pins
 *
 * The Uart and SPI port pins and GPIO ports are mapped
 ****************************************************************************************
 */
void set_pad_functions(void)        // set gpio port function mode
{
#if BLE_INTEGRATED_HOST_GTL
    GPIO_ConfigurePin( UART1_TX_GPIO_PORT, UART1_TX_GPIO_PIN, OUTPUT, PID_UART1_TX, false );
    GPIO_ConfigurePin( UART1_RX_GPIO_PORT, UART1_RX_GPIO_PIN, INPUT, PID_UART1_RX, false );    
	
    GPIO_ConfigurePin( UART1_RTSN_GPIO_PORT, UART1_RTSN_GPIO_PIN, OUTPUT, PID_UART1_RTSN, false );
    GPIO_ConfigurePin( UART1_CTSN_GPIO_PORT, UART1_CTSN_GPIO_PIN, INPUT, PID_UART1_CTSN, false );    
#endif 	
#ifdef CFG_WKUP_EXT_PROCESSOR	
    // external MCU wakeup GPIO
    GPIO_ConfigurePin( EXT_WAKEUP_PORT, EXT_WAKEUP_PIN, OUTPUT, PID_GPIO, false ); // initialized to low
#endif	
}


/**
 ****************************************************************************************
 * @brief Enable pad's and peripheral clocks assuming that peripherals' power domain is down. The Uart and SPi clocks are set.
 *
 * @return void
 ****************************************************************************************
 */
void periph_init(void) 
{
	// Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP)) ; 
    
#if ES4_CODE
    SetBits16(CLK_16M_REG,XTAL16_BIAS_SH_DISABLE, 1);
#endif    

#if BLE_INTEGRATED_HOST_GTL	
    // Initialize UART component

    SetBits16(CLK_PER_REG, UART1_ENABLE, 1);    // enable clock - always @16MHz
	
    // baudr=9-> 115k2
    // mode=3-> no parity, 1 stop bit 8 data length
    uart_init(UART_BAUDRATE_115K2, 3);
#endif    

	//rom patch
	patch_func();
	
	//Init pads
	set_pad_functions();

#if (BLE_APP_PRESENT)
/*
* (Re)Initialize peripherals
i.e.    
    uart_init(UART_BAUDRATE_115K2, 3);
*/        
#endif
    
    // Enable the pads
	SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
}
