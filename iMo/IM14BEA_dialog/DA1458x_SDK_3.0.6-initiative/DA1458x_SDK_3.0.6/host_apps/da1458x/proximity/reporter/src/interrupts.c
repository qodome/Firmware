//------------------------------------------------------------------------------
// (c) Copyright 2012, Dialog Semiconductor BV
// All Rights Reserved
//------------------------------------------------------------------------------
// Dialog SEMICONDUCTOR CONFIDENTIAL
//------------------------------------------------------------------------------
// This code includes Confidential, Proprietary Information and is a Trade 
// Secret of Dialog Semiconductor BV. All use, disclosure, and/or reproduction 
// is prohibited unless authorized in writing.
//------------------------------------------------------------------------------
// Description: Interrupt Handlers

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "global_io.h"
#include "gpio.h"
#include "peripherals.h"
#include "app_button_led.h"
#include "ble_msg.h"
#include "spi_hci_msg.h"
#include <core_cm0.h>

// Receive buffer
extern uint8_t rd_data[256];


// DREADY high - Slave request to send interrupt
void GPIO0_Handler(void)
{
	NVIC_DisableIRQ(GPIO0_IRQn);
	
	spi_receive_hci_msg(rd_data);
	SetWord16(GPIO_RESET_IRQ_REG,1);  // Reset GPIO0 interrupt
	
	NVIC_ClearPendingIRQ(GPIO0_IRQn);	// Clear interrupt requests while disabled
	NVIC_EnableIRQ(GPIO0_IRQn);				// Enable this interrupt
} 

// P11 low - Button pressed interrupt
void GPIO1_Handler(void)
{
	NVIC_DisableIRQ(GPIO1_IRQn);			// Disable this interrupt
	
	SetWord16(GPIO_RESET_IRQ_REG, 2);
	turn_off_led();
	
	NVIC_ClearPendingIRQ(GPIO1_IRQn);	// Clear interrupt requests while disabled
	NVIC_EnableIRQ(GPIO1_IRQn);       // Enable this interrupt
}

void HardFault_Handler(void) 
{
    while(1);
}
