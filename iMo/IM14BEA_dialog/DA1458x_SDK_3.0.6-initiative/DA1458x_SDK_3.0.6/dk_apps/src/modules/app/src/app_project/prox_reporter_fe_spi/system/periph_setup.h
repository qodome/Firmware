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
 
#include "global_io.h"
#include "gpio.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// define UART pins
#define UART_GPIO_PORT  GPIO_PORT_0
#define UART_TX_PIN     GPIO_PIN_4
#define UART_RX_PIN     GPIO_PIN_5
#undef  UART_ENABLED

/// define SPI pins
#define SPI_GPIO_PORT   GPIO_PORT_0
#define SPI_CLK_PIN     GPIO_PIN_0
#define SPI_CS_PIN      GPIO_PIN_1
#define SPI_DO_PIN      GPIO_PIN_2
#define SPI_DI_PIN      GPIO_PIN_3
#define SPI_DREADY_PIN  GPIO_PIN_7

/****************************************************************************************/ 
/* External CPU to DA14580 wake-up pin selection                                     		*/
/****************************************************************************************/ 
#if (EXTERNAL_WAKEUP)
	/*Auto select the external GPIO wakeup signal according to the HCI_EIF_SELECT_PORT/HCI_EIF_SELECT_PIN configuration*/
	#define EIF_WAKEUP_GPIO												1
	#ifdef EIF_WAKEUP_GPIO
		#ifdef CFG_HCI_BOTH_EIF	
			#define EXTERNAL_WAKEUP_GPIO_PORT					(GPIO_GetPinStatus(HCI_EIF_SELECT_PORT, HCI_EIF_SELECT_PIN) == 1)?UART1_CTSN_GPIO_PORT:SPI_CLK_PORT
			#define EXTERNAL_WAKEUP_GPIO_PIN					(GPIO_GetPinStatus(HCI_EIF_SELECT_PORT, HCI_EIF_SELECT_PIN) == 1)?UART1_CTSN_GPIO_PIN:SPI_CS_PIN
			#define EXTERNAL_WAKEUP_GPIO_POLARITY			(GPIO_GetPinStatus(HCI_EIF_SELECT_PORT, HCI_EIF_SELECT_PIN) == 1)?1:0
		#else
			#if (defined(CFG_HCI_SPI) || defined(CFG_GTL_SPI))
				#define EXTERNAL_WAKEUP_GPIO_PORT				SPI_GPIO_PORT
				#define EXTERNAL_WAKEUP_GPIO_PIN 				SPI_CS_PIN
				#define EXTERNAL_WAKEUP_GPIO_POLARITY	 	0
			#else // UART
				#define EXTERNAL_WAKEUP_GPIO_PORT				UART1_CTSN_GPIO_PORT
				#define EXTERNAL_WAKEUP_GPIO_PIN 				UART1_CTSN_GPIO_PIN
				#define EXTERNAL_WAKEUP_GPIO_POLARITY	 	1
			#endif
		#endif
	#else
		#define EXTERNAL_WAKEUP_GPIO_PORT						GPIO_PORT_1
		#define EXTERNAL_WAKEUP_GPIO_PIN 						GPIO_PORT_7
		#define EXTERNAL_WAKEUP_GPIO_POLARITY	 			1
	#endif
#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
void periph_init(void);

void GPIO_reservations(void);

void set_pad_functions(void);
