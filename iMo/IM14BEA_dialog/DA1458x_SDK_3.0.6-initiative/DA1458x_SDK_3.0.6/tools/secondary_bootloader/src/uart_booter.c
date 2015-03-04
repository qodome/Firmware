/**
 ****************************************************************************************
 *
 * @file uart_booter.c
 *
 * @brief Firmware download through UART
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

#include <stdio.h>
#include "uart_booter.h"
#include "periph_setup.h"
#include "gpio.h"
#include "uart.h"
 
  
volatile WORD timeout_multi=0;
volatile WORD timeout_occur=0;
   
/**
****************************************************************************************
* @brief download firmware from UART interface
****************************************************************************************
*/
int FwDownload(void)
{
    int fw_size;
    int i;
    char *code;
    char mchoice;
    char crc_code;
    
    uart_initialization();  
    uart_send_byte(STX);         // send start TX char
    mchoice=uart_receive_byte();   
    if (mchoice!=SOH)            //  wrong response return false
        return -1;  
    mchoice=uart_receive_byte();
    fw_size= mchoice;
    mchoice=uart_receive_byte();
    fw_size+= (mchoice<<8);
    if (fw_size>max_code_length){        //  check if the program fit to memory 
        uart_send_byte(NAK); 
        return -2;
    }else{
        uart_send_byte(ACK);             // if fit return  ACK
    }
    crc_code=0;
    code=(char*) (SYSRAM_COPY_BASE_ADDRESS);        // set pointer to SYSRAM
    for(i=0;i<fw_size;i++){            // copy code from UART to RAM
        mchoice=uart_receive_byte(); 
        crc_code^=mchoice;        // update CRC
        *code=mchoice;          // write to RAM
        code++;                    // increase RAM pointer
    }
    uart_send_byte(crc_code);            // send CRC byte
    GPIO_ConfigurePin( UART_GPIO_PORT, UART_TX_PIN, INPUT, PID_UART1_RX, false );    
    GPIO_ConfigurePin( UART_GPIO_PORT, UART_RX_PIN, INPUT, PID_UART1_RX, false );    
    return fw_size;
}


