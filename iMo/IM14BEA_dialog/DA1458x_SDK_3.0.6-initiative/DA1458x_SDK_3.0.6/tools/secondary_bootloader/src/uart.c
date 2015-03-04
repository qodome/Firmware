/**
 ****************************************************************************************
 *
 * @file uart.c
 *
 * @brief uart initialization & print functions
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
 
#include "global_io.h"
//#include "timer.h"
#include "gpio.h"
#include <core_cm0.h>
#include "periph_setup.h"
#include "uart.h"
#include "timer.h"

extern volatile WORD timeout_occur;
  
/*****************************************************************************************************/
/*                             U   A   R   T                                                         */
/*****************************************************************************************************/
#define SYSRAM_START_ADDRESS 0x0   
/**
 ****************************************************************************************
 * @brief UART initialization 
 *
 ****************************************************************************************
 */
void uart_initialization(void)
{   
  SetBits16(CLK_PER_REG, UART1_ENABLE, 1);    // enable  clock for UART 1
  SetWord16(UART_LCR_REG, 0x80); // set bit to access DLH and DLL register 
  SetWord16(UART_RBR_THR_DLL_REG,UART_BAUDRATE&0xFF);//set low byte
   SetWord16(UART_LCR_REG,3);    // no parity, 1 stop bit 8 data length and clear bit 8
  SetBits16(UART_MCR_REG, UART_SIRE, 0);  // mode 0 for normal , 1 for IRDA
  SetWord16(UART_IIR_FCR_REG,1);  // enable fifo  
  SetBits16(UART_IER_DLH_REG,ERBFI_dlh0,0);  // IER access, disable interrupt for available data
  SetWord16(UART_RBR_THR_DLL_REG,UART_BAUDRATE);
  SetWord16(UART_LCR_REG,3);      
  SetWord16(UART_IIR_FCR_REG,UART_FIFO_EN|UART_RX_FIFO_RESET|UART_TX_FIFO_RESET);    // enable fifo and reset TX and RX fifo
  GPIO_ConfigurePin( UART_GPIO_PORT, UART_TX_PIN, OUTPUT, PID_UART1_TX, false );
  GPIO_ConfigurePin( UART_GPIO_PORT, UART_RX_PIN, INPUT, PID_UART1_RX, false );    
}

extern   char *code; 
void uart_test(void){
    printf_string("\n\r\n\r*************");
    printf_string("\n\r* UART TEST *\n\r");
    printf_string("*************\n\r");
    printf_string("\n\rHello World! == UART printf_string() ==.\n\r");
}
/**
 ****************************************************************************************
 * @brief Receive byte from UART   
 *
 ****************************************************************************************
 */ 
char uart_receive_byte(void){
  start_timer(15);
  do{
    
   }while( (GetWord16(UART_LSR_REG) &0x01)==0 && (timeout_occur==0)   );  // wait until serial data is ready 
  if (timeout_occur==0) 
    stop_timer();
  else 
    timeout_occur =0;
  return 0xFF&GetWord16(UART_RBR_THR_DLL_REG);  // read from receive data buffer
} 
 /**
 ****************************************************************************************
 * @brief Sent byte from UART   
 * @param[in] ch:  character to print
 ****************************************************************************************
 */  
void uart_send_byte(char ch){
  do{
  }while((GetWord16(UART_LSR_REG)&0x20)==0);    // read status reg to check if THR is empty
  SetWord16(UART_RBR_THR_DLL_REG,(0xFF&ch));    // write to THR register
  do{                        // wait until start transmission 
  }while((GetWord16(UART_LSR_REG)&0x40)==0);    // read status reg to check if THR is empty
  return;
} 
/**
 ****************************************************************************************
 * @brief Print a string to UART 
 * @param[in] ch:  string to print
 *
 ****************************************************************************************
 */ 
void printf_string(char * str){   
  while(*str!=0){        // while not reach the last string character
    uart_send_byte(*str); // send next string character
    str++;
  }
 }

/**
 ****************************************************************************************
 * @brief Print a character to UART 
 * @param[in] ch:  character to print
 *
 ****************************************************************************************
 */ 
 void printf_byte(int a){      // print a Byte in hex format
  char b;
  b=((0xF0&a)>>4);
  b+= (b<10)?48:55;
  uart_send_byte(b);
  b=(0xF&a);
  b+= (b<10)?48:55;
  uart_send_byte(b);
}
