/**
****************************************************************************************
*
* @file app_button_led.c
*
* @brief Push Button and LED user interface
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

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */
 
 /*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "global_io.h"
#include "gpio.h"
#include <core_cm0.h>
#include "app_button_led.h"

 /*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

void red_blink_start(void)
{
  SetWord16(P13_MODE_REG,PID_PWM0|OUTPUT);   // set pin as PWM  
  SetBits16(CLK_PER_REG, TMR_DIV, 3);        // 2 MHz input clock
  SetBits16(CLK_PER_REG, TMR_ENABLE, 1);     // enable clock
  SetBits16(TIMER0_CTRL_REG,PWM_MODE,0);     // normal PWM output
  SetBits16(TIMER0_CTRL_REG,TIM0_CLK_SEL,0); // Select 32kHz input clock
  SetBits16(TIMER0_CTRL_REG,TIM0_CLK_DIV,0); // Div by 10 input clock. so 200KHz for OnCounter

  SetWord16(TIMER0_RELOAD_M_REG,16000);      // PWM DC 50% with freq 2MHz/5K = 400Hz  
  SetWord16(TIMER0_RELOAD_N_REG,16000);
	
  SetBits16(TIMER0_CTRL_REG,TIM0_CTRL,1);    // Enable timer
}

void green_blink_slow(void)
{
  SetWord16(P12_MODE_REG,PID_PWM0|OUTPUT);   // set pin as PWM  
  SetBits16(CLK_PER_REG, TMR_DIV, 3);        // 2 MHz input clock
  SetBits16(CLK_PER_REG, TMR_ENABLE, 1);     // enable clock
  SetBits16(TIMER0_CTRL_REG,PWM_MODE,0);     // normal PWM output
  SetBits16(TIMER0_CTRL_REG,TIM0_CLK_SEL,0); // Select 32kHz input clock
  SetBits16(TIMER0_CTRL_REG,TIM0_CLK_DIV,0); // Div by 10 input clock. so 200KHz for OnCounter

  SetWord16(TIMER0_RELOAD_M_REG,24000);      // PWM DC 50% with freq 2MHz/5K = 400Hz  
  SetWord16(TIMER0_RELOAD_N_REG,24000);
	
  SetBits16(TIMER0_CTRL_REG,TIM0_CTRL,1);    // Enable timer
}

void green_blink_fast(void)
{
  SetWord16(P12_MODE_REG,PID_PWM0|OUTPUT);   // set pin as PWM  
  SetBits16(CLK_PER_REG, TMR_DIV, 3);        // 2 MHz input clock
  SetBits16(CLK_PER_REG, TMR_ENABLE, 1);     // enable clock
  SetBits16(TIMER0_CTRL_REG,PWM_MODE,0);     // normal PWM output
  SetBits16(TIMER0_CTRL_REG,TIM0_CLK_SEL,0); // Select 32kHz input clock
  SetBits16(TIMER0_CTRL_REG,TIM0_CLK_DIV,0); // Div by 10 input clock. so 200KHz for OnCounter

  SetWord16(TIMER0_RELOAD_M_REG,8000);       // PWM DC 50% with freq 2MHz/5K = 400Hz  
  SetWord16(TIMER0_RELOAD_N_REG,8000);
	
  SetBits16(TIMER0_CTRL_REG,TIM0_CTRL,1);    // Enable timer
}

void turn_off_led(void)
{
  SetBits16(TIMER0_CTRL_REG,TIM0_CTRL,0);    // Disable Timer 
  SetWord16(P12_MODE_REG,PID_GPIO|INPUT);    // set pin as GPIO input
  SetWord16(P13_MODE_REG,PID_GPIO|INPUT);    // set pin as GPIO input  	
  SetBits16(CLK_PER_REG, TMR_ENABLE, 0);     // disable clock
  SetWord16(TIMER0_RELOAD_M_REG,0);          // set zero to M reg
  SetWord16(TIMER0_RELOAD_N_REG,0);          // set zero to N reg
  SetWord16(TIMER0_ON_REG,0);                // set zero to On reg
  SetBits16(CLK_PER_REG, TMR_ENABLE, 10);    // close clock
}

void app_button_enable(void)
{   
  NVIC_DisableIRQ(GPIO1_IRQn);

  //Push Button input	
  SetWord16(P11_MODE_REG,INPUT_PULLUP);                // set P1_1 as pull-up input
  SetBits16(GPIO_INT_LEVEL_CTRL_REG, EDGE_LEVELn1, 1); //select falling edge P1.1
  SetBits16(GPIO_INT_LEVEL_CTRL_REG, INPUT_LEVEL1, 1); //select falling edge P1.1
  SetWord16(GPIO_IRQ1_IN_SEL_REG, 10);                 //P1.1 is selected, GPIO Input push buttton
	
  NVIC_SetPriority(GPIO1_IRQn,1);
  NVIC_EnableIRQ(GPIO1_IRQn);
}
