#ifndef APP_BUTTON_LED_H_
#define APP_BUTTON_LED_H_

/**
****************************************************************************************
*
* @file app_button_led.h
*
* @brief Push Button and LED handling header file.
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
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
void red_blink_start(void);
void green_blink_fast(void);
void green_blink_slow(void);
void turn_off_led(void);
void app_button_enable(void);
void GPIO1_Handler(void);

/**
 ****************************************************************************************
 *
 * Button LED Functions
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable the proximity profile
 *
 ****************************************************************************************
 */
 
// void GPIO3_Handler(void);

// void button_enable_wakeup_irq(void);

// void app_prec_adv_start(void);

// void app_prec_adv_stop(void);

// void app_button_enable(void);

// void app_led_enable(void);
/// @} APP

#endif // APP_H_
