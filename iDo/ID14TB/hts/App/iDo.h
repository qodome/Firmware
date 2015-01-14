/**************************************************************************************************
  Filename:       iDo.h
  Revised:        $Date: 2013-03-25 07:58:08 -0700 (Mon, 25 Mar 2013) $
  Revision:       $Revision: 33575 $

  Copyright 2012-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef IDO_H
#define IDO_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// iDo Task Events
#define IDO_START_DEVICE_EVT                              0x0001
#define IDO_DO_SAMPLE_TEMP_EVT                            0x0002
#define IDO_READ_SAMPLE_TEMP_EVT                          0x0004
#define IDO_CHECK_VDD_VOLTAGE                             0x0008
#ifdef ADVERTISE_TEMP
#define IDO_ADVERTISE_EVT                                 0x0010
#endif
#define IDO_MEASUREMENT_INDICATION                        0x0020
#define IDO_SHUTDOWN_ADT7320                              0x0040
#define IDO_MONITOR_ADT7320                               0x0080
#define IDO_PWR_HEART_BEAT                                0x0100
#define IDO_SCHEDULE_RECORD_API_TASK                      0x0200

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          2056
    
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define IOS_DEFAULT_DESIRED_MIN_CONN_INTERVAL           400
#define ANDROID_DEFAULT_DESIRED_MIN_CONN_INTERVAL       800
#define FIRMWARE_UPDATE_DESIRED_MIN_CONN_INTERVAL       8
#define FAST_READ_DESIRED_MIN_CONN_INTERVAL             80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define IOS_DEFAULT_DESIRED_MAX_CONN_INTERVAL           800
#define ANDROID_DEFAULT_DESIRED_MAX_CONN_INTERVAL       1200
#define FIRMWARE_UPDATE_DESIRED_MAX_CONN_INTERVAL       30
#define FAST_READ_DESIRED_MAX_CONN_INTERVAL             100

// Slave latency to use if automatic parameter update request is enabled
#define IOS_DEFAULT_DESIRED_SLAVE_LATENCY               1
#define ANDROID_DEFAULT_DESIRED_SLAVE_LATENCY           1
#define FIRMWARE_UPDATE_DESIRED_SLAVE_LATENCY           0
#define FAST_READ_DESIRED_SLAVE_LATENCY                 1

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define IOS_DEFAULT_DESIRED_CONN_TIMEOUT                600
#define ANDROID_DEFAULT_DESIRED_CONN_TIMEOUT            800
#define FIRMWARE_UPDATE_DESIRED_CONN_TIMEOUT            500
#define FAST_READ_DESIRED_CONN_TIMEOUT                  500    
            
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */    

/*
 * Thermometer Interval Range 
 */
typedef struct
{
  uint16 low;         
  uint16 high; 
} thermometerIRange_t;

extern uint32 tempSampleSleepPeriod;

/*
 * Task Initialization for the BLE Application
 */
extern void iDo_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 iDo_ProcessEvent( uint8 task_id, uint16 events );

extern void iDo_FirmwareUpdateParameter();

extern void iDo_FastReadUpdateParameter();

void tempRead();
void tempReadCallback();

void iDoTurnOnTemp(void);
void iDoTurnOffTemp(void);
void iDo_turn_on_measurement_indication(void);
void iDo_turn_off_measurement_indication(void);

void iDo_indicate_measurement_ready(void);

void iDo_schedule_recorder_API_task(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* IDO_H */
