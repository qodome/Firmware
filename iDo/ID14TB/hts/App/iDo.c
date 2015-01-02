/**************************************************************************************************
Filename:       iDo.c
Revised:        $Date: 2013-08-23 11:45:31 -0700 (Fri, 23 Aug 2013) $
Revision:       $Revision: 35100 $

Description:    This file contains the Sensor Tag sample application
for use with the TI Bluetooth Low Energy Protocol Stack.

Copyright 2012-2013  Texas Instruments Incorporated. All rights reserved.

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

/*********************************************************************
* INCLUDES
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"

#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"

#if defined ( PLUS_BROADCASTER )
#include "peripheralBroadcaster.h"
#else
#include "peripheral.h"
#endif

#include "gapbondmgr.h"

#if defined FEATURE_OAD
#include "oad.h"
#include "oad_target.h"
#endif

// Services
#include "st_util.h"
#include "temperature.h"
#include "battservice.h"

#ifdef DEBUG_STATS
#include "statistics.h"
#endif

// Sensor drivers
#include "iDo.h"
#include "SoftwareSPI.h"
#include "recorder.h"
#include "cmdbuffer.h"
#include "linkdb.h"
#include "tempState.h"
#include "memdump.h"
#include "devinfoservice.h"
#include "OSAL_Clock.h"
#include "customize.h"
#include "currenttime.h"
#include "pwrmgmt.h"

/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/
#define TEMP_SAMPLE_PERIOD_DEFAULT     10000
#define TEMP_WAIT_SAMPLE            100
#define ADT_CHECK_PERIOD            300000

// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         1

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
#define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/
static uint8 iDo_TaskID;   // Task ID for internal task/event processing
static uint8 iDo_CurrentConnParam = 0;          // 0 for iOS, 1 for Android

static gaprole_States_t gapProfileState = GAPROLE_INIT;
static uint16 gapConnHandle;
static uint8 tempIntermediateSwitch = 0;
static uint8 tempMeasurementSwitch = 0;
uint32 tempSampleSleepPeriod = TEMP_SAMPLE_PERIOD_DEFAULT;
#ifdef ADVERTISE_TEMP
static int16 lastTemp = 0;
#endif
#ifdef DELETE_INFO_UNKNOWN_PEER
static uint8 lastConnAddr[B_ADDR_LEN] = {0xf,0xf,0xf,0xf,0xf,0xe};
#endif
static struct cmd_buffer *lastReadBuffer = NULL;
static uint8 adtMonitorCnt = 0;

uint16 ios_min_interval = IOS_DEFAULT_DESIRED_MIN_CONN_INTERVAL;
uint16 ios_max_interval = IOS_DEFAULT_DESIRED_MAX_CONN_INTERVAL;
uint16 ios_slave_latency = IOS_DEFAULT_DESIRED_SLAVE_LATENCY;
uint16 ios_conn_timeout = IOS_DEFAULT_DESIRED_CONN_TIMEOUT;

uint16 android_min_interval = ANDROID_DEFAULT_DESIRED_MIN_CONN_INTERVAL;
uint16 android_max_interval = ANDROID_DEFAULT_DESIRED_MAX_CONN_INTERVAL;
uint16 android_slave_latency = ANDROID_DEFAULT_DESIRED_SLAVE_LATENCY;
uint16 android_conn_timeout = ANDROID_DEFAULT_DESIRED_CONN_TIMEOUT;

/*
#ifdef DEBUG_STATS
static uint32 adv_begin_stats_tick = 0;
static uint32 conn_begin_stats_tick = 0;
static uint8 conn_parameter_type = 0;   // 0: default, 1: iOS, 2: android
#endif
*/

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[1 + 1 + GAP_DEVICE_NAME_LEN - 1] =
{
    // complete name
    0x04,   // length of this data
    
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'i',
    'D',
    'o',
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
    // Flags; this sets the device to use limited discoverable
    // mode (advertises for 30 seconds at a time) instead of general
    // discoverable mode (advertises indefinitely)
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    
    // appearance
    0x03,   // length of this data
    GAP_ADTYPE_APPEARANCE,
    LO_UINT16(GAP_APPEARE_GENERIC_THERMOMETER),
    HI_UINT16(GAP_APPEARE_GENERIC_THERMOMETER),
    
    // service UUID, to notify central devices what services are included
    // in this peripheral
    0x03,   // length of second data structure (7 bytes excluding length byte)
    GAP_ADTYPE_16BIT_MORE,   // list of 16-bit UUID's available, but not complete list
    LO_UINT16( TEMP_SERVICE ),        // Health Therometer Service
    HI_UINT16( TEMP_SERVICE ),
};

#ifdef ADVERTISE_TEMP
static uint8 advertDataWithTemp[] =
{
    // Flags; this sets the device to use limited discoverable
    // mode (advertises for 30 seconds at a time) instead of general
    // discoverable mode (advertises indefinitely)
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    
    // appearance
    0x03,   // length of this data
    GAP_ADTYPE_APPEARANCE,
    LO_UINT16(GAP_APPEARE_GENERIC_THERMOMETER),
    HI_UINT16(GAP_APPEARE_GENERIC_THERMOMETER),
    
    // service UUID, to notify central devices what services are included
    // in this peripheral
    0x03,   // length of second data structure (7 bytes excluding length byte)
    GAP_ADTYPE_16BIT_MORE,   // list of 16-bit UUID's available, but not complete list
    LO_UINT16( TEMP_SERVICE ),        // Health Therometer Service
    HI_UINT16( TEMP_SERVICE ),
    
    0x07,
    GAP_ADTYPE_SERVICE_DATA,
    LO_UINT16(TEMP_SERVICE), HI_UINT16(TEMP_SERVICE),
    0x00, 0x00, 0x00, 0xFC,
};
#endif

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = {0};

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void iDo_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void ggsValueChangeCB(uint8 attrId);
static void iDo_prepare_send_indication();

/*********************************************************************
* PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t iDo_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t iDo_BondMgrCBs =
{
    NULL,                     // Passcode callback (not used by application)
    NULL                      // Pairing / Bonding state Callback (not used by application)
};

// GAP device name changed over the air
static ggsAppCBs_t iDo_GGSCB =
{
    ggsValueChangeCB
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/
void iDo_FirmwareUpdateParameter()
{
    GAPRole_SendUpdateParam(FIRMWARE_UPDATE_DESIRED_MIN_CONN_INTERVAL,
                            FIRMWARE_UPDATE_DESIRED_MAX_CONN_INTERVAL,
                            FIRMWARE_UPDATE_DESIRED_SLAVE_LATENCY,
                            FIRMWARE_UPDATE_DESIRED_CONN_TIMEOUT,
                            GAPROLE_NO_ACTION);
}

void iDo_FastReadUpdateParameter()
{
    GAPRole_SendUpdateParam(FAST_READ_DESIRED_MIN_CONN_INTERVAL,
                            FAST_READ_DESIRED_MAX_CONN_INTERVAL,
                            FAST_READ_DESIRED_SLAVE_LATENCY,
                            FAST_READ_DESIRED_CONN_TIMEOUT,
                            GAPROLE_NO_ACTION);
}

/*
 * Return 0 to continue update
 */
int iDo_ContinueUpdateConnParameter()
{
    if (iDo_CurrentConnParam == 0) {
        iDo_CurrentConnParam = 1;
        return 0;
    } else {
        return 1;
    }
}

void iDo_ParamUpdateCB(uint16 connInterval,
                       uint16 connSlaveLatency,
                       uint16 connTimeout)
{
    if (iDo_CurrentConnParam == 0) {
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &android_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &android_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &android_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &android_conn_timeout );
/*        
#ifdef DEBUG_STATS        
        conn_parameter_type = 1;
#endif
*/
    } else {
/*
#ifdef DEBUG_STATS
        conn_parameter_type = 2;
#endif
*/
    }
        
    pwrmgmt_set_conn_param(connInterval, connSlaveLatency);
}

gapRolesParamUpdateCB_t func_param = iDo_ParamUpdateCB;

/*********************************************************************
* @fn      iDo_Init
*
* @brief   Initialization function for the Simple BLE Peripheral App Task.
*          This is called during initialization and should contain
*          any application specific initialization (ie. hardware
*          initialization/setup, table initialization, power up
*          notificaiton ... ).
*
* @param   task_id - the ID assigned by OSAL.  This ID should be
*                    used to send messages and set timers.
*
* @return  none
*/
void iDo_Init( uint8 task_id )
{
    uint16 appearance = 768;
    uint8 devNamePermission = GATT_PERMIT_READ | GATT_PERMIT_WRITE;
    
    iDo_TaskID = task_id;
    
    // Setup the GAP
    VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
    VOID GAP_SetParamValue( TGAP_CONN_PARAM_TIMEOUT, 3000);
    
    // Setup the GAP Peripheral Role Profile
    {
        // Device starts advertising upon initialization
        uint8 initial_advertising_enable = TRUE;
        
        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        
        // First apply iOS settings, this should be success; then try apply Android settings
        uint16 gapRole_AdvertOffTime = 0;
        uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
        
        // Set the GAP Role Parameters
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
        
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
        
        GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &ios_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &ios_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &ios_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &ios_conn_timeout );
        
        iDo_CurrentConnParam = 0;
    }
    
    // Set the GAP Characteristics
    custom_get_dn(attDeviceName);
    for (uint8 idx = 0; idx < (GAP_DEVICE_NAME_LEN - 1); idx++) {
        scanRspData[2 + idx] = attDeviceName[idx];
    }
    scanRspData[0] = osal_strlen((char *)attDeviceName) + 1;
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, (scanRspData[0] + 1), scanRspData );
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
    GGS_SetParameter(GGS_APPEARANCE_ATT, sizeof( uint16 ), (void *)&appearance);
    GGS_SetParameter(GGS_W_PERMIT_DEVICE_NAME_ATT, sizeof(devNamePermission), (void *)&devNamePermission);
    
    // Set advertising interval
    {
        uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;
        
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    }
    
    // Setup the GAP Bond Manager
    {
        uint32 passkey = 0; // passkey "000000"
        uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        //uint8 pairMode = GAPBOND_PAIRING_MODE_INITIATE;
        uint8 mitm = TRUE;
        uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        uint8 bonding = TRUE;
        
        GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
        GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
        GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
        GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
        GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
    }
    
    // Add services
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    Temp_AddService(GATT_ALL_SERVICES);
    DevInfo_AddService();
    CurrentTime_AddService(GATT_ALL_SERVICES);
    Batt_AddService();

#if defined FEATURE_OAD
    VOID OADTarget_AddService();                    // OAD Profile
#endif
        
    // Enable clock divide on halt
    // This reduces active current while radio is active and CC254x MCU
    // is halted
    HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

    // Initialize all IO ports to output
    P0DIR = 0xFF;
    P1DIR = 0xFF;
    P2DIR |= 0x1F;
    P0 = 0;
    P1 = 0;
    P2 = 0;   
    
    // Initialize data structures
    recorder_init();
    
    // Initialize IO to ADT7320
    Temp_Init();
#ifdef DEBUG_STATS
    //Stats_Init();
    MemDump_AddService();
#endif
    
    temp_state_init();
    CBInit();
    
    // Reset ADT7320 SPI interface
    SPI_intf_reset();
    // Schedule shutdown cmd after 1ms
    osal_start_timerEx(iDo_TaskID, IDO_SHUTDOWN_ADT7320, 1);
    // Schedule ADT7320 monitor
    osal_start_timerEx(iDo_TaskID, IDO_MONITOR_ADT7320, ADT_CHECK_PERIOD);
        
    // Setup a delayed profile startup
    osal_set_event(iDo_TaskID, IDO_START_DEVICE_EVT);
    
    // Disable unrelevent interrupts
    IEN0 &= ~0x1F;
    IEN1 &= ~0x3A;
    IEN2 &= ~0x1E;
    
    // Start watchdog, timeout period: 1s
    WDCTL = BV(3);
    WD_KICK();
    
    // Initialize custom name
    custom_init();

    // Initialize power management module
    pwrmgmt_init();
    
    HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_MINUS_23_DBM);
    pwrmgmt_event(TX_LOW);
    
    HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_STD);
    pwrmgmt_event(RX_LOW);
}

/*********************************************************************
* @fn      iDo_ProcessEvent
*
* @brief   Simple BLE Peripheral Application Task event processor.  This function
*          is called to process all events for the task.  Events
*          include timers, messages and any other user defined events.
*
* @param   task_id  - The OSAL assigned task ID.
* @param   events - events to process.  This is a bit map and can
*                   contain more than one event.
*
* @return  events not processed
*/
uint16 iDo_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function
    
    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;
        
        if ( (pMsg = osal_msg_receive( iDo_TaskID )) != NULL )
        {
            iDo_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
            
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }
        
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }
    
    if ( events & IDO_START_DEVICE_EVT )
    {            
        // Start the Device
        VOID GAPRole_StartDevice(&iDo_PeripheralCBs);
        
        // Start Bond Manager
        VOID GAPBondMgr_Register(&iDo_BondMgrCBs);
        
        // Register GGS device name chage callback
        VOID GGS_RegisterAppCBs(&iDo_GGSCB);
        
        // Connection parameter update callback
        VOID GAPRole_RegisterAppCBs(&func_param);
        
        return ( events ^ IDO_START_DEVICE_EVT );
    }
    
    if (events & IDO_DO_SAMPLE_TEMP_EVT) {
        if (tempIntermediateSwitch == 1) {
            // Update Temperature value
            tempRead();
/*            
#ifdef DEBUG_STATS
            s.intermediate_cycle++;       
#endif
*/
        }
        return (events ^ IDO_DO_SAMPLE_TEMP_EVT);
    }
    
    if (events & IDO_READ_SAMPLE_TEMP_EVT) {
        tempReadCallback();
        osal_start_timerEx(iDo_TaskID, IDO_DO_SAMPLE_TEMP_EVT, tempSampleSleepPeriod);
        pwrmgmt_event(MEASURE_TEMP);   
        return (events ^ IDO_READ_SAMPLE_TEMP_EVT);
    }    
    
    if (events & IDO_MEASUREMENT_INDICATION) {        
        osal_stop_timerEx(iDo_TaskID, IDO_MEASUREMENT_INDICATION);
        
        // If temperature measurement has been turned off, exit right away
        if (tempMeasurementSwitch == 0) {
            return (events ^ IDO_MEASUREMENT_INDICATION);
        }
        
        if (temp_state_mesaurement_ready()) {
            CBPushTemp(temp_state_get_last_filtered_temp());
        }

/*        
#ifdef DEBUG_STATS
        s.temp_measure_cycle++;        
#endif
*/        
        iDo_prepare_send_indication();
        
        osal_start_timerEx(iDo_TaskID, IDO_MEASUREMENT_INDICATION, Temp_mill_seconds_before_next_indication());
        
        return (events ^ IDO_MEASUREMENT_INDICATION);
    }
    
    if (events & IDO_SHUTDOWN_ADT7320) {
        Temp_Disable();
        osal_stop_timerEx(iDo_TaskID, IDO_SHUTDOWN_ADT7320);
        return (events ^ IDO_SHUTDOWN_ADT7320);
    }
    
    if (events & IDO_MONITOR_ADT7320) {        
        adtMonitorCnt++;
        if (adtMonitorCnt >= 12) {
            
            // Trigger power mgmt module dump statistics
            pwrmgmt_flash_dump();
        
            adtMonitorCnt = 0;
            if ((Temp_Monitor() & 0x60) == 0x00) {
                // ADT7320 has been reset? Disable temperature measurement to save power
                Temp_Disable();
/*
#ifdef DEBUG_STATS
                s.adt_abnormal_cnt++;             
#endif
*/                
            }
        }
        osal_start_timerEx(iDo_TaskID, IDO_MONITOR_ADT7320, ADT_CHECK_PERIOD); 
        return (events ^ IDO_MONITOR_ADT7320);
    }

#ifdef ADVERTISE_TEMP     
    if (events & IDO_ADVERTISE_EVT) {
       
        // We will send temperature data over advertise scan rsp
        if (tempIntermediateSwitch != 0 || tempMeasurementSwitch != 0) {
            int16 t16 = lastTemp;
            int32 t32;
            int32 sign = -4;
            int32 *p32 = (int32 *)&(advertDataWithTemp[15]);
            
            if (t16 & 0x8000) {
                t32 = (int32)t16 | 0xFFFF0000;
            } else {
                t32 = t16;
            }
            *p32 = (sign << 24) | ((t32 * 625) & 0xFFFFFF);        
            GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertDataWithTemp), advertDataWithTemp);
        }

        uint32 timeNow = osal_getRelativeClock();
        /*
         * If we have been lost connection for 10 minutes, give up retrial and back to low power
         */
        if ((timeNow - lostConnectionTime) >= 600) {
            if (tempIntermediateSwitch != 0 || tempMeasurementSwitch != 0) {
                GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
            }           
            osal_stop_timerEx(iDo_TaskID, IDO_ADVERTISE_EVT);
            
            HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_MINUS_23_DBM);
            pwrmgmt_event(TX_LOW);
            
            HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_STD);
            pwrmgmt_event(RX_LOW);
        } else {
            osal_start_timerEx(iDo_TaskID, IDO_ADVERTISE_EVT, 10000);            
        }
        
        return (events ^ IDO_ADVERTISE_EVT);
    }
#endif
    
    if (events & IDO_CHECK_VDD_VOLTAGE) {
        pwrmgmt_checkvdd_callback();
        return (events ^ IDO_CHECK_VDD_VOLTAGE);
    }
    
    if (events & IDO_PWR_HEART_BEAT) {
        if (pwrmgmt_hb() == 0) {
            osal_start_timerEx(iDo_TaskID, IDO_PWR_HEART_BEAT, 60000);
        } else {
            osal_stop_timerEx(iDo_TaskID, IDO_PWR_HEART_BEAT);
        }
        return (events ^ IDO_PWR_HEART_BEAT);
    }
    
    // Discard unknown events
    return 0;
}

void iDo_ScheduleCheckVDD(uint16 delay)
{
    osal_start_timerEx(iDo_TaskID, IDO_CHECK_VDD_VOLTAGE, delay);
}

void iDo_StopVDDCheck(void)
{
    osal_stop_timerEx(iDo_TaskID, IDO_CHECK_VDD_VOLTAGE);
}

/*********************************************************************
* Private functions
*/
static void iDo_ProcessGattMsg(gattMsgEvent_t *pMsg)
{    
    int16 temp; 
    uint8 validFlag;
    UTCTimeStruct tc;

    //Measurement Indication Confirmation
    if (pMsg->method == ATT_HANDLE_VALUE_CFM) {
        if (lastReadBuffer != NULL) {
            CBPutBuffer(lastReadBuffer);
            lastReadBuffer = NULL;
            if (CBGetNextBufferForTX(&temp, &validFlag, &tc) != NULL) {
                iDo_prepare_send_indication();
            }
        }
    }
}

/*********************************************************************
* @fn      iDo_ProcessOSALMsg
*
* @brief   Process an incoming task message.
*
* @param   pMsg - message to process
*
* @return  none
*/
static void iDo_ProcessOSALMsg(osal_event_hdr_t *pMsg)
{
    switch (pMsg->event)
    {
    case GATT_MSG_EVENT:
        iDo_ProcessGattMsg((gattMsgEvent_t *) pMsg);
        break;
        
    default:
        break;
    }
}

/*********************************************************************
* @fn      peripheralStateNotificationCB
*
* @brief   Notification from the profile of a state change.
*
* @param   newState - new state
*
* @return  none
*/
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef DELETE_INFO_UNKNOWN_PEER
    linkDBItem_t  *pItem;
#endif    
/*
#ifdef DEBUG_STATS
    uint32 connected_sec = 0;
#endif
*/

    switch ( newState )
    {
    case GAPROLE_STARTED:
        break;
        
    case GAPROLE_ADVERTISING:
	    break;
        
    case GAPROLE_CONNECTED:
        pwrmgmt_event(CONNECT);
        
        // Set tx/rx power to high once connected
        HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_0_DBM);
        pwrmgmt_event(TX_HIGH);
        HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
        pwrmgmt_event(RX_HIGH);

#ifdef ADVERTISE_TEMP
        // Stop advertise timer, set default advertise data
        osal_stop_timerEx(iDo_TaskID, IDO_ADVERTISE_EVT);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
#endif
        
        // Start power management heart beat
        osal_start_timerEx(iDo_TaskID, IDO_PWR_HEART_BEAT, 60000);

/*        
#ifdef DEBUG_STATS
        s.adv_seconds += (osal_getClock() - adv_begin_stats_tick);
        
        conn_begin_stats_tick = osal_getClock();
        conn_parameter_type = 1;    // The first parameter set is android    
#endif
*/
        
        // Get connection handle
        GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );
 
        // Update connection parameter
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &ios_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &ios_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &ios_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &ios_conn_timeout );
        iDo_CurrentConnParam = 0;  

#ifdef DELETE_INFO_UNKNOWN_PEER
        // Get peer MAC address
        if ((pItem = linkDB_Find(gapConnHandle)) != NULL) {
            if (!osal_memcmp(pItem->addr, lastConnAddr, B_ADDR_LEN) && CBDataAvailable()) {
                // Cleanup the previous measurement buffer
                CBCleanup();
            }           
        }
        VOID osal_memcpy(lastConnAddr, pItem->addr, B_ADDR_LEN);
#endif
        
        break;
        
    case GAPROLE_WAITING:
    case GAPROLE_WAITING_AFTER_TIMEOUT:        
        pwrmgmt_event(DISCONNECT);
                
        if (newState == GAPROLE_WAITING) {
            osal_stop_timerEx(iDo_TaskID, IDO_PWR_HEART_BEAT);
        } else {
            pwrmgmt_timeout();            
        }
        
        if (newState == GAPROLE_WAITING) {
            // Link terminated intentionally: disable temperature sensor, lower tx power
            HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_MINUS_23_DBM);
            pwrmgmt_event(TX_LOW);
            
            HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_STD);
            pwrmgmt_event(RX_LOW);
        } else {
            
            HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_0_DBM);
            pwrmgmt_event(TX_HIGH);    
            
            HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
            pwrmgmt_event(RX_HIGH);   
        }
    
/*
#ifdef DEBUG_STATS
        adv_begin_stats_tick = osal_getClock();
        
        connected_sec = osal_getClock() - conn_begin_stats_tick;
        if (conn_parameter_type == 2) {
            s.connected_android_seconds += connected_sec;
        } else if (conn_parameter_type == 1) {
            s.connected_ios_seconds += connected_sec;
        } else {
            s.connected_default_seconds += connected_sec;
        }
#endif
*/        
#ifndef DEBUG_STATS
        if (MemDump_ServiceNeedDelete()) {
            MemDump_DelService();
        }
#endif
        break;
        
#if 0
    case GAPROLE_WAITING_AFTER_TIMEOUT:
        // If disconnected due to timeout, do not stop timer
        //osal_stop_timerEx(iDo_TaskID, IDO_PWR_HEART_BEAT);
        
        pwrmgmt_timeout();
        pwrmgmt_event(DISCONNECT);
        
        HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_0_DBM);
        pwrmgmt_event(TX_HIGH);

#ifdef ADVERTISE_TEMP        
        // Start monitor 
        osal_set_event(iDo_TaskID, IDO_ADVERTISE_EVT);
#endif
        
/*
#ifdef DEBUG_STATS
        adv_begin_stats_tick = osal_getClock();
        
        connected_sec = osal_getClock() - conn_begin_stats_tick;
        if (conn_parameter_type == 2) {
            s.connected_android_seconds += connected_sec;
        } else if (conn_parameter_type == 1) {
            s.connected_ios_seconds += connected_sec;
        } else {
            s.connected_default_seconds += connected_sec;
        }
#endif
*/
#ifndef DEBUG_STATS        
        if (MemDump_ServiceNeedDelete()) {
            MemDump_DelService();
        }
#endif
        break;
#endif
        
    default:
	    break;
    }
    
    gapProfileState = newState;
}

static void ggsValueChangeCB(uint8 attrId)
{
    GGS_GetParameter(GGS_DEVICE_NAME_ATT, attDeviceName);
    custom_set_dn(attDeviceName);
    
    for (uint8 idx = 0; idx < (GAP_DEVICE_NAME_LEN - 1); idx++) {
        scanRspData[2 + idx] = attDeviceName[idx];
    }
    scanRspData[0] = osal_strlen((char *)attDeviceName) + 1;
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, (scanRspData[0] + 1), scanRspData );
}

int8 iDo_Connected(void)
{
    if (gapProfileState == GAPROLE_CONNECTED)
        return 1;
    else
        return 0;
}

/*
* This function is root source for all temperatures (both measurement and intermediate)
* Intermediate temperature is reported here, measurement is reported by another routine
*/
void tempReadCallback()
{
    int16 v;
    uint8 buf[20];
    uint8 len;
    
    Temp_Disable();
    v = spi_read(0x02);
    v = v >> 3;    
    if (v & 0x1000) {
        // negative
        v |= 0xE000;
    }
    
#ifdef ADVERTISE_TEMP
    lastTemp = v;
#endif
    
    temp_state_update(v);
    recorder_add_temperature(v);
        
    if (gapProfileState == GAPROLE_CONNECTED) {
        // Skip intermediate report if temperature measurement is turned on
        if ((tempMeasurementSwitch == 1) && Temp_TM_sending(gapConnHandle) && temp_state_mesaurement_ready()) {
            return;
        }   
        
#ifdef ATTACH_DETECTION
        len = Temp_FinishPacket(buf, v, temp_state_is_attached(), NULL);
#else
        len = Temp_FinishPacket(buf, v, NULL);
#endif
        Temp_NotifyTemperature(gapConnHandle, buf, len);
    }   
}

void tempRead()
{
/*
#ifdef DEBUG_STATS
    s.adt_read_count++;
#endif
*/
    Temp_Enable();
    osal_start_timerEx(iDo_TaskID, IDO_READ_SAMPLE_TEMP_EVT, TEMP_WAIT_SAMPLE);
}

void iDoTurnOnTemp(void)
{
    if (tempIntermediateSwitch == 0) {
        tempIntermediateSwitch = 1;
#ifdef ADVERTISE_TEMP
        lastTemp = 0;
#endif
        osal_start_timerEx(iDo_TaskID, IDO_DO_SAMPLE_TEMP_EVT, tempSampleSleepPeriod);
    }
}

void iDoTurnOffTemp(void)
{
    if (tempIntermediateSwitch != 0) {
        tempIntermediateSwitch = 0;
        osal_stop_timerEx(iDo_TaskID, IDO_DO_SAMPLE_TEMP_EVT);
    }
}

void iDo_turn_on_measurement_indication(void)
{
    if (tempMeasurementSwitch == 0) {
        tempMeasurementSwitch = 1;
        osal_set_event(iDo_TaskID, IDO_MEASUREMENT_INDICATION);
    }
}

void iDo_turn_off_measurement_indication(void)
{
    if (tempMeasurementSwitch != 0) {
        tempMeasurementSwitch = 0;
        osal_stop_timerEx(iDo_TaskID, IDO_MEASUREMENT_INDICATION);
    }
}

void iDo_indicate_measurement_ready(void)
{
    if (tempMeasurementSwitch != 0) {
        // Reschedule the next measurement indication
        osal_set_event(iDo_TaskID, IDO_MEASUREMENT_INDICATION);
    }
}

static void iDo_prepare_send_indication()
{    
    int16 temp;
    UTCTimeStruct tc;
    uint8 buf[20];
    uint8 len;
    uint8 validFlag;
    
    if (gapProfileState == GAPROLE_CONNECTED) {
        lastReadBuffer = CBGetNextBufferForTX(&temp, &validFlag, &tc);
        if (lastReadBuffer != NULL) {
#ifdef ATTACH_DETECTION   
            len = Temp_FinishPacket(buf, temp, (validFlag & TYPE_VALID_FLAG), &tc);
#else
            len = Temp_FinishPacket(buf, temp, (validFlag & TIME_VALID_FLAG) ? &tc : NULL);
            
#endif
            Temp_IndicateTemperature(gapConnHandle, buf, len, iDo_TaskID);
        }
    }
}

/*********************************************************************
*********************************************************************/

