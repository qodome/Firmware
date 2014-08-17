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
#include "battservice.h"

// Sensor drivers
#include "iDo.h"
#include "statistics.h"
#include "ecgservice.h"
#include "ecg_hw_cfg.h"
#include "cmdbuffer.h"
#include "memdump.h"
#include "accelerometer.h"
#include "acc.h"

/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/  
// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          2056

// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define IOS_DEFAULT_DESIRED_MIN_CONN_INTERVAL           400
#define ANDROID_DEFAULT_DESIRED_MIN_CONN_INTERVAL       800

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define IOS_DEFAULT_DESIRED_MAX_CONN_INTERVAL           800
#define ANDROID_DEFAULT_DESIRED_MAX_CONN_INTERVAL       1200

// Slave latency to use if automatic parameter update request is enabled
#define IOS_DEFAULT_DESIRED_SLAVE_LATENCY               1
#define ANDROID_DEFAULT_DESIRED_SLAVE_LATENCY           1

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define IOS_DEFAULT_DESIRED_CONN_TIMEOUT                600
#define ANDROID_DEFAULT_DESIRED_CONN_TIMEOUT            800

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
static uint8 iDo_CurrentConnParam = 0;          // 0 for android, 1 for ios

static gaprole_States_t gapProfileState = GAPROLE_INIT;
static uint16 gapConnHandle;
uint16 skip_count = 0;
static uint8 expect_seq = 0;
static uint8 seq_init_done = 0;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
#if defined(ECG_AD8232) || defined(ECG_E9624)
    0x06,
#else
0x05,   // length of this data
#endif

GAP_ADTYPE_LOCAL_NAME_COMPLETE,
'E',
'C',
'G',
'-',
#if defined(ECG_AD8232)
'A',
#elif defined(ECG_E9624)
'A',
#endif
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
};

// GAP GATT Attributes
#if defined(ECG_AD8232)
static uint8 attDeviceName[] = "ECG-A";
#elif defined(ECG_E9624)
static uint8 attDeviceName[] = "ECG-A";
#endif

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void iDo_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void ecgEnablerChangeCB(void);
static void accEnablerChangeCB(void);
static void readAccData(void);

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

// ECG Profile Callbacks
static ecgCBs_t iDo_ECGCBs =
{
    ecgEnablerChangeCB,          // Called when Enabler attribute changes
};

// ACC Profile Callbacks
static accelCBs_t iDo_ACCCBs =
{
    accEnablerChangeCB,          // Called when Enabler attribute changes
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/
void iDo_UpdateFastParameter()
{
    GAPRole_SendUpdateParam(8,
                            30,
                            0,
                            500,
                            GAPROLE_NO_ACTION);
}

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
    
    iDo_TaskID = task_id;
    
    // Setup the GAP
    VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
    VOID GAP_SetParamValue( TGAP_CONN_PARAM_TIMEOUT, 2000);
    
    // Setup the GAP Peripheral Role Profile
    {
        // Device starts advertising upon initialization
        uint8 initial_advertising_enable = TRUE;
        
        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16 gapRole_AdvertOffTime = 0;
        uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16 desired_min_interval = ANDROID_DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16 desired_max_interval = ANDROID_DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16 desired_slave_latency = ANDROID_DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout = ANDROID_DEFAULT_DESIRED_CONN_TIMEOUT;
        
        // Set the GAP Role Parameters
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
        
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
        
        GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
        
        iDo_CurrentConnParam = 0;
    }
    
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, sizeof(attDeviceName), attDeviceName );
    GGS_SetParameter( GGS_APPEARANCE_ATT, sizeof( uint16 ), (void *)&appearance);
    
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
    Stats_AddService(GATT_ALL_SERVICES);
    Batt_AddService();
    ECG_AddService(GATT_ALL_SERVICES);
    MemDump_AddService(GATT_ALL_SERVICES);
    Accel_AddService(GATT_ALL_SERVICES);
    
#if defined FEATURE_OAD
    VOID OADTarget_AddService();                    // OAD Profile
#endif
    
    ecg_hw_init_io();
    ecg_hw_sample_init();
    if (!HalAccInit()) {
        while (1);
    }
    
    // Disable clock save feature for timing acuracy
    // REF: http://e2e.ti.com/support/wireless_connectivity/f/538/t/196614.aspx
    //      http://e2e.ti.com/support/wireless_connectivity/f/538/t/319822.aspx
    HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT );
    HCI_EXT_HaltDuringRfCmd( HCI_EXT_HALT_DURING_RF_DISABLE );
    
    // Setup a delayed profile startup
    osal_set_event( iDo_TaskID, IDO_START_DEVICE_EVT );
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
        VOID GAPRole_StartDevice( &iDo_PeripheralCBs );
        
        // Start Bond Manager
        VOID GAPBondMgr_Register( &iDo_BondMgrCBs );
        
        // Start the ECG Profile
        VOID ECG_RegisterAppCBs( &iDo_ECGCBs );

        // Start the Acc Profile
        VOID Accel_RegisterAppCBs( &iDo_ACCCBs );
        
        return ( events ^ IDO_START_DEVICE_EVT );
    }
    
    if (events & IDO_ECG_TASK) {
        uint8 ecgEnabler = 0;
        bStatus_t status = ECG_GetParameter(ECG_ENABLER, &ecgEnabler);
        
        if (status == SUCCESS) {
            if (ecgEnabler) {
                struct cmd_buffer *buf_ptr = NULL;
                
                buf_ptr = CBGetNextBufferForTX();
                while (buf_ptr != NULL) {
                    if (seq_init_done == 0) {
                        seq_init_done = 1;
                        expect_seq = buf_ptr->e_buf.seq;
                    } else {
                        if (expect_seq != buf_ptr->e_buf.seq) {
                            skip_count++;
                            expect_seq = buf_ptr->e_buf.seq;
                        }
                    }
                    expect_seq++;
                    ECG_Notify(gapConnHandle, (uint8 *)&(buf_ptr->e_buf));
                    CBPutBuffer(buf_ptr);
                    buf_ptr = CBGetNextBufferForTX();
                }
                osal_start_timerEx( iDo_TaskID, IDO_ECG_TASK, ECG_TASK_PERIOD );
            } else {
                // Stop ECG
                osal_stop_timerEx(iDo_TaskID, IDO_ECG_TASK);
            }
        }
        
        return (events ^ IDO_ECG_TASK);
    }
    
    // Accelerometer
    if (events & IDO_ACC_TASK) {
        uint8 accEnabler = 0;
        
        bStatus_t status = Accel_GetParameter(ACCEL_ENABLER, &accEnabler);
        
        if (accEnabler) {
            readAccData();
            osal_start_timerEx(iDo_TaskID, IDO_ACC_TASK, ACC_TASK_PERIOD);
        } else {
            osal_stop_timerEx(iDo_TaskID, IDO_ACC_TASK);
        }        
        return (events ^ IDO_ACC_TASK);
    }
    
    // Discard unknown events
    return 0;
}

/*********************************************************************
* Private functions
*/


/*********************************************************************
* @fn      iDo_ProcessOSALMsg
*
* @brief   Process an incoming task message.
*
* @param   pMsg - message to process
*
* @return  none
*/
static void iDo_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {
    default:
        // do nothing
        break;
    }
}

int iDo_ChangeConnParameter()
{
    uint16 desired_min_interval = IOS_DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = IOS_DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = IOS_DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = IOS_DEFAULT_DESIRED_CONN_TIMEOUT;
    
    if (iDo_CurrentConnParam == 0) {
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
        iDo_CurrentConnParam = 1;
        return 0;
    } else {
        return 1;
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
    uint8 valFalse = FALSE;
    uint16 desired_min_interval = ANDROID_DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = ANDROID_DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = ANDROID_DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = ANDROID_DEFAULT_DESIRED_CONN_TIMEOUT;
    
    switch ( newState )
    {
    case GAPROLE_STARTED:
        break;
        
    case GAPROLE_ADVERTISING:
	    break;
        
    case GAPROLE_CONNECTED:
        // Get connection handle
        GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );
        
        // Update connection parameter
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
        iDo_CurrentConnParam = 0;
        
        break;
        
    case GAPROLE_WAITING:
    case GAPROLE_WAITING_AFTER_TIMEOUT:      
        // Link terminated intentionally
        
        // Change attribute value of ECG Enable to FALSE
        ECG_SetParameter(ECG_ENABLER, sizeof(valFalse), &valFalse);
        
        // Stop the ECG
        ecgEnablerChangeCB(); // SetParameter does not trigger the callback
        
        // Change attribute value of ACC Enable to FALSE
        Accel_SetParameter(ACCEL_ENABLER, sizeof(valFalse), &valFalse);
        
        // Stop the ACC
        accEnablerChangeCB(); // SetParameter does not trigger the callback        
        
        break;
        
    default:
	    break;
    }
    
    gapProfileState = newState;
}

static void ecgEnablerChangeCB( void )
{
    uint8 ecgEnabler = 0;
    bStatus_t status = ECG_GetParameter(ECG_ENABLER, &ecgEnabler);
    
    if (status == SUCCESS) {
        if (ecgEnabler)
        {
            // Turn on ECG
            ecg_hw_turn_on();
            ecg_hw_sample_begin();
            iDo_UpdateFastParameter();
            
            // Setup timer for ECG task
            osal_start_timerEx( iDo_TaskID, IDO_ECG_TASK, ECG_TASK_PERIOD );
        } else
        {
            // Turn off ECG
            ecg_hw_sample_end();            
            ecg_hw_turn_off();
            
            osal_stop_timerEx( iDo_TaskID, IDO_ECG_TASK);
        }
    } else
    {
        //??
    }
}

static void accEnablerChangeCB( void )
{
    uint8 accEnabler = 0;
    bStatus_t status = Accel_GetParameter(ACCEL_ENABLER, &accEnabler);
    
    if (status == SUCCESS) {
        if (accEnabler) {            
            // Setup timer for ECG task
            osal_start_timerEx(iDo_TaskID, IDO_ACC_TASK, ACC_TASK_PERIOD);
        } else {            
            osal_stop_timerEx(iDo_TaskID, IDO_ACC_TASK);
        }
    } else {
        //??
    }
}

static void readAccData(void)
{
    int16 aData[3];
    
    if (HalAccRead(aData)) {
        Accel_SetParameter(ACCEL_X_ATTR, sizeof(int16), (uint8 *)&aData[0]);
        Accel_SetParameter(ACCEL_Y_ATTR, sizeof(int16), (uint8 *)&aData[1]);
        Accel_SetParameter(ACCEL_Z_ATTR, sizeof(int16), (uint8 *)&aData[2]);        
    }
}

/*********************************************************************
*********************************************************************/

