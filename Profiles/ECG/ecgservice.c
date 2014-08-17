/*
* ECG profile 
*/
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "hal_assert.h"
#include "hal_board.h"
#include "hal_defs.h"
#include "hal_drivers.h"
#include "hal_dma.h"
#include "hal_mcu.h"
#include "hal_types.h"
#include "hal_uart.h"
#include "hal_led.h"

#include "ecgservice.h"
#include "st_util.h"

/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/
// ECG service
CONST uint8 ecgServUUID[ATT_BT_UUID_SIZE] =
{ 
    LO_UINT16(ECG_SERVICE), HI_UINT16(ECG_SERVICE)
};

// ECG characteristic
CONST uint8 ecgEnablerUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(ECG_ENABLE_UUID), HI_UINT16(ECG_ENABLE_UUID)
};

// ECG characteristic
CONST uint8 ecgValueUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(ECG_UUID), HI_UINT16(ECG_UUID)
};

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/

static ecgServiceCB_t ecgServiceCB;
static ecgCBs_t *ecg_AppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

static CONST gattAttrType_t ecgService = { ATT_BT_UUID_SIZE, ecgServUUID };

// ECG Value Characteristic
static uint8 ecgValueProps = GATT_PROP_NOTIFY;

#define ECG_NOTIFY_BUFFER_SIZE      20
uint8 ecgValue[ECG_NOTIFY_BUFFER_SIZE] = {0};
static gattCharCfg_t valueConfigCoordinates[GATT_MAX_NUM_CONN];

// Enabler Characteristic Properties
static uint8 ecgEnabledCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Enabler Characteristic Value
static uint8 ecgEnabled = FALSE;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t ecgAttrTbl[] = 
{
    // ECG Service
    { 
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8 *)&ecgService                /* pValue */
    },
    
    // ECG Enabler Characteristic Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &ecgEnabledCharProps 
    },
    
    // ECG Enable Characteristic Value
    { 
        { ATT_BT_UUID_SIZE, ecgEnablerUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        &ecgEnabled 
    },
    
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &ecgValueProps 
    },
    
    { 
        { ATT_BT_UUID_SIZE, ecgValueUUID },
        0, 
        0, 
        (unsigned char *)&ecgValue 
    },
    
    { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        (uint8 *)&valueConfigCoordinates 
    },    
};


/*********************************************************************
* LOCAL FUNCTIONS
*/
static uint8 ecg_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t ecg_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

/*********************************************************************
* PROFILE CALLBACKS
*/
CONST gattServiceCBs_t ecgCBs =
{
    ecg_ReadAttrCB,  // Read callback function pointer
    ecg_WriteAttrCB, // Write callback function pointer
    NULL             // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      ECG_AddService
*
* @brief   Initializes the ECG service by registering
*          GATT attributes with the GATT server.
*
* @param   services - services to add. This is a bit map and can
*                     contain more than one service.
*
* @return  Success or Failure
*/
bStatus_t ECG_AddService( uint32 services )
{
    uint8 status = SUCCESS;
    
    VOID linkDB_Register( ECG_HandleConnStatusCB );
    
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, valueConfigCoordinates );    
    
    status = GATTServApp_RegisterService( ecgAttrTbl, 
                                         GATT_NUM_ATTRS( ecgAttrTbl ),
                                         &ecgCBs );
    
    return ( status );
}

/*********************************************************************
* @fn      ECG_Register
*
* @brief   Register a callback function with the ECG Service.
*
* @param   pfnServiceCB - Callback function.
*
* @return  None.
*/
extern void ECG_Register( ecgServiceCB_t pfnServiceCB )
{
    ecgServiceCB = pfnServiceCB;
}

bStatus_t ECG_RegisterAppCBs( ecgCBs_t *appCallbacks )
{
    if ( appCallbacks )
    {
        ecg_AppCBs = appCallbacks;
        
        return ( SUCCESS );
    }
    else
    {
        return ( bleAlreadyInRequestedMode );
    }
}

bStatus_t ECG_SetParameter( uint8 param, uint8 len, uint8 *value)
{
    bStatus_t ret = SUCCESS;
    
    switch ( param )
    {
    case ECG_ENABLER:
        if ( len == sizeof ( uint8 ) ) 
        {
            ecgEnabled = *((uint8*)value);
        }
        else
        {
            ret = bleInvalidRange;
        }
        break;
        
    default:
        ret = INVALIDPARAMETER;
        break;
    }
    
    return ( ret );
}

bStatus_t ECG_GetParameter( uint8 param, void *value )
{
    bStatus_t ret = SUCCESS;
    switch ( param )
    {
    case ECG_ENABLER:
        *((uint8*)value) = ecgEnabled;
        break;
        
    default:
        ret = INVALIDPARAMETER;
        break;
    }
    
    return ( ret );
}

/*********************************************************************
* @fn          ecg_ReadAttrCB
*
* @brief       Read an attribute.
*
* @param       connHandle - connection message was received on
* @param       pAttr - pointer to attribute
* @param       pValue - pointer to data to be read
* @param       pLen - length of data to be read
* @param       offset - offset of the first octet to be read
* @param       maxLen - maximum length of data to be read
*
* @return      Success or Failure
*/
static uint8 ecg_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
    bStatus_t status = SUCCESS;
    uint16 uuid = 0;
    
    if (utilExtractUuid16(pAttr,&uuid) == FAILURE)
    {
        // Invalid handle
        *pLen = 0;
        return ATT_ERR_INVALID_HANDLE;
    }
    
    switch (uuid) {
    case ECG_UUID:
        *pLen = ECG_NOTIFY_BUFFER_SIZE;
        VOID osal_memcpy(pValue, pAttr->pValue, ECG_NOTIFY_BUFFER_SIZE);
        break;
        
    case ECG_ENABLE_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
        
    default:
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
    
    return ( status );
}

/*********************************************************************
* @fn      ecg_WriteAttrCB
*
* @brief   Validate attribute data prior to a write operation
*
* @param   connHandle - connection message was received on
* @param   pAttr - pointer to attribute
* @param   pValue - pointer to data to be written
* @param   len - length of data
* @param   offset - offset of the first octet to be written
* @param   complete - whether this is the last packet
* @param   oper - whether to validate and/or write attribute value  
*
* @return  Success or Failure
*/
static bStatus_t ecg_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
    bStatus_t status = SUCCESS;
    uint8 notify = 0xFF;
    
    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch ( uuid )
        {
        case ECG_ENABLE_UUID:
            //Validate the value
            // Make sure it's not a blob oper
            if ( offset == 0 )
            {
                if ( len > 1 )
                    status = ATT_ERR_INVALID_VALUE_SIZE;
                else if ( pValue[0] != FALSE && pValue[0] != TRUE )
                    status = ATT_ERR_INVALID_VALUE;
            }
            else
            {
                status = ATT_ERR_ATTR_NOT_LONG;
            }
            
            //Write the value
            if ( status == SUCCESS )
            {
                uint8 *pCurValue = (uint8 *)pAttr->pValue;
                
                *pCurValue = pValue[0];
                notify = ECG_ENABLER;        
            }             
            break;        
            
        case GATT_CLIENT_CHAR_CFG_UUID:
            status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                    offset, GATT_CLIENT_CFG_NOTIFY );
            break;
            
        default:
            status = ATT_ERR_ATTR_NOT_FOUND;
        }
    }
    
    // If an attribute changed then callback function to notify application of change
    if ((notify != 0xFF) && ecg_AppCBs && ecg_AppCBs->pfnECGEnabler) {
        ecg_AppCBs->pfnECGEnabler();
    }
    
    return ( status );
}

/*********************************************************************
* @fn          ECG_HandleConnStatusCB
*
* @brief       ECG Service link status change handler function.
*
* @param       connHandle - connection handle
* @param       changeType - type of change
*
* @return      none
*/
void ECG_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
    // Make sure this is not loopback connection
    if ( connHandle != LOOPBACK_CONNHANDLE )
    {
        // Reset Client Char Config if connection has dropped
        if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
            ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
             ( !linkDB_Up( connHandle ) ) ) )
        {
            GATTServApp_InitCharCfg( connHandle, valueConfigCoordinates );
        }
    }
}

void ECG_Notify(uint16 connHandle, uint8 *ptr)
{
    attHandleValueNoti_t ecgNotify;
    uint16 value = GATTServApp_ReadCharCfg(connHandle, valueConfigCoordinates);
    
    ecgNotify.handle = ecgAttrTbl[4].handle;
    ecgNotify.len = ECG_NOTIFY_BUFFER_SIZE;
    VOID osal_memcpy((uint8 *)&(ecgNotify.value[0]), ptr, ECG_NOTIFY_BUFFER_SIZE);
    
    if (value & GATT_CLIENT_CFG_NOTIFY) {
      GATT_Notification(connHandle, &ecgNotify, FALSE );
    }
}

/*********************************************************************
*********************************************************************/
