/*
 * Current Time Profile
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"

#include "currenttime.h"
#include "st_util.h"
#include "OSAL_Clock.h"

// Current time service
CONST uint8 timeServUUID[ATT_BT_UUID_SIZE] =
{ 
    LO_UINT16(CURRENT_TIME_SERV_UUID), HI_UINT16(CURRENT_TIME_SERV_UUID)
};

// Date time characteristic
CONST uint8 timeUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(DATE_TIME_UUID), HI_UINT16(DATE_TIME_UUID)
};

static CONST gattAttrType_t timeService = {ATT_BT_UUID_SIZE, timeServUUID};

// Memdump Characteristic Properties
static UTCTimeStruct timeNow;
static uint8 timeCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t timeAttrTbl[] = 
{
    // Memdump Service
    { 
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8 *)&timeService                /* pValue */
    },
    
    // Memdump Characteristic Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &timeCharProps 
    },
    
    // ECG Enable Characteristic Value
    { 
        { ATT_BT_UUID_SIZE, timeUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        (uint8 *)&timeNow
    },
};

/*********************************************************************
* LOCAL FUNCTIONS
*/
static uint8 time_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t time_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

/*********************************************************************
* PROFILE CALLBACKS
*/
CONST gattServiceCBs_t timeCBs =
{
    time_ReadAttrCB,     // Read callback function pointer
    time_WriteAttrCB,    // Write callback function pointer
    NULL                    // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/
bStatus_t CurrentTime_AddService(uint32 services)
{
    uint8 status = SUCCESS;
    
    status = GATTServApp_RegisterService( timeAttrTbl, 
                                         GATT_NUM_ATTRS( timeAttrTbl ),
                                         &timeCBs );
    
    return ( status );
}

/*
 * Read attribute
 */
static uint8 time_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
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
    case DATE_TIME_UUID:
        *pLen = 7;
        osal_ConvertUTCTime(&timeNow, osal_getClock());
        VOID osal_memcpy(pValue, pAttr->pValue, 7);
        break;
        
    default:
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
    
    return ( status );
}

/*
 * Write attribute
 */
static bStatus_t time_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
    bStatus_t status = SUCCESS;
    
    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch ( uuid )
        {
        case DATE_TIME_UUID:
            if (len == 7) {
                osal_setClock(osal_ConvertUTCSecs((UTCTimeStruct *)pValue));
            } else {
                status = ATT_ERR_ATTR_NOT_LONG;
            }
            break;        
            
        default:
            status = ATT_ERR_ATTR_NOT_FOUND;
        }
    }
    
    return ( status );
}

