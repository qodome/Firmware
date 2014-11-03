/*
 * Memory dump profile
 */
#ifdef DEBUG_MEM

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "memdump.h"
#include "st_util.h"

// Memory dump service
CONST uint8 memdumpServUUID[ATT_BT_UUID_SIZE] =
{ 
    LO_UINT16(MEMDUMP_SERVICE), HI_UINT16(MEMDUMP_SERVICE)
};

// Memory dump characteristic
CONST uint8 memdumpByteUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(MEMDUMP_BYTES), HI_UINT16(MEMDUMP_BYTES)
};

static CONST gattAttrType_t memdumpService = {ATT_BT_UUID_SIZE, memdumpServUUID};

// Memdump Characteristic Properties
static uint8 memdumpBytes[18];
static uint8 memdumpByteCharProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 memType = 0;
static uint8 *memPtr = 0x0000;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t memdumpAttrTbl[] = 
{
    // Memdump Service
    { 
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8 *)&memdumpService                /* pValue */
    },
    
    // Memdump Characteristic Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &memdumpByteCharProps 
    },
    
    // ECG Enable Characteristic Value
    { 
        { ATT_BT_UUID_SIZE, memdumpByteUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        &(memdumpBytes[0])
    },
};

/*********************************************************************
* LOCAL FUNCTIONS
*/
static uint8 memdump_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t memdump_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

/*********************************************************************
* PROFILE CALLBACKS
*/
CONST gattServiceCBs_t memdumpCBs =
{
    memdump_ReadAttrCB,     // Read callback function pointer
    memdump_WriteAttrCB,    // Write callback function pointer
    NULL                    // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/
bStatus_t MemDump_AddService(uint32 services)
{
    uint8 status = SUCCESS;
    
    status = GATTServApp_RegisterService( memdumpAttrTbl, 
                                         GATT_NUM_ATTRS( memdumpAttrTbl ),
                                         &memdumpCBs );
    
    return ( status );
}

/*
 * Read attribute
 */
static uint8 memdump_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
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
    case MEMDUMP_BYTES:
        *pLen = 18;
        memdumpBytes[0] = (uint8)(((uint16)memPtr >> 8) & 0xFF);
        memdumpBytes[1] = (uint8)((uint16)memPtr & 0xFF);
        osal_memcpy(&(memdumpBytes[2]), memPtr, 16);
        VOID osal_memcpy(pValue, pAttr->pValue, 18);
        if (memType) {
            memPtr += 16;
        }
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
static bStatus_t memdump_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
    bStatus_t status = SUCCESS;
    
    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch ( uuid )
        {
        case MEMDUMP_BYTES:
            if (len == 2) {
                if (pValue[0] & 0x80) {
                    memType = 1;
                } else {
                    memType = 0;
                }
                memPtr = (uint8 *)((((uint16)pValue[0] << 8) | (uint16)pValue[1]) & 0x7FFF);
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

#endif