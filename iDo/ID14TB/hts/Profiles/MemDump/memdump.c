/*
 * Memory dump profile
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "memdump.h"
#include "st_util.h"
#include "hal_flash.h"

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
static uint8 memdumpBytes[20];
static uint8 memdumpByteCharProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 memType = 0;
static uint8 memFlash = 0;
static uint32 memPtr = 0x00000000;
static uint8 deleteService = 0;

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
uint8 MemDump_ServiceNeedDelete(void)
{
    return deleteService;
}

bStatus_t MemDump_AddService(void)
{
    uint8 status;
    
    status = GATTServApp_RegisterService( memdumpAttrTbl, 
                                         GATT_NUM_ATTRS( memdumpAttrTbl ),
                                         &memdumpCBs );
    if (status == SUCCESS) {
        deleteService = 1;
    }
    return ( status );
}

bStatus_t MemDump_DelService(void)
{
    uint8 status;
    gattAttribute_t *pServ;
    
    status = GATTServApp_DeregisterService(GATT_SERVICE_HANDLE(memdumpAttrTbl), &pServ);
    if (status == SUCCESS) {
        deleteService = 0;
    }
    return status;
}

/*
 * Read attribute
 */
static uint8 memdump_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
    bStatus_t status = SUCCESS;
    uint16 uuid;
    uint8 flash_pg;
    uint16 flash_offset;
    uint16 ptr;
    
    if (utilExtractUuid16(pAttr,&uuid) == FAILURE)
    {
        // Invalid handle
        *pLen = 0;
        return ATT_ERR_INVALID_HANDLE;
    }
    
    switch (uuid) {
    case MEMDUMP_BYTES:
        *pLen = 20;
        osal_memcpy(memdumpBytes, (uint8 *)&memPtr, 4);
        ptr = (uint16)memPtr;
        if (memFlash == 0) {
            osal_memcpy(&(memdumpBytes[4]), (void *)ptr, 16);
        } else {
            flash_pg = (uint8)(memPtr/2048);
            if (flash_pg < 128) {
                flash_offset = (uint16)(memPtr - (uint32)flash_pg * 2048);
                HalFlashRead(flash_pg, flash_offset, &(memdumpBytes[4]), 16);
            }
        }
        VOID osal_memcpy(pValue, pAttr->pValue, 20);
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
            if (len == 4) {
                if (pValue[0] & 0x80) {
                    memType = 1;
                } else {
                    memType = 0;
                }
                if (pValue[0] & 0x40) {
                    memFlash = 1;
                } else {
                    memFlash = 0;
                }
                memPtr = (uint32)((((uint32)pValue[0] << 24) | ((uint32)pValue[1] << 16) | ((uint32)pValue[2] << 8) | (uint32)pValue[3]) & 0x3FFFFFFF);
            } else {
                status = ATT_ERR_ATTR_NOT_LONG;
            }
            break;        
            
        default:
            status = ATT_ERR_ATTR_NOT_FOUND;
        }
    }
    
    return status;
}
