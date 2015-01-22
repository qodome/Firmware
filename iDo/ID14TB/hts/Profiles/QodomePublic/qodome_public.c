/*
 * Qodome public service implementations
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "qodome_public.h"
#include "st_util.h"
#include "customize.h"
#include "peripheral.h"
#include "gap.h"
#include "gapgattserver.h"

// Qodome public service
CONST uint8 qodomeServUUID[ATT_BT_UUID_SIZE] =
{ 
    LO_UINT16(QODOME_PUBLIC_SERVICE), HI_UINT16(QODOME_PUBLIC_SERVICE)
};

// Qodome set name characteristic
CONST uint8 qodomeSetNameUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(QODOME_PUBLIC_SET_NAME), HI_UINT16(QODOME_PUBLIC_SET_NAME)
};

static CONST gattAttrType_t qodomeService = {ATT_BT_UUID_SIZE, qodomeServUUID};

// Qodome Characteristic Properties
static uint8 qodomeSetName[20];
static uint8 qodomeSetNameProps = GATT_PROP_WRITE;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t qodomeAttrTbl[] = 
{
    // Qodome Service
    { 
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8 *)&qodomeService                /* pValue */
    },
    
    // Qodome Characteristic Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &qodomeSetNameProps
    },
    
    // Qodome SetName Characteristic
    { 
        { ATT_BT_UUID_SIZE, qodomeSetNameUUID },
        GATT_PERMIT_WRITE, 
        0,
        &(qodomeSetName[0])
    },
};

/*********************************************************************
* LOCAL FUNCTIONS
*/
static bStatus_t qodome_WriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset);

/*********************************************************************
* PROFILE CALLBACKS
*/
CONST gattServiceCBs_t qodomeCBs =
{
    NULL,     // Read callback function pointer
    qodome_WriteAttrCB,    // Write callback function pointer
    NULL                    // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/
bStatus_t QodomePublic_AddService(void)
{
    return GATTServApp_RegisterService(qodomeAttrTbl, 
                                        GATT_NUM_ATTRS(qodomeAttrTbl),
                                        &qodomeCBs);
}

/*
 * Write attribute
 */
static bStatus_t qodome_WriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset)
{
    bStatus_t status = SUCCESS;
    uint8 idx, newName[24], scanRspData[24];

    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch ( uuid )
        {
        case QODOME_PUBLIC_SET_NAME:
            if (offset == 0) {
                if (len > 20) {
                    len = 20;
                }
                for (idx = 0; idx < len; idx++) {
                    newName[idx] = pValue[idx];
                }
                newName[idx] = 0;

                custom_set_dn(newName);
                GGS_SetParameter(GGS_DEVICE_NAME_ATT, (len + 1), newName);

                scanRspData[0] = len + 1;
                scanRspData[1] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
                for (uint8 idx = 0; idx < len; idx++) {
                    scanRspData[2 + idx] = pValue[idx];
                }
                GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, (scanRspData[0] + 1), scanRspData);
            }

            break;        
            
        default:
            status = ATT_ERR_ATTR_NOT_FOUND;
        }
    }
    
    return status;
}
