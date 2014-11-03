/*
 * Statistics profile 
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "statistics.h"
#if defined FEATURE_OAD
#include "oad.h"
#include "oad_target.h"
#endif
#include "st_util.h"
#include "cmdbuffer.h"
#include "hal_board_cfg.h"

// Statistics service
CONST uint8 statsServUUID[ATT_BT_UUID_SIZE] =
{ 
    LO_UINT16(STATS_SERVICE), HI_UINT16(STATS_SERVICE)
};

// Stats characteristic
CONST uint8 statsValueUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(STATS_UUID), HI_UINT16(STATS_UUID)
};

// Stats characteristic
CONST uint8 statsRecUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(STATS_REC_UUID), HI_UINT16(STATS_REC_UUID)
};

static CONST gattAttrType_t statsService = { ATT_BT_UUID_SIZE, statsServUUID };

// Statistics Value Characteristic
static uint8 statsValueProps = GATT_PROP_READ;
static struct stats_t statsLocal1;

static uint8 statsRecProps = GATT_PROP_READ;
static struct stats_t statsLocal2;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t statsAttrTbl[] = 
{
    // Statistics Service
    { 
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8 *)&statsService                /* pValue */
    },

    // RW Register Value Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &statsValueProps 
    },

    // RW Register Value
    { 
        { ATT_BT_UUID_SIZE, statsValueUUID },
        GATT_PERMIT_READ, 
        0, 
        (unsigned char *)&statsLocal1
    },

    // Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &statsRecProps 
    },

    // Debug REC Value
    { 
        { ATT_BT_UUID_SIZE, statsRecUUID },
        GATT_PERMIT_READ, 
        0, 
        (unsigned char *)&statsLocal2 
    },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 stats_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
        uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t stats_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
        uint8 *pValue, uint8 len, uint16 offset );

/*********************************************************************
 * PROFILE CALLBACKS
 */
CONST gattServiceCBs_t statsCBs =
{
    stats_ReadAttrCB,  // Read callback function pointer
    stats_WriteAttrCB, // Write callback function pointer
    NULL                   // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

bStatus_t Stats_AddService( uint32 services )
{
    uint8 status = SUCCESS;

    status = GATTServApp_RegisterService( statsAttrTbl, 
      GATT_NUM_ATTRS( statsAttrTbl ),
      &statsCBs );

    return ( status );
}

extern uint16 skip_count;

static uint8 stats_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
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
    case STATS_UUID:
        *pLen = sizeof(struct stats_t);
        CBQueryStatus((uint8 *)&statsLocal1);
        VOID osal_memcpy(pValue, pAttr->pValue, sizeof(struct stats_t));
        break;
        
    case STATS_REC_UUID:
        *pLen = sizeof(struct stats_t);
        CBQueryStatus2((uint8 *)&statsLocal2);
        *(uint16 *)(&statsLocal2.buf[18]) = skip_count;
        VOID osal_memcpy(pValue, pAttr->pValue, sizeof(struct stats_t));
        break;
        
    default:
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
    
    return ( status );
}

static bStatus_t stats_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
        uint8 *pValue, uint8 len, uint16 offset )
{
    bStatus_t status = ATT_ERR_ATTR_NOT_FOUND;

    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
        default:
            status = ATT_ERR_ATTR_NOT_FOUND;
    }
    return ( status );
}

/*********************************************************************
 *********************************************************************/
