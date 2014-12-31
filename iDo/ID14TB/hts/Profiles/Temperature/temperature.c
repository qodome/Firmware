/*
 * Temperature profile 
 * We are going to strictly follow Health Thermometer Service:
 * http://www.bluetooth.org/docman/handlers/downloaddoc.ashx?doc_id=238688
 * & Health Thermometer Profile
 * http://www.bluetooth.org/docman/handlers/downloaddoc.ashx?doc_id=238687
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

#include "temperature.h"
#include "SoftwareSPI.h"
#include "st_util.h"
#include "recorder.h"
#include "iDo.h"
#include "OSAL_Clock.h"
#include "memdump.h"
#include <string.h>

#ifdef DEBUG_STATS
#include "statistics.h"
#endif

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
// Temperature service
CONST uint8 tempServUUID[ATT_BT_UUID_SIZE] =
{ 
    LO_UINT16(TEMP_SERVICE), HI_UINT16(TEMP_SERVICE)
};

// Temperature characteristic
CONST uint8 tempValueUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(TEMP_UUID), HI_UINT16(TEMP_UUID)
};

CONST uint8 tempIntermediateUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(TEMP_INTERMEDIATE), HI_UINT16(TEMP_INTERMEDIATE)
};

CONST uint8 tempIntervalUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(TEMP_INTERVAL), HI_UINT16(TEMP_INTERVAL)
};

CONST uint8 thermometerIRangeUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(GATT_VALID_RANGE_UUID), HI_UINT16(GATT_VALID_RANGE_UUID)
};

#ifdef PRIVATE_TIME_UUID
CONST uint8 tempTimeUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(TEMP_TIME_UUID), HI_UINT16(TEMP_TIME_UUID)
};
#endif

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/

/*********************************************************************
* Profile Attributes - variables
*/

static CONST gattAttrType_t tempService = { ATT_BT_UUID_SIZE, tempServUUID };

// Temperature measurement Characteristic
static uint8 tempValueProps = GATT_PROP_INDICATE;
struct temp_measure_with_ts tempValue;
static gattCharCfg_t valueConfigCoordinates[GATT_MAX_NUM_CONN];

// Temperature intermediate Characteristic
static uint8 tempIntermediateProps = GATT_PROP_NOTIFY | GATT_PROP_READ | GATT_PROP_WRITE;
struct temp_intermediate_rw tempIntermediate;
static gattCharCfg_t intermediateConfigCoordinates[GATT_MAX_NUM_CONN];

// Temperature interval Characteristic
static uint8 tempIntervalProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint16 tempInterval = 30;

// Measurement Interval Range
static thermometerIRange_t  thermometerIRange = {2, 300};

#ifdef PRIVATE_TIME_UUID
// Timestamp related setting
static uint8 tempTimeProps = GATT_PROP_READ | GATT_PROP_WRITE;
UTCTimeStruct tempTime;
#endif

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t tempAttrTbl[] = 
{
    // Temperature Service
    { 
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8 *)&tempService                /* pValue */
    },
    
    // Temperature measure Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &tempValueProps 
    },
    
    { 
        { ATT_BT_UUID_SIZE, tempValueUUID },
        0, 
        0, 
        (unsigned char *)&tempValue 
    },
    
    { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        (uint8 *)&valueConfigCoordinates 
    },    
    
    // Intermediate temperature Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &tempIntermediateProps 
    },
    
    { 
        { ATT_BT_UUID_SIZE, tempIntermediateUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (unsigned char *)&tempIntermediate 
    },
    
    { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        (uint8 *)&intermediateConfigCoordinates 
    },    
    
    // Measurement interval Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &tempIntervalProps 
    },
    
    { 
        { ATT_BT_UUID_SIZE, tempIntervalUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (unsigned char *)&tempInterval 
    },
       
    // Valid range for 
    { 
      { ATT_BT_UUID_SIZE, thermometerIRangeUUID },
      GATT_PERMIT_READ,
      0, 
      (uint8 *)&thermometerIRange 
    },
    
#ifdef PRIVATE_TIME_UUID
    // Initialize UNIX Time Property Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &tempTimeProps 
    },
    
    // UNIX Time
    { 
        { ATT_BT_UUID_SIZE, tempTimeUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (unsigned char *)&tempTime 
    },
#endif
};

/*********************************************************************
* LOCAL FUNCTIONS
*/
static uint8 temp_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                             uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t temp_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                  uint8 *pValue, uint8 len, uint16 offset );

/*********************************************************************
* PROFILE CALLBACKS
*/
CONST gattServiceCBs_t tempCBs =
{
    temp_ReadAttrCB,  // Read callback function pointer
    temp_WriteAttrCB, // Write callback function pointer
    NULL              // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      Temp_AddService
*
* @brief   Initializes the Heart Rate service by registering
*          GATT attributes with the GATT server.
*
* @param   services - services to add. This is a bit map and can
*                     contain more than one service.
*
* @return  Success or Failure
*/
bStatus_t Temp_AddService( uint32 services )
{
    uint8 status = SUCCESS;
    
    VOID linkDB_Register( Temp_HandleConnStatusCB );
    
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, valueConfigCoordinates );  
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, intermediateConfigCoordinates );  
    tempInterval = 30;
    
    status = GATTServApp_RegisterService( tempAttrTbl, 
                                         GATT_NUM_ATTRS( tempAttrTbl ),
                                         &tempCBs );
    
    return ( status );
}

// Application logic notify intermediate temperature
void Temp_NotifyTemperature(uint16 connHandle, uint8 *bufPtr, uint8 bufLen)
{
    attHandleValueNoti_t tempNotify;
    uint16 value = GATTServApp_ReadCharCfg(connHandle, intermediateConfigCoordinates);
    
    if (value & GATT_CLIENT_CFG_NOTIFY) {
        tempNotify.handle = tempAttrTbl[5].handle;
        tempNotify.len = bufLen;
        VOID osal_memcpy((uint8 *)&(tempNotify.value[0]), bufPtr, bufLen);
    
        if (GATT_Notification(connHandle, &tempNotify, FALSE) == SUCCESS) {
/*
#ifdef DEBUG_STATS
            s.pkt_count++;
#endif
*/  
        }
    }
}

// Application logic indicates the measured temperature
void Temp_IndicateTemperature(uint16 connHandle, uint8 *bufPtr, uint8 bufLen, uint8 taskId)
{
    attHandleValueInd_t tempIndication;
    uint16 value = GATTServApp_ReadCharCfg(connHandle, valueConfigCoordinates);
    
    if (value & GATT_CLIENT_CFG_INDICATE) {
        tempIndication.handle = tempAttrTbl[2].handle;
        tempIndication.len = bufLen;
        VOID osal_memcpy((uint8 *)&(tempIndication.value[0]), bufPtr, bufLen);
    
        if (GATT_Indication(connHandle, &tempIndication, FALSE, taskId) == SUCCESS) {
/*
#ifdef DEBUG_STATS
            s.pkt_count++;
#endif
*/            
        }
    }
}

/*********************************************************************
* @fn          temp_ReadAttrCB
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
static uint8 temp_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                             uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
    bStatus_t status = SUCCESS;
    uint16 uuid;
#ifdef PRIVATE_TIME_UUID
    UTCTimeStruct tc;
#endif
    struct query_criteria q;    
    
    if (utilExtractUuid16(pAttr,&uuid) == FAILURE)
    {
        // Invalid handle
        *pLen = 0;
        return ATT_ERR_INVALID_HANDLE;
    }
    
    switch (uuid) 
    {
    case TEMP_INTERMEDIATE:
        *pLen = sizeof(struct query_criteria);
        recorder_get_query_result(&q);
        VOID osal_memcpy(pValue, (uint8 *)&q, sizeof(struct query_criteria));
        break;
    
    case TEMP_INTERVAL:
        *pLen = sizeof(tempInterval);
        VOID osal_memcpy(pValue, (void *)&tempInterval, sizeof(tempInterval));
        break;
        
#ifdef PRIVATE_TIME_UUID
    case TEMP_TIME_UUID:
        osal_memset((void *)&tc, 0, sizeof(UTCTimeStruct));
        if (osal_TimeInitialized()) {
            osal_ConvertUTCTime(&tc, osal_getClock());
        }
        *pLen = sizeof(UTCTimeStruct);
        VOID osal_memcpy(pValue, (void *)&tc, sizeof(UTCTimeStruct));
        break;
#endif
        
    default:
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
    
    return ( status );
}

/*********************************************************************
* @fn      temp_WriteAttrCB
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
static bStatus_t temp_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                  uint8 *pValue, uint8 len, uint16 offset )
{
    uint16 newInterval;
    bStatus_t status = ATT_ERR_ATTR_NOT_FOUND;
    
    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch ( uuid )
        {
        case GATT_CLIENT_CHAR_CFG_UUID:
            // Validate/Write Temperature measurement setting
            if (pAttr->handle == tempAttrTbl[3].handle) {
                status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                        offset, GATT_CLIENT_CFG_INDICATE);
                
                if (GATTServApp_ReadCharCfg(connHandle, valueConfigCoordinates) & GATT_CLIENT_CFG_INDICATE) {
                    iDoTurnOnTemp();
                    iDo_turn_on_measurement_indication();
                } else {
                    iDo_turn_off_measurement_indication();
                    if (!(GATTServApp_ReadCharCfg(connHandle, intermediateConfigCoordinates) & GATT_CLIENT_CFG_NOTIFY)) {
                        iDoTurnOffTemp();                        
                    }
                }    
            } else if (pAttr->handle == tempAttrTbl[6].handle) {
                status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                        offset, GATT_CLIENT_CFG_NOTIFY);
                
                if (GATTServApp_ReadCharCfg(connHandle, intermediateConfigCoordinates) & GATT_CLIENT_CFG_NOTIFY) {
                    iDoTurnOnTemp();
                } else {
                    if (!(GATTServApp_ReadCharCfg(connHandle, valueConfigCoordinates) & GATT_CLIENT_CFG_INDICATE)) {
                        iDoTurnOffTemp();                        
                    }
                }    
            }            
            break;
            
        case TEMP_INTERMEDIATE:
            if (len != sizeof(struct query_criteria)) {
                return ATT_ERR_INVALID_VALUE_SIZE;
            }
            if (osal_memcmp(pValue, "_QoDoMe_2014", 12) == TRUE) {
                iDo_FastReadUpdateParameter();
#ifndef DEBUG_STATS
                MemDump_AddService();
#endif
            } else {
                recorder_set_query_criteria((struct query_criteria *)pValue);
            }
            status = SUCCESS;
            break;
            
        case TEMP_INTERVAL:
            if (len != sizeof(tempInterval)) {
                return ATT_ERR_INVALID_VALUE_SIZE;
            }
            newInterval = *(uint16 *)pValue;
            if (newInterval >= 2 && newInterval <= 300) {
                tempInterval = *(uint16 *)pValue;
                status = SUCCESS;
            } else {
                status = ATT_ERR_INVALID_PDU;
            }
            break;
            
#ifdef PRIVATE_TIME_UUID
        case TEMP_TIME_UUID:
            if (len != sizeof(UTCTimeStruct)) {
                return ATT_ERR_INVALID_VALUE_SIZE;
            }
            
            osal_setClock(osal_ConvertUTCSecs((UTCTimeStruct *)pValue));
            status = SUCCESS;
            break;
#endif
            
        default:
            status = ATT_ERR_ATTR_NOT_FOUND;
        }
    }
    return ( status );
}

/*********************************************************************
* @fn          Temp_HandleConnStatusCB
*
* @brief       Heart Rate Service link status change handler function.
*
* @param       connHandle - connection handle
* @param       changeType - type of change
*
* @return      none
*/
void Temp_HandleConnStatusCB(uint16 connHandle, uint8 changeType)
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
            GATTServApp_InitCharCfg( connHandle, intermediateConfigCoordinates );
        }
    }
}

void Temp_Init(void)
{
    softwareSPIInit();
    // shutdown command will be sent after interface reset
}

void Temp_Enable(void)
{
    spi_write(0x01, 0x4040);
}

void Temp_Disable(void)
{
    spi_write(0x01, 0x6060);
}

uint32 Temp_mill_seconds_before_next_indication(void)
{
    return (uint32)tempInterval * (uint32)1000;
}

uint8 Temp_Monitor(void)
{
    return (uint8)(spi_read(0x01) & 0xFF);
}

uint8 Temp_TM_sending(uint16 connHandle)
{
    uint16 value = GATTServApp_ReadCharCfg(connHandle, valueConfigCoordinates);
    
    if (value & GATT_CLIENT_CFG_INDICATE) {
        return 1;
    } else {
        return 0;
    }
}

uint8 Temp_FinishPacket(uint8 *ptr, int16 temp, uint8 round, uint8 is_attached, UTCTimeStruct *ptc)
{
    int32 temp32;
    int32 temp32_delta;
    uint32 temp32u;
    int32 sign = -4;    
    uint8 negative;
    uint8 len;
    
    if (temp & 0x8000) {
        negative = 1;
        temp32 = (int32)temp | 0xFFFF0000;
        if (round == 1) {
            temp32 = 0 - temp32;
        }
    } else {
        negative = 0;
        temp32 = temp;           
    }        

    temp32 = temp32 * 625;    
    if (round == 1) {
        temp32_delta = (temp32 / 1000) * 1000;
        temp32_delta = temp32 - temp32_delta;
        temp32 = (temp32 / 1000) * 1000;
        if (temp32_delta >= 500) {
            temp32 += 1000;                
        }
        if (negative == 1) {
            temp32 = 0 - temp32;
        }
    }
        
    temp32u = (sign << 24) | (temp32 & 0xFFFFFF);
    
    ptr[0] = 0x00;
    len = 5;
#ifdef ATTACH_DETECTION
    if (is_attached) {
        ptr[0] |= 0x04;
        len += 1;
    }
#endif
    if (ptc != NULL) {
        ptr[0] |= 0x02;
        len += 7;
    }
    osal_memcpy(&(ptr[1]), (uint8 *)&temp32u, sizeof(temp32u));
    if (ptc != NULL) {
        osal_memcpy(&(ptr[5]), (uint8 *)ptc, sizeof(UTCTimeStruct));
#ifdef ATTACH_DETECTION
        if (is_attached) {
            ptr[12] = 0x02;
        }
    } else if (is_attached) {
        ptr[5] = 0x02;
#endif
    }
    return len;
}

/*********************************************************************
*********************************************************************/
