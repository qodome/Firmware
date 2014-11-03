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

CONST uint8 tempTimeUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(TEMP_TIME_UUID), HI_UINT16(TEMP_TIME_UUID)
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
static thermometerIRange_t  thermometerIRange = {1, 60};

// Timestamp related setting
static uint8 tempTimeProps = GATT_PROP_READ | GATT_PROP_WRITE;
UTCTimeStruct tempTime;
static int enable_fast_read = 0;

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
    enable_fast_read = 0;
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
#ifdef DEBUG_STATS
            s.pkt_count++;
#endif  
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
#ifdef DEBUG_STATS
            s.pkt_count++;
#endif     
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
    uint16 uuid = 0;
    UTCTimeStruct tc;
    int16 t_raw;
    int32 t32 = 0;
    struct temp_intermediate_rw t;
    int32 sign = -4;    
    
    if (utilExtractUuid16(pAttr,&uuid) == FAILURE)
    {
        // Invalid handle
        *pLen = 0;
        return ATT_ERR_INVALID_HANDLE;
    }
    
    switch (uuid) 
    {
    case TEMP_INTERMEDIATE:
        if (enable_fast_read == 0) {
            enable_fast_read = 1;
            iDo_UpdateFastParameter();
        }
        *pLen = sizeof(struct temp_intermediate_rw);
        osal_memset(&tc, 0, sizeof(UTCTimeStruct));
        t_raw = recorder_get_temperature(&tc);
        VOID osal_memcpy(((uint8 *)&t + TEMP_INTERMEDIATE_TIME_OFFSET), (uint8 *)&tc, sizeof(UTCTimeStruct));
        t.flag = 0x02;
        if (t_raw & 0x8000) {
            t32 = (int32)t_raw | 0xFFFF0000;
        } else {
            t32 = t_raw;
        }
        t.u.temp = (sign << 24) | ((t32 * 625) & 0xFFFFFF);
        VOID osal_memcpy(pValue, (uint8 *)&t, sizeof(struct temp_intermediate_rw));
        break;
    
    case TEMP_INTERVAL:
        *pLen = sizeof(tempInterval);
        VOID osal_memcpy(pValue, (void *)&tempInterval, sizeof(tempInterval));
        break;
        
    case TEMP_TIME_UUID:
        osal_memset((void *)&tc, 0, sizeof(UTCTimeStruct));
        if (osal_TimeInitialized()) {
            osal_ConvertUTCTime(&tc, osal_getClock());
        }
        *pLen = sizeof(UTCTimeStruct);
        VOID osal_memcpy(pValue, (void *)&tc, sizeof(UTCTimeStruct));
        break;
        
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
            if (len != sizeof(struct temp_intermediate_rw)) {
                return ATT_ERR_INVALID_VALUE_SIZE;
            }
            recorder_set_read_base_ts((UTCTimeStruct *)(pValue + TEMP_INTERMEDIATE_TIME_OFFSET));
            status = SUCCESS;
            break;
            
        case TEMP_INTERVAL:
            if (len != sizeof(tempInterval)) {
                return ATT_ERR_INVALID_VALUE_SIZE;
            }
            tempInterval = *(uint16 *)pValue;
            status = SUCCESS;
            break;
            
        case TEMP_TIME_UUID:
            if (len != sizeof(UTCTimeStruct)) {
                return ATT_ERR_INVALID_VALUE_SIZE;
            }
            
            osal_setClock(osal_ConvertUTCSecs((UTCTimeStruct *)pValue));
            status = SUCCESS;
            break;
            
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
            enable_fast_read = 0;
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
/*********************************************************************
*********************************************************************/
