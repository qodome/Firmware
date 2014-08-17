/*
 * Temperature profile 
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
    //TI_UUID(TEMP_SERVICE)
    LO_UINT16(TEMP_SERVICE), HI_UINT16(TEMP_SERVICE)
};

// Temperature characteristic
CONST uint8 tempValueUUID[ATT_BT_UUID_SIZE] =
{
    //TI_UUID(TEMP_UUID)
    LO_UINT16(TEMP_UUID), HI_UINT16(TEMP_UUID)
};

// Time characteristic for temperature
CONST uint8 tempTimeUUID[ATT_BT_UUID_SIZE] =
{
    //TI_UUID(TEMP_UUID)
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

static tempServiceCB_t tempServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

static CONST gattAttrType_t tempService = { ATT_BT_UUID_SIZE, tempServUUID };

// Temperature Value Characteristic
static uint8 tempValueProps = GATT_PROP_NOTIFY | GATT_PROP_READ | GATT_PROP_WRITE;
struct temp tempValue;
static gattCharCfg_t valueConfigCoordinates[GATT_MAX_NUM_CONN];

static uint8 tempTimeProps = GATT_PROP_READ | GATT_PROP_WRITE;
struct calendar tempTime;
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

    // Temperature RWN Property Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &tempValueProps 
    },

    // Temperature Value
    { 
        { ATT_BT_UUID_SIZE, tempValueUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (unsigned char *)&tempValue 
    },
    
    // Notification Registration
    { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        (uint8 *)&valueConfigCoordinates 
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
        GATT_PERMIT_READ|GATT_PERMIT_WRITE, 
        0, 
        (unsigned char *)&tempTime 
    },
};

/*
 * External Functions
 */
extern void iDoTurnOnTemp(void);
extern void iDoTurnOffTemp(void);

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
    NULL                   // Authorization callback function pointer
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
    
    status = GATTServApp_RegisterService( tempAttrTbl, 
      GATT_NUM_ATTRS( tempAttrTbl ),
      &tempCBs );

    return ( status );
}

/*********************************************************************
 * @fn      Temp_Register
 *
 * @brief   Register a callback function with the Temperature Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void Temp_Register( tempServiceCB_t pfnServiceCB )
{
    tempServiceCB = pfnServiceCB;
}

// Callback function invoked by app logic
// to notify temperature update
void Temp_NotifyTemperature(uint16 connHandle, struct temp *tBuf)
{
    attHandleValueNoti_t tempNotify;
    uint16 value = GATTServApp_ReadCharCfg(connHandle, valueConfigCoordinates);
    
    tempNotify.handle = tempAttrTbl[2].handle;
    tempNotify.len = sizeof(struct temp);
    VOID osal_memcpy((uint8 *)&(tempNotify.value[0]), (uint8 *)tBuf, sizeof(struct temp));
    
    if (value & GATT_CLIENT_CFG_NOTIFY) {
      GATT_Notification(connHandle, &tempNotify, FALSE );
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
    struct calendar tc;
    int16 t_raw;
    int32 t32 = 0;
    struct temp t;
    int32 sign = -4;
    
    if (utilExtractUuid16(pAttr,&uuid) == FAILURE)
    {
      // Invalid handle
      *pLen = 0;
      return ATT_ERR_INVALID_HANDLE;
    }
       
    switch (uuid) {
        case TEMP_UUID:
            if (enable_fast_read == 0) {
                enable_fast_read = 1;
                iDo_UpdateFastParameter();
            }
            *pLen = sizeof(struct temp_ts);
            osal_memset(&tc, 0, sizeof(struct calendar));
            t_raw = recorder_get_temperature(&tc);
            t.flag = 0x02;
            if (t_raw & 0x8000) {
                t32 = (int32)t_raw | 0xFFFF0000;
            } else {
                t32 = t_raw;
            }
            t.u.temp = (sign << 24) | ((t32 * 625) & 0xFFFFFF);
            VOID osal_memcpy(pValue, (void *)&t, sizeof(struct temp));
            VOID osal_memcpy(pValue + sizeof(struct temp), (void *)&tc, sizeof(struct calendar));
            break;
        
        case TEMP_TIME_UUID:
            osal_memset((void *)&tc, 0, sizeof(struct calendar));
            recorder_get_calendar_time(&tc);
            *pLen = sizeof(struct calendar);
            VOID osal_memcpy(pValue, (void *)&tc, sizeof(struct calendar));
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
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );

        if (GATTServApp_ReadCharCfg(connHandle, valueConfigCoordinates) & GATT_CLIENT_CFG_NOTIFY) {
          iDoTurnOnTemp();
        } else {
          iDoTurnOffTemp();
        }
        break;
              
      case TEMP_UUID:
        if (len != sizeof(struct temp_ts)) {
          return ATT_ERR_INVALID_VALUE_SIZE;
        }
        recorder_set_read_base_ts((struct calendar *)(pValue + sizeof(struct temp)));
        status = SUCCESS;
        break;
        
      case TEMP_TIME_UUID:
        if (len != sizeof(struct calendar)) {
          return ATT_ERR_INVALID_VALUE_SIZE;
        }
        recorder_set_calendar_time((struct calendar *)pValue);
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
void Temp_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
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
      enable_fast_read = 0;
    }
  }
}

void Temp_Init(void)
{
  softwareSPIInit();
  spi_write(0x01, 0x6060);
}

void Temp_Enable(void)
{
  spi_write(0x01, 0x4040);
}

void Temp_Disable(void)
{
  spi_write(0x01, 0x6060);
}

/*********************************************************************
 *********************************************************************/
