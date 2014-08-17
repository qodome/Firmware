/*
 * Temperature profile
 */
#ifndef __TEMPERATURE__
#define __TEMPERATURE__

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

#define TEMP_SERVICE             0x1809
#define TEMP_UUID                0x2A1C
#define TEMP_TIME_UUID           0xAAAA

struct temp {
  uint8 flag;
  union {
    uint32 temp;
    uint8 temp8[4];
  } u;
};

struct temp_ts {
    struct temp;
    uint16 year;
    uint8 month;
    uint8 day;
    uint8 hours;
    uint8 minutes;
    uint8 seconds;
};
  
/*********************************************************************
 * TYPEDEFS
 */

// Temperature Service callback function
typedef void (*tempServiceCB_t)(uint8 event);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * Temp_AddService- Initializes the Temperature service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

bStatus_t Temp_AddService( uint32 services );

/*
 * Temp_Register - Register a callback function with the
 *          Temperature Service
 *
 * @param   pfnServiceCB - Callback function.
 */

extern void Temp_Register( tempServiceCB_t pfnServiceCB );

/*
 * Temp_SetParameter - Set a Temp parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t Temp_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * Temp_GetParameter - Get a Temp parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t Temp_GetParameter( uint8 param, void *value );

/*********************************************************************
 * @fn          HeartRate_HandleConnStatusCB
 *
 * @brief       Heart Rate Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
extern void Temp_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

// Callback function invoked by app logic
// to notify temperature update
void Temp_NotifyTemperature(uint16 connHandle, struct temp *tBuf);
void Temp_Init(void);
void Temp_Enable(void);
void Temp_Disable(void);
int Temp_ForceUpdate();

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
