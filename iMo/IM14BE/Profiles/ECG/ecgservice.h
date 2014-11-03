/*
 * ECG profile
 */
#ifndef __ECG_SERVICE__
#define __ECG_SERVICE__

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

#define ECG_SERVICE                     0xFFFF
#define ECG_ENABLE_UUID                 0xFFFE   
#define ECG_UUID                        0xFFFF
  
// Profile Parameters
#define ECG_ENABLER                 0  // RW uint8 - Profile Attribute value
#define ECG_VALUE                   1

/*********************************************************************
 * TYPEDEFS
 */
  
// Callback triggered when Enabler property has been changed
typedef void (*ecgEnabler_t)( void );

typedef struct
{
  ecgEnabler_t pfnECGEnabler;  // Called when Enabler attribute changes
} ecgCBs_t;

// ECG Service callback function
typedef void (*ecgServiceCB_t)(uint8 event);

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
 * ECG_AddService- Initializes the ECG service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

bStatus_t ECG_AddService( uint32 services );

/*
 * ECG_Register - Register a callback function with the
 *          ECG Service
 *
 * @param   pfnServiceCB - Callback function.
 */

extern void ECG_Register( ecgServiceCB_t pfnServiceCB );

/*
 * ECG_SetParameter - Set a ECG parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t ECG_SetParameter( uint8 param, uint8 len, uint8 *value);
  
/*
 * ECG_GetParameter - Get a Temp parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t ECG_GetParameter( uint8 param, void *value );

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
extern void ECG_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

bStatus_t ECG_RegisterAppCBs( ecgCBs_t *appCallbacks );

// Callback function invoked by app logic
// to notify ECG update
extern void ECG_Notify(uint16 connHandle, uint8 *ptr);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
