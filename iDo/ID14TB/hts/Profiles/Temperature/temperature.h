/*
 * Temperature profile
 */
#ifndef __TEMPERATURE__
#define __TEMPERATURE__

#include "OSAL_Clock.h"

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

#define TEMP_SERVICE                0x1809
#define TEMP_UUID                   0x2A1C
#define TEMP_INTERMEDIATE           0x2A1E
#define TEMP_INTERVAL               0x2A21
#ifdef PRIVATE_TIME_UUID
#define TEMP_TIME_UUID              0xAAAA
#endif

struct temp_measure_without_ts {
    uint8 flag;
    union {
        uint32 temp;
        uint8 temp8[4];
    } u;
    uint8 temp_type;
};

struct temp_measure_with_ts {
    uint8 flag;
    union {
        uint32 temp;
        uint8 temp8[4];
    } u;
    uint16 year;
    uint8 month;
    uint8 day;
    uint8 hours;
    uint8 minutes;
    uint8 seconds; 
    uint8 temp_type;
};

struct temp_intermediate_notify {
    uint8 flag;
    union {
        uint32 temp;
        uint8 temp8[4];
    } u;    
    uint8 temp_type;    
};

#define TEMP_INTERMEDIATE_TIME_OFFSET   5
struct temp_intermediate_rw {
    uint8 flag;
    union {
        uint32 temp;
        uint8 temp8[4];
    } u;    
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

// Indicate interval update
void Temp_SendTemperature(uint16 connHandle, uint8 *bufPtr, uint8 bufLen, uint8 taskId);

void Temp_Init(void);
void Temp_Enable(void);
void Temp_Disable(void);

uint32 Temp_mill_seconds_before_next_indication(void);

uint8 Temp_Monitor(void);

uint8 Temp_TM_sending(uint16 connHandle);

#ifdef ATTACH_DETECTION
uint8 Temp_FinishPacket(uint8 *ptr, int16 temp, uint8 is_attached, UTCTimeStruct *ptc);
#else
uint8 Temp_FinishPacket(uint8 *ptr, int16 temp, UTCTimeStruct *ptc);
#endif
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
