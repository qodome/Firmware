/*
 * Statistics profile
 */
#ifndef __STATISTICS__
#define __STATISTICS__

#define STATS_SERVICE             0x00FF
#define STATS_UUID                0x00FF
#define STATS_REC_UUID            0x00FE
#define UPDATE_STATS_PERIOD                                  5000

struct stats_t {
    uint8 buf[20];
};

bStatus_t Stats_AddService( uint32 services );
extern bStatus_t Stats_SetParameter( uint8 param, uint8 len, void *value );
extern bStatus_t Stats_GetParameter( uint8 param, void *value );
extern void Stats_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

#endif
