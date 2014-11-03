/*
 * cmd buffer is used to store temperature measurement data
 */
#ifndef __CMD_BUFFER__
#define __CMD_BUFFER__

#include "recorder.h"

#ifdef DEBUG_STATS
#define CB_CIRCULAR_BUFFER_DEPTH 8
#else
#define CB_CIRCULAR_BUFFER_DEPTH 32
#endif

// status bit flags
#define CB_WRITE_DONE           0x01

struct temp_cache {
    int16 tempValue;
    uint8 timeValid;
    UTCTimeStruct timeTick;
};
    
struct cmd_buffer {
    uint8 status_flag;                       // Index of the bit within the Byte;
    struct temp_cache t_buf;
};

struct cmd_buffer_stats {
    uint16 push_samples;
    uint16 pull_samples;
    uint16 overflow_counts;
    uint16 underflow_counts;
    uint16 bug_count;
};

/*
 * Initialize Command Buffer
 */
void CBInit(void);

/*
 * Do cleanup when new peer connected
 */
void CBCleanup(void);

/*
 * Invoked by USB TX to get the next cmd buffer
 * for USB transaction
 */
struct cmd_buffer *CBGetNextBufferForTX (int16 *tempPtr, uint8 *timeValid, UTCTimeStruct *tempTSPtr);

/*
 * Manage ref_cnt
 */
void CBPutBuffer(struct cmd_buffer *pCb);

/*
 * Invoked by sampling routine to push sampled data
 */
void CBPushTemp(int16 temp);

/*
 * Check to see if there is data available in the buffer
 */
uint8 CBDataAvailable(void);

void CBQueryStatus(uint8 *ptr);

#endif
