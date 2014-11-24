/*
 * Command buffer implementation
 * applied on ECG sampling
 */

#include "hal_mcu.h"
#include "hal_types.h"
#include "hal_defs.h"
#include "OSAL.h"
#include "cmdbuffer.h"
#include "recorder.h"
#include "OSAL_Clock.h"
#include "tempState.h"

/*
 * CB related static states
 */
static struct cmd_buffer __cb[CB_CIRCULAR_BUFFER_DEPTH];
static int16 __cb_push_idx = 0;     // write ptr
static int16 __cb_pop_idx = -1;     // read ptr
struct cmd_buffer_stats buf_stats;

/*********************** Static internal utility functions **************************/


/***********************          External APIs            **************************/
/*
 * Initialize all EXG Data Buffer staff
 */
void CBInit (void)
{
    osal_memset((void *)__cb, 0, (CB_CIRCULAR_BUFFER_DEPTH * sizeof(struct cmd_buffer)));
    osal_memset((void *)&buf_stats, 0, sizeof(struct cmd_buffer_stats));
    __cb_push_idx = 0;
    __cb_pop_idx = -1;
}

// The cleanup routine is invoked by iDo when connected to new device
void CBCleanup(void)
{
    halIntState_t intState;
    
    HAL_ENTER_CRITICAL_SECTION(intState);
    
    osal_memset((void *)__cb, 0, (CB_CIRCULAR_BUFFER_DEPTH * sizeof(struct cmd_buffer)));
    __cb_push_idx = 0;
    __cb_pop_idx = -1;    
    
    HAL_EXIT_CRITICAL_SECTION(intState);    
}

/*
 * Get the next Temperature Measurement
 */
struct cmd_buffer *CBGetNextBufferForTX (int16 *tempPtr, uint8 *validFlag, UTCTimeStruct *tempTSPtr)
{
    struct cmd_buffer *ret = NULL;
    halIntState_t intState;
    
    HAL_ENTER_CRITICAL_SECTION(intState);

    if ((__cb_pop_idx != -1) && (__cb[__cb_pop_idx].status_flag & CB_WRITE_DONE)) {
        ret = &(__cb[__cb_pop_idx]);
        *tempPtr = __cb[__cb_pop_idx].t_buf.tempValue;
        *validFlag = __cb[__cb_pop_idx].t_buf.validFlag;
        VOID osal_memcpy(tempTSPtr, &(__cb[__cb_pop_idx].t_buf.timeTick), sizeof(UTCTimeStruct));
    } else {
        buf_stats.underflow_counts++;
    }

    HAL_EXIT_CRITICAL_SECTION(intState);

    return ret;
}

/*
 * Manage ref_cnt
 */
void CBPutBuffer (struct cmd_buffer *pCb)
{
    halIntState_t intState;

    if (__cb_pop_idx == -1) {
        buf_stats.bug_count |= (1 << 1);
        return;
    }

    HAL_ENTER_CRITICAL_SECTION(intState);

    buf_stats.pull_samples++;
    pCb->status_flag &= ~(CB_WRITE_DONE);
    __cb_pop_idx = (__cb_pop_idx + 1) % CB_CIRCULAR_BUFFER_DEPTH;

    HAL_EXIT_CRITICAL_SECTION(intState);
}

/*
 * Invoked by sampling routine to push measured temperature
 */
void CBPushTemp (int16 temp)
{
    halIntState_t intState;
    UTCTimeStruct utc;
    
    if (osal_TimeInitialized()) {
        osal_ConvertUTCTime(&utc, osal_getClock());
    }
    
    HAL_ENTER_CRITICAL_SECTION(intState);

    if (__cb[__cb_push_idx].status_flag & CB_WRITE_DONE) {
        buf_stats.overflow_counts++;
    }
    
	__cb[__cb_push_idx].t_buf.validFlag = 0;
	if (osal_TimeInitialized()) {
		__cb[__cb_push_idx].t_buf.validFlag |= TIME_VALID_FLAG;
        osal_memcpy((void *)&(__cb[__cb_push_idx].t_buf.timeTick), (void *)&utc, sizeof(utc));
	}
	if (temp_state_is_attached()) {
		__cb[__cb_push_idx].t_buf.validFlag |= TYPE_VALID_FLAG;
	}    
    
    __cb[__cb_push_idx].t_buf.tempValue = temp;
    __cb[__cb_push_idx].status_flag |= CB_WRITE_DONE;
    
    // Current buffer is done, notify consumer
    if (__cb_pop_idx == -1) {
        __cb_pop_idx = __cb_push_idx;
    }
    __cb_push_idx = (__cb_push_idx + 1) % CB_CIRCULAR_BUFFER_DEPTH;
    buf_stats.push_samples++;

    HAL_EXIT_CRITICAL_SECTION(intState);
}

uint8 CBDataAvailable(void)
{
    uint8 ret = 0;
    halIntState_t intState;
    
    HAL_ENTER_CRITICAL_SECTION(intState);

    if ((__cb_pop_idx != -1) && (__cb[__cb_pop_idx].status_flag & CB_WRITE_DONE)) {
        ret = 1;
    }

    HAL_EXIT_CRITICAL_SECTION(intState);

    return ret;    
}

void CBQueryStatus (uint8 *ptr)
{
    osal_memcpy(ptr, (uint8 *)&buf_stats, sizeof(buf_stats));
}