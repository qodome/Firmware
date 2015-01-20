/*
 * Command buffer implementation
 * applied on ECG sampling
 */
#include <stdlib.h>
#include <string.h>
#include "nrf_types.h"
#include "cmd_buffer.h"
#include "recorder.h"
#include "temp_state.h"
#include "nrf_soc.h"
#include "ble_time.h"
#include "UTCtime_convert.h"

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
    memset((void *)__cb, 0, (CB_CIRCULAR_BUFFER_DEPTH * sizeof(struct cmd_buffer)));
    memset((void *)&buf_stats, 0, sizeof(struct cmd_buffer_stats));
    __cb_push_idx = 0;
    __cb_pop_idx = -1;
}

#ifdef DELETE_INFO_UNKNOWN_PEER
// The cleanup routine is invoked by iDo when connected to new device
void CBCleanup(void)
{
    halIntState_t intState;

    HAL_ENTER_CRITICAL_SECTION(intState);

    memset((void *)__cb, 0, (CB_CIRCULAR_BUFFER_DEPTH * sizeof(struct cmd_buffer)));
    __cb_push_idx = 0;
    __cb_pop_idx = -1;

    HAL_EXIT_CRITICAL_SECTION(intState);
}
#endif

/*
 * Get the next Temperature Measurement
 */
struct cmd_buffer *CBGetNextBufferForTX (int16 *tempPtr, uint8 *validFlag, ble_date_time_t *tempTSPtr)
{
    struct cmd_buffer *ret = NULL;
    uint8_t nested_critical_region = 0;

    sd_nvic_critical_region_enter(&nested_critical_region);

    if ((__cb_pop_idx != -1) && (__cb[__cb_pop_idx].status_flag & CB_WRITE_DONE)) {
        ret = &(__cb[__cb_pop_idx]);
        *tempPtr = __cb[__cb_pop_idx].t_buf.tempValue;
        *validFlag = __cb[__cb_pop_idx].t_buf.validFlag;
        memcpy(tempTSPtr, &(__cb[__cb_pop_idx].t_buf.timeTick), sizeof(ble_date_time_t));
    } else {
        buf_stats.underflow_counts++;
    }

    sd_nvic_critical_region_exit(nested_critical_region);

    return ret;
}

/*
 * Manage ref_cnt
 */
void CBPutBuffer (struct cmd_buffer *pCb)
{
	uint8_t nested_critical_region = 0;

    if (__cb_pop_idx == -1) {
        buf_stats.bug_count |= (1 << 1);
        return;
    }

    sd_nvic_critical_region_enter(&nested_critical_region);

    buf_stats.pull_samples++;
    pCb->status_flag &= ~(CB_WRITE_DONE);
    __cb_pop_idx = (__cb_pop_idx + 1) % CB_CIRCULAR_BUFFER_DEPTH;

    sd_nvic_critical_region_exit(nested_critical_region);
}

/*
 * Invoked by sampling routine to push measured temperature
 */
void CBPushTemp (int16 temp)
{
	uint8_t nested_critical_region = 0;
    ble_date_time_t utc;

    if (date_time_initialized()) {
        osal_ConvertUTCTime(&utc, date_time_get_wall());
    }

    sd_nvic_critical_region_enter(&nested_critical_region);

    if (__cb[__cb_push_idx].status_flag & CB_WRITE_DONE) {
        buf_stats.overflow_counts++;
    }

	__cb[__cb_push_idx].t_buf.validFlag = 0;
	if (date_time_initialized()) {
		__cb[__cb_push_idx].t_buf.validFlag |= TIME_VALID_FLAG;
        memcpy((void *)&(__cb[__cb_push_idx].t_buf.timeTick), (void *)&utc, sizeof(utc));
	}
#ifdef ATTACH_DETECTION
	if (temp_state_is_attached()) {
		__cb[__cb_push_idx].t_buf.validFlag |= TYPE_VALID_FLAG;
	}
#endif

    __cb[__cb_push_idx].t_buf.tempValue = temp;
    __cb[__cb_push_idx].status_flag |= CB_WRITE_DONE;

    // Current buffer is done, notify consumer
    if (__cb_pop_idx == -1) {
        __cb_pop_idx = __cb_push_idx;
    }
    __cb_push_idx = (__cb_push_idx + 1) % CB_CIRCULAR_BUFFER_DEPTH;
    buf_stats.push_samples++;

    sd_nvic_critical_region_exit(nested_critical_region);
}

uint8 CBDataAvailable(void)
{
    uint8 ret = 0;
    uint8_t nested_critical_region = 0;

    sd_nvic_critical_region_enter(&nested_critical_region);

    if ((__cb_pop_idx != -1) && (__cb[__cb_pop_idx].status_flag & CB_WRITE_DONE)) {
        ret = 1;
    }

    sd_nvic_critical_region_exit(nested_critical_region);

    return ret;    
}

void CBQueryStatus (uint8 *ptr)
{
    memcpy(ptr, (uint8 *)&buf_stats, sizeof(buf_stats));
}
