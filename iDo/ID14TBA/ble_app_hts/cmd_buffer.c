/*
 * Command buffer implementation
 * applied on ECG sampling
 */

// #include "hal_mcu.h"
// #include "hal_types.h"
// #include "hal_defs.h"
// #include "OSAL.h"
// #include "cmdbuffer.h"
// #include "recorder.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51.h"
#include "ble.h"
#include "app_util.h"
#include "ble_date_time.h"
#include "cmd_buffer.h"
#include "nrf_soc.h"
#include "temp_date_time.h"
#include "UTCtime_convert.h"
#include "temp_state.h"
/*
 * CB related static states
 */
cmd_buffer_t __cb[CB_CIRCULAR_BUFFER_DEPTH];
int16_t __cb_push_idx = 0;     // write ptr
int16_t __cb_pop_idx = -1;     // read ptr
cmd_buffer_stats_t buf_stats;

/*********************** Static internal utility functions **************************/


/***********************          External APIs            **************************/
/*
 * Initialize all EXG Data Buffer staff
 */
void cmd_buffer_init (void)
{
    memset((void *)__cb, 0, (CB_CIRCULAR_BUFFER_DEPTH * sizeof(cmd_buffer_t)));
    memset((void *)&buf_stats, 0, sizeof(cmd_buffer_stats_t));
    __cb_push_idx = 0;
    __cb_pop_idx = -1;
}

// The cleanup routine is invoked by iDo when connected to new device
void cmd_buffer_clean_up(void)
{
	// halIntState_t intState;
	uint8_t nested_critical_region = 0;
	// HAL_ENTER_CRITICAL_SECTION(intState);
	sd_nvic_critical_region_enter(&nested_critical_region);

	memset((void *)__cb, 0, (CB_CIRCULAR_BUFFER_DEPTH * sizeof(cmd_buffer_t)));
	__cb_push_idx = 0;
	__cb_pop_idx = -1;

	sd_nvic_critical_region_exit(nested_critical_region);
	//  HAL_EXIT_CRITICAL_SECTION(intState);
}

/*
 * Get the next Temperature Measurement
 */
cmd_buffer_t *CBGetNextBufferForTX (int16_t *tempPtr, uint8_t *flag, ble_date_time_t *tempTSPtr)
{
    cmd_buffer_t *ret = NULL;
  //  halIntState_t intState;
    uint8_t nested_critical_region = 0;
    
  //  HAL_ENTER_CRITICAL_SECTION(intState);
    sd_nvic_critical_region_enter(&nested_critical_region);

    if ((__cb_pop_idx != -1) && (__cb[__cb_pop_idx].status_flag & CB_WRITE_DONE)) {
        ret = &(__cb[__cb_pop_idx]);
        *tempPtr = __cb[__cb_pop_idx].t_buf.tempValue;
        *flag = __cb[__cb_pop_idx].t_buf.validFlag;
        memcpy(tempTSPtr, &(__cb[__cb_pop_idx].t_buf.timeTick), sizeof(ble_date_time_t));
    } else {
        buf_stats.underflow_counts++;
    }

   // HAL_EXIT_CRITICAL_SECTION(intState);
    sd_nvic_critical_region_exit(nested_critical_region);

    return ret;
}

/*
 * Manage ref_cnt
 */
void cmd_put_buffer(cmd_buffer_t *pCb)
{
  //  halIntState_t intState;
	uint8_t nested_critical_region = 0;

    if (__cb_pop_idx == -1) {
    	buf_stats.bug_count |= (1 << 1);
        return;
    }

  //  HAL_ENTER_CRITICAL_SECTION(intState);
    sd_nvic_critical_region_enter(&nested_critical_region);

    buf_stats.pull_samples++;
    pCb->status_flag &= ~(CB_WRITE_DONE);
    __cb_pop_idx = (__cb_pop_idx + 1) % CB_CIRCULAR_BUFFER_DEPTH;

  //  HAL_EXIT_CRITICAL_SECTION(intState);
    sd_nvic_critical_region_exit(nested_critical_region);
}

/*
 * Invoked by sampling routine to push measured temperature
 */
void cmd_buffer_push_temp (int16_t temp)
{
	//  halIntState_t intState;
	uint8_t nested_critical_region = 0;

	// HAL_ENTER_CRITICAL_SECTION(intState);
	sd_nvic_critical_region_enter(&nested_critical_region);

	if (__cb[__cb_push_idx].status_flag & CB_WRITE_DONE) {
		buf_stats.overflow_counts++;
	}
	osal_ConvertUTCTime( &(__cb[__cb_push_idx].t_buf.timeTick), date_time_get());


	__cb[__cb_push_idx].t_buf.validFlag = 0;
	if (flag_time_stamp_get()) {
		__cb[__cb_push_idx].t_buf.validFlag |= TIME_VALID_FLAG;
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

	//  HAL_EXIT_CRITICAL_SECTION(intState);
	sd_nvic_critical_region_exit(nested_critical_region);
}

uint8_t cmd_buffer_data_avalid(void)
{
    uint8_t ret = 0;
   // halIntState_t intState;
	uint8_t nested_critical_region = 0;
   // HAL_ENTER_CRITICAL_SECTION(intState);
    sd_nvic_critical_region_enter(&nested_critical_region);

    if ((__cb_pop_idx != -1) && (__cb[__cb_pop_idx].status_flag & CB_WRITE_DONE)) {
        ret = 1;
    }

   // HAL_EXIT_CRITICAL_SECTION(intState);
    sd_nvic_critical_region_exit(nested_critical_region);
    return ret;    
}

void cmd_buffer_query_status (uint8_t *ptr)
{
    memcpy(ptr, (uint8_t *)&buf_stats, sizeof(buf_stats));
}
