/*
 * Command buffer implementation
 * applied on ECG sampling
 */

#include "hal_mcu.h"
#include "hal_types.h"
#include "hal_defs.h"
#include "OSAL.h"
#include "cmdbuffer.h"
#include "huffman_encode.h"

/*
 * CB related static states
 */
static struct cmd_buffer __cb[CB_CIRCULAR_BUFFER_DEPTH];
static int16 __cb_push_idx = 0;       // The next available buffer for PushBuffer
static int16 __cb_pop_idx = -1;   // The next available buffer for Get by USB
struct cmd_buffer_stats buf_stats;
static uint8 __cb_seq = 0;

/*********************** Static internal utility functions **************************/


/***********************          External APIs            **************************/
/*
 * Initialize all EXG Data Buffer staff
 */
void CBInit (void)
{
    osal_memset((void *)__cb, 0, (CB_CIRCULAR_BUFFER_DEPTH * sizeof(struct cmd_buffer)));
    osal_memset((void *)&buf_stats, 0, sizeof(struct cmd_buffer_stats));
    buf_stats.max_delta = 0;
    buf_stats.min_delta = 0x7FFF;
}

/*
 * Invoked by USB TX to get the next FULL buffer
 * for USB transaction
 */
struct cmd_buffer *CBGetNextBufferForTX (void)
{
    struct cmd_buffer *ret = NULL;
    halIntState_t intState;
    
    /* disable interrupt */
    HAL_ENTER_CRITICAL_SECTION(intState);

    if ((__cb_pop_idx != -1) && (__cb[__cb_pop_idx].status_flag & CB_WRITE_DONE)) {
        ret = &(__cb[__cb_pop_idx]);
        if (__cb[__cb_pop_idx].status_flag & (CB_READ_IN_PROGRESS | CB_READ_DONE | CB_WRITE_IN_PROGRESS)) {
            buf_stats.bug_count |= (1 << 0);
        }
        __cb[__cb_pop_idx].status_flag &= ~(CB_WRITE_DONE);
        __cb[__cb_pop_idx].status_flag |= CB_READ_IN_PROGRESS;
    }

    /* enable interrupt */
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

    /* disable interrupt */
    HAL_ENTER_CRITICAL_SECTION(intState);

    buf_stats.pull_samples += (pCb->e_buf.sample_counts + 1);
    pCb->status_flag &= ~(CB_READ_IN_PROGRESS);
    pCb->status_flag |= CB_READ_DONE;
    __cb_pop_idx = (__cb_pop_idx + 1) % CB_CIRCULAR_BUFFER_DEPTH;

    /* enable interrupt */
    HAL_EXIT_CRITICAL_SECTION(intState);
}

// Query the available bits on the buffer
static uint16 CBAvailableBits(struct cmd_buffer *cBuf)
{
    uint16 ret = 0;
    
    if (cBuf == NULL) {
        return 0;
    }
    if (cBuf->B_idx < CB_BYTES_PER_BUFFER && cBuf->b_idx < 8) {
        ret = CB_BITS_PER_BUFFER - cBuf->B_idx * 8 - cBuf->b_idx;
    }
    return ret;
}

/*
 * Invoked by sampling routine to push generated data
 */
void CBPushTemp (int16 temp)
{
    halIntState_t intState;
    static int16 last_temp = 0;
    int16 delta_temp = 0;
    uint8 nhbits = 0;
    uint16 hB = 0;
    uint32 hB32 = 0;
    uint16 idx = 0;
    
    if (__cb[__cb_push_idx].status_flag & CB_READ_IN_PROGRESS) {
        buf_stats.bug_count |= (1 << 2);
        return;
    }
    
    /* disable interrupt */
    HAL_ENTER_CRITICAL_SECTION(intState);

again:
    if (__cb[__cb_push_idx].status_flag & CB_WRITE_IN_PROGRESS) {
        // Calculate delta and do huffman encoding
        delta_temp = temp - last_temp;
        if (delta_temp > buf_stats.max_delta) {
            buf_stats.max_delta = delta_temp;
        }
        if (delta_temp < buf_stats.min_delta) {
            buf_stats.min_delta = delta_temp;
        }
        if (delta_temp > 126) {
            delta_temp = 127;
        }
        if (delta_temp < -127) {
            delta_temp = -128;
        }
        nhbits = huffman_get_code_nbits((uint8)(delta_temp & 0xFF));
        hB = huffman_get_code((uint8)(delta_temp & 0xFF));
        if (nhbits > CBAvailableBits(&__cb[__cb_push_idx])) {
            // This buffer not big enough for this sample,
            // kick this buffer out and get a new one
            if ((__cb_push_idx + 1) % CB_CIRCULAR_BUFFER_DEPTH == __cb_pop_idx && (__cb[__cb_pop_idx].status_flag & CB_READ_IN_PROGRESS)) {
                // Not able to get the new buffer :(
                buf_stats.overflow_counts++;
                HAL_EXIT_CRITICAL_SECTION(intState);
                last_temp = temp;
                return;                
            }
            // Current buffer is done, notify consumer
            if (__cb_pop_idx == -1) {
                __cb_pop_idx = __cb_push_idx;
            }
            __cb[__cb_push_idx].status_flag &= ~(CB_WRITE_IN_PROGRESS);
            __cb[__cb_push_idx].status_flag |= CB_WRITE_DONE;
            __cb_push_idx = (__cb_push_idx + 1) % CB_CIRCULAR_BUFFER_DEPTH;
            if (__cb[__cb_push_idx].status_flag & CB_WRITE_IN_PROGRESS) {
                buf_stats.bug_count |= (1 << 3);
            }
            // goto again, let the other block of code do the job!
            goto again;
        } else {
            if ((nhbits + __cb[__cb_push_idx].b_idx) <= 8) {
                __cb[__cb_push_idx].e_buf.buf[__cb[__cb_push_idx].B_idx] |= (uint8)(hB & 0xFF) << __cb[__cb_push_idx].b_idx;
            } else if ((nhbits + __cb[__cb_push_idx].b_idx) <= 16) {
                hB = hB << __cb[__cb_push_idx].b_idx;
                __cb[__cb_push_idx].e_buf.buf[__cb[__cb_push_idx].B_idx] |= (uint8)(hB & 0xFF);
                __cb[__cb_push_idx].e_buf.buf[__cb[__cb_push_idx].B_idx + 1] = (uint8)((hB >> 8) & 0xFF);
            } else {
                hB32 = (uint32)hB;
                hB32 = hB32 << __cb[__cb_push_idx].b_idx;
                __cb[__cb_push_idx].e_buf.buf[__cb[__cb_push_idx].B_idx] |= (uint8)(hB32 & 0xFF);
                __cb[__cb_push_idx].e_buf.buf[__cb[__cb_push_idx].B_idx + 1] = (uint8)((hB32 >> 8) & 0xFF);
                __cb[__cb_push_idx].e_buf.buf[__cb[__cb_push_idx].B_idx + 2] = (uint8)((hB32 >> 16) & 0xFF);
            }
            __cb[__cb_push_idx].B_idx += (nhbits + __cb[__cb_push_idx].b_idx) / 8;            
            __cb[__cb_push_idx].b_idx = (nhbits + __cb[__cb_push_idx].b_idx) % 8;
            __cb[__cb_push_idx].e_buf.sample_counts++;
        }
    } else {
        if (__cb[__cb_push_idx].status_flag & CB_WRITE_DONE) {
            buf_stats.bug_count |= (1 << 4);
        }
        __cb[__cb_push_idx].status_flag &= ~(CB_READ_DONE);
        __cb[__cb_push_idx].status_flag |= CB_WRITE_IN_PROGRESS;
        __cb[__cb_push_idx].B_idx = 0;
        __cb[__cb_push_idx].b_idx = 0;
        __cb[__cb_push_idx].e_buf.first_sample = temp;
        __cb[__cb_push_idx].e_buf.sample_counts = 0;
        __cb[__cb_push_idx].e_buf.seq = __cb_seq++;
        for (idx = 0; idx < CB_BYTES_PER_BUFFER; idx++) {
            __cb[__cb_push_idx].e_buf.buf[idx] = 0x00;
        }
    }
    buf_stats.push_samples++;

    /* enable interrupt */
    HAL_EXIT_CRITICAL_SECTION(intState);
    
    last_temp = temp;
}

void CBQueryStatus (uint8 *ptr)
{
    osal_memcpy(ptr, (uint8 *)&buf_stats, sizeof(buf_stats));
}

void CBQueryStatus2 (uint8 *ptr)
{
    int idx = 0;
    
    for (idx = 20; idx < 32; idx++) {
        ptr[idx - 20] = __cb[idx].status_flag;
    }    
    ptr[12] = (uint8)(__cb_push_idx & 0xFF);
    ptr[13] = (uint8)(__cb_pop_idx & 0xFF);
}