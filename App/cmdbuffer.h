/*
 * cmd buffer (for ECG data) used by ECG sampling routine
 */
#ifndef __CMD_BUFFER__
#define __CMD_BUFFER__

#define CB_CIRCULAR_BUFFER_DEPTH 32
#define CB_BYTES_PER_BUFFER 16
#define CB_BITS_PER_BUFFER  128         // @@ UPDATE this value together with CB_BYTES_PER_BUFFER
#define CB_SUCCESS 0
#define CB_FAILURE 1
// Reference related flags
#define CB_STATUS_NEW       0x01
#define CB_STATUS_USB       0x02
#define CB_STATUS_BIP       0x80    // Build In Progress 

#define CB_WRITE_IN_PROGRESS    0x01
#define CB_WRITE_DONE           0x02
#define CB_READ_IN_PROGRESS     0x04
#define CB_READ_DONE            0x08

//#define likely(x) __builtin_expect(!!(x), 1)
//#define unlikely(x) __builtin_expect(!!(x), 0)

struct ecg_buffer {
    int16 first_sample;                 // The first ECG sample within this buffer
    uint8 sample_counts;                // 50 max, not encluding the first sample
    uint8 seq;
    uint8 buf[CB_BYTES_PER_BUFFER];
};
    
struct cmd_buffer {
    uint8 status_flag;
	uint8 B_idx;                        // Index of the byte within CB_BYTES_PER_BUFFER
    uint8 b_idx;                        // Index of the bit within the Byte;
    struct ecg_buffer e_buf;
};

struct cmd_buffer_stats {
    uint16 push_samples;
    uint16 pull_samples;
    uint16 overflow_counts;
    uint16 underflow_counts;
    uint16 bug_count;
    int16 max_delta;
    int16 min_delta;
    uint16 read_skip_count;
    uint8 read_ptr;
    uint8 write_ptr;
};

/*
 * Initialize Command Buffer
 */
void CBInit (void);

/*
 * Invoked by USB TX to get the next cmd buffer
 * for USB transaction
 */
struct cmd_buffer *CBGetNextBufferForTX (void);

/*
 * Manage ref_cnt
 */
void CBPutBuffer (struct cmd_buffer *pCb);

/*
 * Invoked by sampling routine to push sampled data
 */
void CBPushTemp (int16 temp);

void CBQueryStatus (uint8 *ptr);
void CBQueryStatus2 (uint8 *ptr);

#endif
