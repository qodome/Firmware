/*
 * Calculates the measured temperature, detects state
 * transition
 */
#ifndef __TEMP_STATE__
#define __TEMP_STATE__

struct temp_hist {
    int16 temp;
    uint16 idx;
    struct temp_hist *prev;
    struct temp_hist *next;
};

void temp_state_init(void);
void temp_state_update(int16 newTemp);
#ifdef ATTACH_DETECTION
uint8 temp_state_is_attached(void);
#endif
int16 temp_state_get_last_filtered_temp(void);
uint8 temp_state_mesaurement_ready(void);

#endif