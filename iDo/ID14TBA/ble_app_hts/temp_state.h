/*
 * Calculates the measured temperature, detects state
 * transition
 */
#ifndef __TEMP_STATE__
#define __TEMP_STATE__



struct temp_hist {
    int16_t temp;
    uint16_t idx;
    struct temp_hist *prev;
    struct temp_hist *next;
};

void temp_state_init(void);
void temp_state_update(int16_t newTemp);
uint8_t temp_state_is_attached(void);
int16_t temp_state_get_last_filtered_temp(void);
uint8_t temp_state_mesaurement_ready(void);

#endif
