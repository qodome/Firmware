/*
 * Calculates the measured temperature, detects state
 * transition:
 * 1. On/off body detection
 * 2. Detects when temeprature stables after on/off body
 * 3. Keep a copy of latest iir_filtered temperature for
 *    temperature measurement inquiry
 */

#include <stdint.h>
#include <string.h>
#include "temp_state.h"


#define TEMP_HIST_DEPTH     25
#define TEMP_HEURISTIC_CNT  20

struct temp_hist od_max;
struct temp_hist od_min;
struct temp_hist od_dlist[TEMP_HIST_DEPTH];
struct temp_hist *od_ptr[TEMP_HIST_DEPTH] = {NULL};
uint8_t od_ptr_idx = 0;
uint8_t tempStateAttachedStatus = 0;     // 0 for not connected, 1 for connected
uint16_t od_idx = 0;
uint8_t od_up_temp = 0;
int16_t od_last_temp = 0;
uint8_t od_events[TEMP_HEURISTIC_CNT] = {0};
uint8_t od_events_total = 0;
uint8_t od_events_idx = 0;
int16_t tempStateLastFilteredTemp = 0;
uint8_t tempStateTempValid = 0;
// tempStateTransit - 0: no temperature transition
//                  - 1: temperature transition UP
//                  - 2: temperature transition DOWN
uint8_t tempStateTransit = 0;
int16_t transitM = 0;
uint16_t transitMCount = 0;

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
static void __temp_state_init_transit_state(void)
{
    transitMCount = 0;
    if (tempStateTransit == 1) {
        transitM = 0x8000;
    } else if (tempStateTransit == 2) {
        transitM = 0x7FFF;
    }
}

static void __temp_state_transit_state_update(int16_t filteredTemp)
{
    if (tempStateTransit == 1) {
        if (filteredTemp > transitM) {
            transitM = filteredTemp;
            transitMCount = 0;
        } else {
            transitMCount++;
        }
    } else if (tempStateTransit == 2) {
        if (filteredTemp < transitM) {
            transitM = filteredTemp;
            transitMCount = 0;
        } else {
            transitMCount++;
        }
    }
    
    if (transitMCount > 10) {
        tempStateTransit = 0;
        // Invoke iDo callback to notify temperature is ready
        // iDo_indicate_measurement_ready();
    }
}

static struct temp_hist *__OD_remove_node(struct temp_hist *t_node)
{
    if (t_node == NULL) {
        return NULL;
    }
    if (t_node->next != NULL && t_node->prev != NULL) {
        t_node->prev->next = t_node->next;
        t_node->next->prev = t_node->prev;
    }
    t_node->next = NULL;
    t_node->prev = NULL;
    return t_node;
}

static void __OD_insert_node(struct temp_hist *t_node)
{
    struct temp_hist *ptr = od_max.next;

    while (ptr != NULL) {
        if (t_node->temp >= ptr->temp) {
            break;
        }
        if (ptr == &od_min) {
            break;
        } else {
            ptr = ptr->next;
        }
    }
    if (ptr == &od_min) {
        // insert t_node before od_min
        t_node->prev = od_min.prev;
        t_node->next = &od_min;
        od_min.prev->next = t_node;
        od_min.prev = t_node;
    } else {
        // insert t_node before ptr
        t_node->prev = ptr->prev;
        t_node->next = ptr;
        ptr->prev->next = t_node;
        ptr->prev = t_node;
    }
}

static void __temp_state_update_attach_state(int16_t temp)
{
    struct temp_hist *t_ptr = NULL;

    // First: insert the node into temperature count
    // down list
    t_ptr = __OD_remove_node(od_ptr[od_ptr_idx]);
    if (t_ptr == NULL) {
        return;
    }
    t_ptr->temp = temp;
    t_ptr->idx = od_idx++;
    __OD_insert_node(t_ptr);

    // Second: check trend compared with last
    if (temp > od_last_temp) {
        od_up_temp += 1;
    } else {
        if (od_up_temp % 2 == 0) {
            od_up_temp /= 2;
        } else {
            od_up_temp = (od_up_temp + 1) / 2;
        }
    }

    od_events_total -= od_events[od_events_idx];
    if (od_up_temp > 5) {
        od_events[od_events_idx] = 1;
    } else {
        od_events[od_events_idx] = 0;
    }
    od_events_total += od_events[od_events_idx];
    od_events_idx = (od_events_idx + 1) % TEMP_HEURISTIC_CNT;

    if (od_events_total >= 5) {
        if (tempStateAttachedStatus == 0) {
            tempStateTransit = 1;
            __temp_state_init_transit_state();
        }
        tempStateAttachedStatus = 1;
    }

    // Third: check off
    if ((od_max.next->temp - od_min.prev->temp) > 16 &&
            ((od_max.next->idx < od_min.prev->idx) || ((od_max.next->idx - od_min.prev->idx) > 32768))) {
        if (tempStateAttachedStatus == 1) {
            tempStateTransit = 2;
            __temp_state_init_transit_state();
        }
        tempStateAttachedStatus = 0;
    }

    // Last: update last
    od_last_temp = temp;
    od_ptr_idx = (od_ptr_idx + 1) % TEMP_HIST_DEPTH;
}

static int16_t __temp_state_iir_filter(int16_t newTemp)
{
    static uint8_t firstSample = 1;
    static float x_n = 0.0;
    static float x_n_1 = 0.0;
    static float x_n_2 = 0.0;
    static float x_n_3 = 0.0;
    static float y_n = 0.0;
    static float y_n_1 = 0.0;
    static float y_n_2 = 0.0;
    static float y_n_3 = 0.0;
    float ret_y = 0.0;

    const float a_2 = -2.3377, a_3 = 1.8787, a_4 = -0.5091;
    const float b_1 = 0.0558, b_2 = -0.0399, b_3 = -0.0399, b_4 = 0.0558;

    x_n = (float)newTemp;
    // FIXME
    if (firstSample == 1) {
        firstSample = 0;
        x_n_1 = x_n;
        x_n_2 = x_n;
        x_n_3 = x_n;
        y_n_1 = x_n;
        y_n_2 = x_n;
        y_n_3 = x_n;
    }
    y_n = b_1 * x_n + b_2 * x_n_1 + b_3 * x_n_2 + b_4 * x_n_3 -
            (a_2 * y_n_1 + a_3 * y_n_2 + a_4 * y_n_3);
    ret_y = y_n;

    x_n_3 = x_n_2;
    x_n_2 = x_n_1;
    x_n_1 = x_n;
    y_n_3 = y_n_2;
    y_n_2 = y_n_1;
    y_n_1 = y_n;

    return (int16_t)ret_y;
}

// Initialize tempState internal states
void temp_state_init(void)
{
    uint8_t idx = 0;

    for (idx = 0; idx < TEMP_HIST_DEPTH; idx++) {
        od_dlist[idx].temp = 0;
        od_dlist[idx].idx = 0;
        od_dlist[idx].prev = NULL;
        od_dlist[idx].next = NULL;
        od_ptr[idx] = &(od_dlist[idx]);
    }
    od_max.temp = 0;
    od_max.idx = 0;
    od_max.prev = NULL;
    od_max.next = &(od_dlist[0]);

    od_dlist[0].prev = &od_max;
    od_dlist[0].next = &(od_dlist[1]);

    od_dlist[1].prev = &(od_dlist[0]);
    od_dlist[1].next = &(od_dlist[2]);

    od_dlist[2].prev = &(od_dlist[1]);
    od_dlist[2].next = &od_min;

    od_min.temp = 0;
    od_min.idx = 0;
    od_min.prev = &(od_dlist[2]);
    od_min.next = NULL;

    od_ptr_idx = 0;
    od_idx = 0;
    tempStateAttachedStatus = 0;
    tempStateLastFilteredTemp = 0;
    tempStateTransit = 0;

    for (idx = 0; idx < TEMP_HEURISTIC_CNT; idx++) {
        od_events[idx] = 0;
    }
    
    tempStateTempValid = 0;
}

// New temperature is logged here, calculated
// and update the state information
void temp_state_update(int16_t newTemp)
{
    __temp_state_update_attach_state(newTemp);
    tempStateLastFilteredTemp = __temp_state_iir_filter(newTemp);
    tempStateTempValid = 1;
    if (tempStateTransit != 0) {
        __temp_state_transit_state_update(tempStateLastFilteredTemp);
    }
}

uint8_t temp_state_is_attached(void)
{
    return tempStateAttachedStatus;
}

int16_t temp_state_get_last_filtered_temp(void)
{
    return tempStateLastFilteredTemp;
}

uint8_t temp_state_mesaurement_ready(void)
{
    if (tempStateTransit != 0 || tempStateTempValid == 0) {
        return 0;
    } else {
        return 1;
    }
}

