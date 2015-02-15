/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "app_timer.h"
#include <stdlib.h>
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util.h"
#include "app_util_platform.h"
#include "persistent.h"

#define RTC1_IRQ_PRI            APP_IRQ_PRIORITY_LOW                        /**< Priority of the RTC1 interrupt (used for checking for timeouts and executing timeout handlers). */
#define SWI0_IRQ_PRI            APP_IRQ_PRIORITY_LOW                        /**< Priority of the SWI0 interrupt (used for updating the timer list). */

// The current design assumes that both interrupt handlers run at the same interrupt level.
// If this is to be changed, protection must be added to prevent them from interrupting each other
// (e.g. by using guard/trigger flags).
STATIC_ASSERT(RTC1_IRQ_PRI == SWI0_IRQ_PRI);

#define MAX_RTC_COUNTER_VAL     0x00FFFFFF                                  /**< Maximum value of the RTC counter. */

#define APP_HIGH_USER_ID        0                                           /**< User Id for the Application High "user". */
#define APP_LOW_USER_ID         1                                           /**< User Id for the Application Low "user". */
#define THREAD_MODE_USER_ID     2                                           /**< User Id for the Thread Mode "user". */

#define RTC_COMPARE_OFFSET_MIN  3                                           /**< Minimum offset between the current RTC counter value and the Capture Compare register. Although the nRF51 Series User Specification recommends this value to be 2, we use 3 to be safer.*/

#define MAX_RTC_TASKS_DELAY     47                                          /**< Maximum delay until an RTC task is executed. */

/**@brief Timer allocation state type. */
typedef enum
{
    STATE_FREE,                                                             /**< The timer node is available. */
    STATE_ALLOCATED                                                         /**< The timer node has been allocated. */
} timer_alloc_state_t;

/**@brief Timer node type. The nodes will be used form a linked list of running timers. */
typedef struct
{
    timer_alloc_state_t         state;                                      /**< Timer allocation state. */
    app_timer_mode_t            mode;                                       /**< Timer mode. */
    uint32_t                    cycle_to_expire;
    uint32_t                    ticks_to_expire;                            /**< Number of ticks from previous timer interrupt to timer expiry. */
    uint32_t                    ticks_at_start;                             /**< Current RTC counter value when the timer was started. */
    uint32_t                    ticks_interval;                             /**< Number of ticks in the first timer interval. */
    bool                        is_running;                                 /**< True if timer is running, False otherwise. */
    app_timer_timeout_handler_t p_timeout_handler;                          /**< Pointer to function to be executed when the timer expires. */
    void *                      p_context;                                  /**< General purpose pointer. Will be passed to the timeout handler when the timer expires. */
    app_timer_id_t              next;                                       /**< Id of next timer in list of running timers. */
} timer_node_t;

STATIC_ASSERT(sizeof(timer_node_t) <= APP_TIMER_NODE_SIZE);
STATIC_ASSERT(sizeof(timer_node_t) % 4 == 0);

/**@brief User id type.
 *
 * @details In the current implementation, this will automatically be generated from the current
 *          interrupt level.
 */
typedef uint32_t timer_user_id_t;

#define TIMER_NULL                  ((app_timer_id_t)(0 - 1))                   /**< Invalid timer id. */
#define CONTEXT_QUEUE_SIZE_MAX      (2)                                         /**< Timer internal elapsed ticks queue size. */

uint8_t                       m_node_array_size;                         /**< Size of timer node array. */
timer_node_t *                mp_nodes = NULL;                           /**< Array of timer nodes. */
app_timer_id_t                m_timer_id_head;                           /**< First timer in list of running timers. */
app_timer_evt_schedule_func_t m_evt_schedule_func;                       /**< Pointer to function for propagating timeout events to the scheduler. */
bool                          m_rtc1_running;                            /**< Boolean indicating if RTC1 is running. */
uint32_t                      m_cycle_cnt = 0;
uint32_t                      m_last_tick = 0;

static void app_timer_start_internal(app_timer_id_t timer_id, uint32_t timeout_ticks, void * p_context);

/**@brief Function for initializing the RTC1 counter.
 *
 * @param[in] prescaler   Value of the RTC1 PRESCALER register. Set to 0 for no prescaling.
 */
static void rtc1_init(uint32_t prescaler)
{
    NRF_RTC1->PRESCALER = prescaler;
    NVIC_SetPriority(RTC1_IRQn, RTC1_IRQ_PRI);
}


/**@brief Function for starting the RTC1 timer.
 */
static void rtc1_start(void)
{
    NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE0_Msk;
    NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;

    NVIC_ClearPendingIRQ(RTC1_IRQn);
    NVIC_EnableIRQ(RTC1_IRQn);

    NRF_RTC1->TASKS_START = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);

    m_rtc1_running = true;
}

/**@brief Function for returning the current value of the RTC1 counter.
 *
 * @return     Current value of the RTC1 counter.
 */
static __INLINE uint32_t rtc1_counter_get(void)
{
    return NRF_RTC1->COUNTER;
}


/**@brief Function for computing the difference between two RTC1 counter values.
 *
 * @return     Number of ticks elapsed from ticks_old to ticks_now.
 */
static __INLINE uint32_t ticks_diff_get(uint32_t ticks_now, uint32_t ticks_old)
{
    return ((ticks_now - ticks_old) & MAX_RTC_COUNTER_VAL);
}


/**@brief Function for setting the RTC1 Capture Compare register 0, and enabling the corresponding
 *        event.
 *
 * @param[in] value   New value of Capture Compare register 0.
 */
static __INLINE void rtc1_compare0_set(uint32_t value)
{
    NRF_RTC1->CC[0] = value;
}

/**@brief Function for executing an application timeout handler, either by calling it directly, or
 *        by passing an event to the @ref app_scheduler.
 *
 * @param[in]  p_timer   Pointer to expired timer.
 */
static void timeout_handler_exec(timer_node_t * p_timer)
{
    if (m_evt_schedule_func != NULL)
    {
        uint32_t err_code = m_evt_schedule_func(p_timer->p_timeout_handler, p_timer->p_context);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        p_timer->p_timeout_handler(p_timer->p_context);
    }
}

void rtc1_update_cycle_tick(void)
{
    uint32_t time_now = rtc1_counter_get();
    if (time_now < m_last_tick) {
        m_cycle_cnt++;
    }
    m_last_tick = time_now;
}

uint8_t timer_need_exec(app_timer_id_t tid)
{
    if ((mp_nodes[tid].cycle_to_expire == m_cycle_cnt) && (mp_nodes[tid].ticks_to_expire <= (m_last_tick + 3))) {
        return 1;
    } else if (mp_nodes[tid].cycle_to_expire < m_cycle_cnt) {
        return 1;
    } else {
        return 0;
    }
}

/**@brief Function for handling the RTC1 interrupt.
 *
 * @details Checks for timeouts, and executes timeout handlers for expired timers.
 */
void RTC1_IRQHandler(void)
{
    app_timer_id_t timer_id_idx;

    // Clear all events (also unexpected ones)
    NRF_RTC1->EVENTS_COMPARE[0] = 0;
    NRF_RTC1->EVENTS_COMPARE[1] = 0;
    NRF_RTC1->EVENTS_COMPARE[2] = 0;
    NRF_RTC1->EVENTS_COMPARE[3] = 0;
    NRF_RTC1->EVENTS_TICK       = 0;
    NRF_RTC1->EVENTS_OVRFLW     = 0;

    rtc1_update_cycle_tick();

    // Check for expired timers
    while (m_timer_id_head != TIMER_NULL && (timer_need_exec(m_timer_id_head) == 1)) {
        timer_id_idx = m_timer_id_head;
        m_timer_id_head = mp_nodes[timer_id_idx].next;
        mp_nodes[timer_id_idx].next = TIMER_NULL;
        if (mp_nodes[timer_id_idx].is_running == true) {
        	timeout_handler_exec(&mp_nodes[timer_id_idx]);
        }
        if ((mp_nodes[timer_id_idx].mode == APP_TIMER_MODE_REPEATED) && (mp_nodes[timer_id_idx].is_running == true)) {
            app_timer_start_internal(timer_id_idx, mp_nodes[timer_id_idx].ticks_interval, mp_nodes[timer_id_idx].p_context);
        }
        rtc1_update_cycle_tick();
    }
    if (m_timer_id_head != TIMER_NULL) {
        rtc1_compare0_set(mp_nodes[m_timer_id_head].ticks_to_expire);
    }
}

uint32_t app_timer_init(uint32_t                      prescaler,
                        uint8_t                       max_timers,
                        void *                        p_buffer,
                        app_timer_evt_schedule_func_t evt_schedule_func)
{
    int i;

    // Check that buffer is correctly aligned
    if (!is_word_aligned(p_buffer))
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    // Check for NULL buffer
    if (p_buffer == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    m_evt_schedule_func = evt_schedule_func;

    // Initialize timer node array
    m_node_array_size = max_timers;
    mp_nodes          = p_buffer;
    
    for (i = 0; i < max_timers; i++)
    {
        mp_nodes[i].state      = STATE_FREE;
        mp_nodes[i].is_running = false;
    }
    
    m_timer_id_head             = TIMER_NULL;

    NVIC_ClearPendingIRQ(SWI0_IRQn);
    NVIC_SetPriority(SWI0_IRQn, SWI0_IRQ_PRI);
    NVIC_EnableIRQ(SWI0_IRQn);

    rtc1_init(prescaler);
    rtc1_start();
    
    return NRF_SUCCESS;
}


uint32_t app_timer_create(app_timer_id_t *            p_timer_id,
                          app_timer_mode_t            mode,
                          app_timer_timeout_handler_t timeout_handler)
{
    int i;

    // Check state and parameters
    if (mp_nodes == NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (timeout_handler == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (p_timer_id == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }    
    
    // Find free timer
    for (i = 0; i < m_node_array_size; i++)
    {
        if (mp_nodes[i].state == STATE_FREE)
        {
            mp_nodes[i].state             = STATE_ALLOCATED;
            mp_nodes[i].mode              = mode;
            mp_nodes[i].p_timeout_handler = timeout_handler;
            
            *p_timer_id = i;
            return NRF_SUCCESS;
        }
    }
    
    return NRF_ERROR_NO_MEM;
}

static void app_timer_start_internal(app_timer_id_t timer_id, uint32_t timeout_ticks, void * p_context)
{
    app_timer_id_t timer_id_idx, timer_id_idx_prev;

    // Timer node is accessed by timer_id, no need to protect timer node itself
    rtc1_update_cycle_tick();

    if (mp_nodes[timer_id].is_running == true) {
        timer_id_idx = m_timer_id_head;
        timer_id_idx_prev = TIMER_NULL;

        while (timer_id_idx != TIMER_NULL && timer_id_idx != timer_id) {
            timer_id_idx_prev = timer_id_idx;
            timer_id_idx = mp_nodes[timer_id_idx].next;
        }

        if (timer_id_idx == timer_id) {
        	persistent_record_error(PERSISTENT_ERROR_TIMER1, (uint32_t)timer_id);
        	return;
        }
    }

    mp_nodes[timer_id].ticks_at_start = m_last_tick;
    mp_nodes[timer_id].ticks_interval = timeout_ticks;
    mp_nodes[timer_id].ticks_to_expire = (mp_nodes[timer_id].ticks_at_start + timeout_ticks) & MAX_RTC_COUNTER_VAL;
    mp_nodes[timer_id].cycle_to_expire = m_cycle_cnt;
    if (mp_nodes[timer_id].ticks_to_expire < mp_nodes[timer_id].ticks_at_start) {
    	mp_nodes[timer_id].cycle_to_expire++;
    }
    mp_nodes[timer_id].is_running = true;
    mp_nodes[timer_id].p_context = p_context;
    mp_nodes[timer_id].next = TIMER_NULL;

    timer_id_idx = m_timer_id_head;
    timer_id_idx_prev = TIMER_NULL;

    while (timer_id_idx != TIMER_NULL) {
        if (mp_nodes[timer_id].cycle_to_expire < mp_nodes[timer_id_idx].cycle_to_expire) {
            break;
        } else if ((mp_nodes[timer_id].cycle_to_expire == mp_nodes[timer_id_idx].cycle_to_expire) &&
                    (mp_nodes[timer_id].ticks_to_expire < mp_nodes[timer_id_idx].ticks_to_expire)) {
            break;
        }
        timer_id_idx_prev = timer_id_idx;
        timer_id_idx = mp_nodes[timer_id_idx].next;
    }

    if (timer_id_idx_prev == TIMER_NULL) {
        mp_nodes[timer_id].next = m_timer_id_head;
        m_timer_id_head = timer_id;
        
        // Reschedule RTC1
        rtc1_compare0_set(mp_nodes[timer_id].ticks_to_expire);
    } else {
        mp_nodes[timer_id].next = timer_id_idx;
        mp_nodes[timer_id_idx_prev].next = timer_id;
    }
}

uint32_t app_timer_start(app_timer_id_t timer_id, uint32_t timeout_ticks, void * p_context)
{
    uint8_t nested_critical_region;
    
    // Do not allow execution from IRQ_HIGH
    if (current_int_priority_get() == APP_IRQ_PRIORITY_HIGH) {
    	while (1);
    }

    // Check state and parameters
    if (mp_nodes == NULL) {
        return NRF_ERROR_INVALID_STATE;
    }
    if ((timer_id >= m_node_array_size) || (timeout_ticks < APP_TIMER_MIN_TIMEOUT_TICKS)) {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (mp_nodes[timer_id].state != STATE_ALLOCATED) {
        return NRF_ERROR_INVALID_STATE;
    }

    // Backup&&disable interrput to update global timer list
    sd_nvic_critical_region_enter(&nested_critical_region);

    app_timer_start_internal(timer_id, timeout_ticks, p_context);

    // Restore interrupt
    sd_nvic_critical_region_exit(nested_critical_region);

    return NRF_SUCCESS;
}

uint32_t app_timer_stop(app_timer_id_t timer_id)
{
    uint8_t nested_critical_region;
    app_timer_id_t timer_id_idx, timer_id_idx_prev;

    // Do not allow execution from IRQ_HIGH
    if (current_int_priority_get() == APP_IRQ_PRIORITY_HIGH) {
    	while (1);
    }

    // Check state and parameters
    if (mp_nodes == NULL) {
        return NRF_ERROR_INVALID_STATE;
    }
    if (timer_id >= m_node_array_size) {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (mp_nodes[timer_id].state != STATE_ALLOCATED) {
        return NRF_ERROR_INVALID_STATE;
    }

    // Backup&&disable interrput to update global timer list
    sd_nvic_critical_region_enter(&nested_critical_region);

    mp_nodes[timer_id].is_running = false;
    timer_id_idx = m_timer_id_head;
    timer_id_idx_prev = TIMER_NULL;

    while (timer_id_idx != TIMER_NULL && timer_id_idx != timer_id) {
        timer_id_idx_prev = timer_id_idx;
        timer_id_idx = mp_nodes[timer_id_idx].next;
    }

    if (timer_id_idx == timer_id) {
    	if (timer_id_idx_prev == TIMER_NULL) {
    		m_timer_id_head = mp_nodes[timer_id_idx].next;
    	} else {
    		mp_nodes[timer_id_idx_prev].next = mp_nodes[timer_id_idx].next;
    	}
		mp_nodes[timer_id_idx].next = TIMER_NULL;
    } else {
    	// Timer not started yet, not error!
    	//persistent_record_error(PERSISTENT_ERROR_TIMER2, (uint32_t)timer_id);
    }

    // Restore interrupt
    sd_nvic_critical_region_exit(nested_critical_region);

    return NRF_SUCCESS;
}


uint32_t app_timer_stop_all(void)
{
    int i;
    uint8_t nested_critical_region;

    // Do not allow execution from IRQ_HIGH
    if (current_int_priority_get() == APP_IRQ_PRIORITY_HIGH) {
    	while (1);
    }

    // Check state and parameters
    if (mp_nodes == NULL) {
        return NRF_ERROR_INVALID_STATE;
    }

    // Backup&&disable interrput to update global timer list
    sd_nvic_critical_region_enter(&nested_critical_region);

    m_timer_id_head = TIMER_NULL;
    for (i = 0; i < m_node_array_size; i++) {
        mp_nodes[i].is_running = false;
        mp_nodes[i].next = TIMER_NULL;
    }

    // Restore interrupt
    sd_nvic_critical_region_exit(nested_critical_region);

    return NRF_SUCCESS;
}

uint32_t app_timer_cnt_get(uint32_t * p_ticks)
{
    *p_ticks = rtc1_counter_get();
    return NRF_SUCCESS;
}


uint32_t app_timer_cnt_diff_compute(uint32_t   ticks_to,
                                    uint32_t   ticks_from,
                                    uint32_t * p_ticks_diff)
{
    *p_ticks_diff = ticks_diff_get(ticks_to, ticks_from);
    return NRF_SUCCESS;
}

