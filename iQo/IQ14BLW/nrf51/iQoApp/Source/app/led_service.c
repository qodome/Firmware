/*
 * LED service
 */

#include "led_service.h"
#include "nrf_pwm.h"
#include "app_timer.h"
#include "app_error.h"

#define LED_STEP_INTERVAL					APP_TIMER_TICKS(20, 0)
#define LED_STEP_SIZE						2
static app_timer_id_t led_dim_timer_id[4];
static uint8_t led_current[4] = {0};
static uint8_t led_target[4] = {0};
static uint8_t led_turning[4] = {0};
static uint8_t led_step_size[4] = {0};
static uint8_t led_step_cnt[4] = {0};
static uint8_t led_recalculate[4] = {0};
static uint8_t led_tag[4] = {0, 1, 2, 3};

void led_set_light(uint8_t idx, uint8_t target)
{
	uint8_t delta;

	// Four LEDs
	if ((idx >= 4) || (led_turning[idx] != 0) || (target == led_current[idx])) {
		return;
	}

	led_target[idx] = target;
	if (target > led_current[idx]) {
		led_turning[idx] = 1;
		delta = target - led_current[idx];
	} else {
		led_turning[idx] = 0xFF;
		delta = led_current[idx] - target;
	}

#ifdef LED_STEP_SIZE
	if (delta <= LED_STEP_SIZE) {
		led_step_size[idx] = delta;
		led_step_cnt[idx] = 1;
	} else {
		led_step_size[idx] = LED_STEP_SIZE;
		led_step_cnt[idx] = delta / LED_STEP_SIZE;
		if (delta % LED_STEP_SIZE != 0) {
			led_step_cnt[idx]++;
		}
	}
	led_recalculate[idx] = 0;
#else
	if (delta <= 20) {
		led_step_size[idx] = 1;
		led_step_cnt[idx] = delta;
		led_recalculate[idx] = 0;
	} else {
		led_step_size[idx] = delta / 20;
		led_step_cnt[idx] = 20;
		led_recalculate[idx] = 1;
	}
#endif

	if (led_turning[idx] == 1) {
		led_current[idx] += led_step_size[idx];
		led_step_cnt[idx]--;
		if (led_recalculate[idx] != 0) {
			led_step_size[idx] = (led_target[idx] - led_current[idx]) / led_step_cnt[idx];
		}
	} else {
		led_current[idx] -= led_step_size[idx];
		led_step_cnt[idx]--;
		if (led_recalculate[idx] != 0) {
			led_step_size[idx] = (led_current[idx] - led_target[idx]) / led_step_cnt[idx];
		}
	}
	nrf_pwm_set_value(idx, led_current[idx]);

	if (led_step_cnt[idx] > 0) {
		app_timer_start(led_dim_timer_id[idx], LED_STEP_INTERVAL, &led_tag[idx]);
	} else {
		led_turning[idx] = 0;
	}
}

static void led_dim_timeout_handler(void * p_context)
{
	uint8_t idx;

	idx = *(uint8_t *)p_context;

	if (led_step_cnt[idx] > 1) {
		if (led_turning[idx] == 1) {
			led_current[idx] += led_step_size[idx];
			led_step_cnt[idx]--;
			if (led_recalculate[idx] != 0) {
				led_step_size[idx] = (led_target[idx] - led_current[idx]) / led_step_cnt[idx];
			}
		} else {
			led_current[idx] -= led_step_size[idx];
			led_step_cnt[idx]--;
			if (led_recalculate[idx] != 0) {
				led_step_size[idx] = (led_current[idx] - led_target[idx]) / led_step_cnt[idx];
			}
		}
		nrf_pwm_set_value(idx, led_current[idx]);
		app_timer_start(led_dim_timer_id[idx], LED_STEP_INTERVAL, &led_tag[idx]);
	} else {
		led_current[idx] = led_target[idx];
		led_step_cnt[idx] = 0;
		led_turning[idx] = 0;
		nrf_pwm_set_value(idx, led_target[idx]);
	}
}

void led_service_init(void)
{
    uint8_t idx;

    for (idx = 0; idx < 4; idx++) {
    	APP_ERROR_CHECK(app_timer_create(&(led_dim_timer_id[idx]), APP_TIMER_MODE_SINGLE_SHOT, led_dim_timeout_handler));
    }

    nrf_pwm_init(8, 9, 10, 12, PWM_MODE_LED_255);
    nrf_pwm_set_value(0, 0);
    nrf_pwm_set_value(1, 0);
    nrf_pwm_set_value(2, 0);
    nrf_pwm_set_value(3, 0);
}
