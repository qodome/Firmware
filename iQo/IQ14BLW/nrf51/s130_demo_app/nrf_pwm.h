#ifndef __NRF_PWM_H__
#define __NRF_PWM_H__

#include <stdint.h>

// This define sets the number of PWM channels to use, valid options are 1, 2, 3 and 4
#define PWM_NUM_CHANNELS        3

// To change the timer used for the PWM library replace the three defines below
#define PWM_TIMER               NRF_TIMER1
#define PWM_IRQHandler          TIMER1_IRQHandler
#define PWM_IRQn                TIMER1_IRQn

#define PWM_TIMER2				NRF_TIMER2
#define PWM_IRQ2Handler         TIMER2_IRQHandler
#define PWM_IRQ2n               TIMER2_IRQn

// To change the GPIOTE channels used by the different PWM channels, please change the defines below
#define PWM_GPIOTE_CHANNEL0     2
#define PWM_GPIOTE_CHANNEL1     3
#define PWM_GPIOTE_CHANNEL2     0
#define PWM_GPIOTE_CHANNEL3		1

typedef enum
{
    PWM_MODE_LED_100,   // 0-100 resolution, 156 Hz PWM frequency, 32 kHz timer frequency (prescaler 9)
    PWM_MODE_LED_255,   // 8-bit resolution, 122 Hz PWM frequency, 65 kHz timer frequency (prescaler 8)
    PWM_MODE_LED_1000,  // 0-1000 resolution, 125 Hz PWM frequency, 500 kHz timer frequency (prescaler 5)
    
    PWM_MODE_MTR_100,   // 0-100 resolution, 20 kHz PWM frequency, 4MHz timer frequency (prescaler 2)
    PWM_MODE_MTR_255    // 8-bit resolution, 31 kHz PWM frequency, 16MHz timer frequency (prescaler 0)
}nrf_pwm_mode_t;


#if (PWM_NUM_CHANNELS == 1)
void nrf_pwm_init(uint32_t io_select_pwm0, nrf_pwm_mode_t pwm_mode);
#elif (PWM_NUM_CHANNELS == 2)
void nrf_pwm_init(uint32_t io_select_pwm0, uint32_t io_select_pwm1, nrf_pwm_mode_t pwm_mode);
#elif (PWM_NUM_CHANNELS == 3)
void nrf_pwm_init(uint32_t io_select_pwm0, uint32_t io_select_pwm1, uint32_t io_select_pwm2, nrf_pwm_mode_t pwm_mode);
#elif (PWM_NUM_CHANNELS == 4)
void nrf_pwm_init(uint32_t io_select_pwm0, uint32_t io_select_pwm1, uint32_t io_select_pwm2, uint32_t io_select_pwm3, nrf_pwm_mode_t pwm_mode);
#else
#error Invalid number of PWM channels selected! Please choose a number between 1 and 3
#endif

void nrf_pwm_set_value(uint32_t pwm_channel, uint32_t pwm_value);

#endif
