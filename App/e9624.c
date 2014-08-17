/*
 * ECG solution with E9624
 */
#if defined(ECG_E9624)

#include "hal_adc.h"
#include "ecg_hw_cfg.h"

void ecg_hw_init_io(void)
{
    // Enable AIN7 as reference voltage
    P0DIR &= ~(1<<5);
    P0SEL &= ~(1<<5);  
    P0DIR &= ~(1<<7);
    P0SEL &= ~(1<<7); 
    APCFG |= (BV(5) | BV(7));  
    
    // Configure power control pin
    P1SEL &= ~(1<<1);
    P1DIR |= BV(1);
    // Initially power off
    P1_1 = 0;
    
    // Setup ADC reference
    HalAdcSetReference(HAL_ADC_REF_AIN7);
}

void ecg_hw_turn_on(void)
{
    P1_1 = 1;
}

void ecg_hw_turn_off(void)
{
    P1_1 = 0;
}

uint16 ecg_hw_read_sample(void)
{
    return HalAdcRead(HAL_ADC_CHANNEL_5, HAL_ADC_RESOLUTION_12);
}

#endif