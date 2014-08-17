/*
 * ECG solution with AD8232
 */
#if defined(ECG_AD8232)

#include "hal_adc.h"
#include "ecg_hw_cfg.h"

void ecg_hw_init_io(void)
{
    // LDO enable
    P1SEL &= ~(1<<1);
    P1DIR |= BV(1);
    P1_1 = 0;
    
    // 8232_INT1
    P1SEL &= ~(1<<0);
    P1DIR &= ~(1<<0);
    
    // 8232 enable
    P0SEL &= ~(1<<0);
    P0DIR |= BV(0);
    P0_0 = 0; 
    
    // 8232_LODP
    P0SEL &= ~(1<<1);
    P0DIR &= ~(1<<1); 
    
    // 8232_LODN
    P0SEL &= ~(1<<2);
    P0DIR &= ~(1<<2);
    
    // 8232_INT2
    P0SEL &= ~(1<<6);
    P0DIR &= ~(1<<6);  
    
    // Enable AIN7 as reference voltage
    P0DIR &= ~(1<<4);
    P0SEL &= ~(1<<4);  
    P0DIR &= ~(1<<5);
    P0SEL &= ~(1<<5);
    P0DIR &= ~(1<<7);
    P0SEL &= ~(1<<7); 
    APCFG |= (BV(4) | BV(5) | BV(7)); 
    
    // Setup ADC reference
    HalAdcSetReference(HAL_ADC_REF_AIN7);
}

void ecg_hw_turn_on(void)
{
    // LDO enable
    P1_1 = 1;
    
    // AD8232 enable
    P0_0 = 1; 
}

void ecg_hw_turn_off(void)
{
    P1_1 = 0;
    P0_0 = 0;
}

uint16 ecg_hw_read_sample(void)
{
    return HalAdcRead(/*HAL_ADC_CHN_A4A5*/HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_12);    
}

#endif