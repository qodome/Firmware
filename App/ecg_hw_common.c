/*
 * The common logic for both AD8232 and E9624 ECG
 * ADC sampler logic is implemented here!
 * Timer1 is configured to do sampling
 */

#include "hal_timer.h"
#include "ecg_hw_cfg.h"
#include "ioCC2541.h"
#include "cmdbuffer.h"
#include "ioCC254x_bitdef.h"

static uint16 sampler_freq = 1000;

void ecg_hw_sample_init(void)
{
    // Halt timer1
    T1CTL = (T1CTL & ~(T1CTL_MODE)) | T1CTL_MODE_SUSPEND;
    
    // Setup interrupt
    // clear
    T1STAT = ~T1STAT_CH0IF;
    T1CCTL0 |= T1CCTLn_IM;      // Enable interrupt on channel 0.
    T1CCTL0 = (T1CCTL0 & ~(T1CCTLn_CMP_SET_ON_CMP)) | T1CCTLn_CMP_SET_ON_CMP;
    T1CCTL0 |= T1CCTLn_MODE;    // compare mode
        
    // Configure counting period 1M/sampler_freq;
    T1CC0L = (uint8)((1000000 / sampler_freq) & 0xFF);
    T1CC0H = (uint8)(((1000000 / sampler_freq) >> 8) & 0xFF);
    
    // Enable interrupt
    T1IE = 1;
    EA = 1;
        
    // Timer tick is 1MHz
    T1CTL = (T1CTL & ~(T1CTL_DIV)) | T1CTL_DIV_32;
}

void ecg_hw_sample_set_freq(uint16 freq)
{
    sampler_freq = freq;
    // Configure counting period 1M/sampler_freq;
    T1CC0L = (uint8)((1000000 / sampler_freq) & 0xFF);
    T1CC0H = (uint8)(((1000000 / sampler_freq) >> 8) & 0xFF);    
}


void ecg_hw_sample_begin(void)
{
    T1CTL = (T1CTL & ~(T1CTL_MODE)) | T1CTL_MODE_MODULO;
}

void ecg_hw_sample_end(void)
{
    T1CTL = (T1CTL & ~(T1CTL_MODE)) | T1CTL_MODE_SUSPEND;
}

HAL_ISR_FUNCTION(ecg_sampler, T1_VECTOR)
{
    uint16 temp_sample = 0;
    
    HAL_ENTER_ISR();

    if (T1STAT & 0x01) {
        // Channel 0
        temp_sample = ecg_hw_read_sample();
        CBPushTemp(temp_sample);
    }

    T1STAT = 0x00;
    T1IF = 0;
    
    HAL_EXIT_ISR();
}
