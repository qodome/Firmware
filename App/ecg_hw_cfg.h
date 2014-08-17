/*
 * Header file for ECG
 */

#ifndef __ECG_HW_CFG__
#define __ECG_HW_CFG__

void ecg_hw_init_io(void);
void ecg_hw_turn_on(void);
void ecg_hw_turn_off(void);
uint16 ecg_hw_read_sample(void);

void ecg_hw_sample_init(void);
void ecg_hw_sample_begin(void);
void ecg_hw_sample_end(void);

#endif
