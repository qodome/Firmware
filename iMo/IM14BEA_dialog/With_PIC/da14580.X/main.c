/*
 * File:   main.c
 * Author: Administrator
 *
 * Created on 2014?9?1?, ??2:01
 */
#include <xc.h>
#include <pic.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#undef DEBUG_SPI

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF    // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = ON       // Low Power Brown-Out Reset Enable Bit (Low power brown-out is enabled)
#pragma config LVP = ON        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

uint16_t adc_buffer[2][500] = {{0}, {0}};
uint8_t adc_buf_wptr = 0;
uint8_t adc_buf_rptr = 0;
uint16_t adc_buf_widx = 0;
uint16_t adc_buf_ridx = 0;
uint8_t trigger_spi_send = 0;
uint8_t *adc_buf_read_ptr = NULL;

// Switch to fast clock allow faster SPI transfer
void switch_to_fast(void)
{
    OSCCON = 0b01111010;

    CCPR1H = 0x20;
    CCPR1L = 0x00;
}

// Switch to slow clock allow power save
void switch_to_slow(void)
{
    OSCCON = 0b01010010;

    CCPR1H = 0x01;
    CCPR1L = 0x00;

    TMR1H = 0x00;
    TMR1L = 0x00;
}

// Handle SPI/ADC interrupt
interrupt void isr(void)
{
    uint8_t dummy;

    // SPI interrupt
    if (SSP1IF & SSP1IE) {
        SSP1IF = 0;
        dummy = SSPBUF;
        
        if (adc_buf_ridx < 1000) {
            SSPBUF = adc_buf_read_ptr[adc_buf_ridx++];
        } else {
            switch_to_slow();
            RC4 = 0;
            RA0 = 0;
        }
    }
}

/*
 * 1. Configure clock, default IO etc.
 * 2. Initialize IO port connected with AD8232
 * 3. Configure INT/SPI interface for bulk transfer with BLE
 * 3. Configure ADC for sampling
 * 4. Start sampling timer
 * 5. while (1) waiting for SPI bulk request
 */
int main(int argc, char** argv)
{
    uint8_t dummy;

    OSCCON = 0b01010010;     // PLL disabled, IRCF 31KHz, Sys clock: FOSC setting
    //OSCCON = 0b01111010;

    //while(!MFIOFR);         // Wait until LF clock ready
    while (!HFIOFR);

    /*
    // For those unused pins, set them to output digital
    // RA0-4, 6-7; RB2-5; RC0, 7
    ANSELA &= ~0x9F;
    TRISA &= ~0x9F;
    PORTA &= ~0x9F;

    ANSELB &= ~0x3C;
    TRISB &= ~0x3C;
    PORTB &= ~0x3C;

    ANSELC &= ~0x81;
    TRISC &= ~0x81;
    PORTC &= ~0x81;
     */

    // Configure SPI pin function location

    ANSA5 = 0;              // MCU_SS   - RA5 digital
    ANSB6 = 0;              // MCU_MOSI - RB6 digital
    ANSC5 = 0;              // MCU_MISO - RC5 digital
    
    SDOSEL = 0;             // SDO RC5
    SDISEL = 1;             // SDI RB6
    SCKSEL = 1;             // SCK RB7
    APFCON2bits.SSSEL = 0;  // SS RA5

    // SPI pin direction
    TRISA5 = 1;
    TRISB6 = 1;
    TRISB7 = 1;
    TRISC5 = 0;

    // INT GPIO direction
    // BULK_RDY configured on RC4
    ANSC4 = 0;
    TRISC4 = 0;             // RC4 output
    RC4 = 0;
    // RA0 for debug
    ANSA0 = 0;
    TRISA0 = 0;
    RA0 = 0;

    // User clear error indication bits for SPI
    WCOL = 0;
    SSPOV = 0;

    // SPI settings
    CKP = 0;
    SSPCON1bits.SSPM = 0x04;  // SSPM
    SMP = 0;
    CKE = 0;
    BOEN = 0;
    // Enable SPI
    SSPEN = 1;

    // Enable SPI interrupt
    SSP1IF = 0;
    GIE = 1;
    SSP1IE = 1;
    PEIE = 1;

    // ADC
    // ADC port configuration
    ANSB0 = 1;
    ANSB1 = 1;
    TRISB0 = 1;
    TRISB1 = 1;

    // ADC configuration
    ADFM = 1;
    ADRMD = 0;
    ADCON0bits.CHS = 0b01100;
    ADCON2bits.CHSN = 0b1010;
    ADCON1bits.ADNREF = 0;
    ADCON1bits.ADPREF = 0b11;
    // Configure FVR to use 2.048V
    FVRCONbits.ADFVR = 0b10;
    FVREN = 1;
    ADCON1bits.ADCS = 0b111;
    ADCON2bits.TRIGSEL = 0b0001;
    ADON = 1;
    // Enable interrupt
    ADIE = 0;
    ADIF = 0;

    // Configure CCP
    CCPR1H = 0x01;
    CCPR1L = 0x00;
    //CCPR1H = 0x20;
    //CCPR1L = 00;
    CCP1SEL = 0;
    TRISC2 = 0;             // RC0 output
    CCP1IE = 0;                 // disable CCP interrupt
    CCP1CONbits.CCP1M = 0b1011;

    //CCP1IE = 1;                 // debug enable CCP interrupt
    //CCP1CONbits.CCP1M = 0b1010;

    // Configure Timer1 in timer mode
    T1CONbits.TMR1CS = 0b00;
    T1CONbits.nT1SYNC = 0;
    T1CONbits.T1CKPS = 0b00;
    T1OSCEN = 0;
    TMR1H = 0x00;
    TMR1L = 0x00;
    TMR1ON = 1;
    TMR1GE = 0;

    while (1) {
        // Handle ADC event
        while (ADIF == 0);
        CCP1IF = 0;
        ADIF = 0;
        adc_buffer[adc_buf_wptr][adc_buf_widx++] = ((uint16_t)ADRESL << 8) | ADRESH;
        if (adc_buf_widx >= 500) {
            adc_buf_rptr = adc_buf_wptr;
            adc_buf_ridx = 0;
            adc_buf_read_ptr = (uint8_t *)&(adc_buffer[adc_buf_rptr][0]);

            dummy = SSPBUF;
            SSPBUF = adc_buf_read_ptr[adc_buf_ridx++];
            switch_to_fast();

            // Interrupt BLE
            RC4 = 1;
            RA0 = 1;

            adc_buf_widx = 0;
            adc_buf_wptr = (adc_buf_wptr + 1) % 2;
        }
        if (adc_buf_widx == 250) {
            RC4 = 0;
            RA0 = 0;
        }
    }
    return (EXIT_SUCCESS);
}
