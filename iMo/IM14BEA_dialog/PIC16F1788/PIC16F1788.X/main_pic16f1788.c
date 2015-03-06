/*
 * File:   main_test.c
 * Author: Administrator
 *
 * Created on January 30, 2015, 5:19 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
// PIC16F1788 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>
#include <pic.h>
#include "spi_model.h"

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = ON       // Low Power Brown-Out Reset Enable Bit (Low power brown-out is enabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)
/*
 *
 */
#define    delay_us(x) { unsigned int us;  us = (x)/(3)|1; while(--us != 0) continue; }                        // i used 3 (similarly i used in pic16f690-it worked for 8Mhz) i changed

uint16_t adc_buffer[2][500] = {{0}, {0}};
uint8_t adc_buf_wptr = 0;
uint8_t adc_buf_rptr = 0;
uint16_t adc_buf_widx = 0;
uint16_t adc_buf_ridx = 0;
uint8_t trigger_spi_send = 0;
uint8_t *adc_buf_read_ptr = NULL;

// Switch to slow clock allow power save
void start_oscillator_low(void)
{
    OSCCONbits.SPLLEN = 0; //PLL disabled
    OSCCONbits.IRCF = 0b1010; // IRCF 500KHz  default after reset
    OSCCONbits.SCS = 0b10;    //system clock select
    while(OSCSTATbits.HFIOFR == 0);
    CCPR1H = 0x01;
    CCPR1L = 0x00;
    TMR1H = 0x00;
    TMR1L = 0x00;
}

// Switch to fast clock allow faster SPI transfer
void start_oscillator_fast(void)
{
    OSCCONbits.SPLLEN = 0; //PLL disabled
    OSCCONbits.IRCF = 0b1111; // IRCF 16MHz  
    OSCCONbits.SCS = 0b10;    //system clock select
    while(OSCSTATbits.HFIOFR == 0);
    CCPR1H = 0x20;
    CCPR1L = 0x00;
    TMR1H = 0x00;
    TMR1L = 0x00;
}
void initial_port(void)
{
    // ADC AD input port configuration
    ANSELBbits.ANSB0 = 1;
    ANSELBbits.ANSB1 = 1;
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    PORTBbits.RB0 = 0;
    PORTBbits.RB1 = 0;
    //configure I/0 port  debug
    TRISAbits.TRISA2 = 0;
    PORTAbits.RA2 = 0;
    TRISC1 = 0;
    RC1 = 0;
    TRISC6 = 0;
    RC6 = 0;
    // I/O  spi port configuration
    APFCON1bits.SDOSEL = 1; //RB5
    APFCON1bits.SDISEL = 0;// RC4
    APFCON1bits.SCKSEL = 0;//RC3
    APFCON2bits.SSSEL = 0b10; // RB4

    ANSELBbits.ANSB5 = 0;
    ANSELCbits.ANSC4 = 0;
    ANSELCbits.ANSC3 = 0;
    ANSELBbits.ANSB4 = 0;
    TRISBbits.TRISB5 = 0;
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC3 = 0;
    TRISBbits.TRISB4 = 1;
    RB5 = 0; //MISO
    RC4 = 0; // MOSI
    RC3 = 0 ; //SCLK
    RB4 = 1; // CS_MCU

    // SET I/0 for input 
    TRISA6 = 1;
    RA6 = 0;
    TRISA7 = 1;
    RA7 =0;
    TRISC2 = 1;
    RC2 = 0;
}
void initial_isr(void)
{
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    PIE1bits.SSP1IE = 1;
    PIR1bits.SSP1IF = 0;
}
void initial_fvr(void)
{
   // configure ADFVR  reference  2.048V
    FVRCONbits.FVREN = 1;
    FVRCONbits.ADFVR = 0b10;
}
void initial_adc (void)
{
    // ADCON0 configuration
    ADCON0bits.ADON = 1;
    ADCON0bits.GO_nDONE = 0;
    ADCON0bits.CHS = 0b01010;
    ADCON0bits.ADRMD = 0;
    //ADCON1 configuration
    ADCON1bits.ADFM = 1;
    ADCON1bits.ADCS = 0b111;
    ADCON1bits.ADNREF = 0;  // Vref - is connected to Vss
    ADCON1bits.ADPREF = 0b11; // Vref+ is connected internally to FVR buffer 1
   // ADCON2 configuration
    ADCON2bits.CHSN = 0b1100;
    ADCON2bits.TRIGSEL = 0b0001;
    // ADC interrupt configuration
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 0;


}
void initial_ccp_tmr1(void)
{
    CCPR1H = 0x01;
    CCPR1L = 0x00;
    CCP1CONbits.CCP1M = 0b1011;

    TMR1H = 0x00;
    TMR1L = 0x00;
    T1CONbits.T1CKPS = 0b00;//Timer1 Input Clock Prescale Select bits
    T1CONbits.T1OSCEN = 0;
    T1CONbits.TMR1CS = 0b00;//Timer1 clock source is instruction clock (FOSC/4)(FOSC = 500KHz)
    T1CONbits.TMR1ON = 1;
    T1CONbits.nT1SYNC = 0;
    T1GCONbits.TMR1GE = 0;

    // initial interrupt
    PIE1bits.CCP1IE = 0;
   
}
void initial_spi (void)
{
    //SSPCON1  configuration
    SSPCON1bits.CKP = 0;
    SSPCON1bits.SSPM = 0b0100;
    SSPCON1bits.SSPEN = 1;
    SSPCON1bits.SSPOV = 0;
    SSPCON1bits.WCOL = 0;
    //SSPSTAT configuration
    SSPSTATbits.SMP = 0;
    SSPSTATbits.CKE = 0;
    SSPCON3bits.BOEN = 0;

}

interrupt void isr (void)
{
    uint8_t dummy;
    //spi interrupt
    if(PIE1bits.SSP1IE & PIR1bits.SSP1IF)
    {
        PIR1bits.SSP1IF = 0;
        dummy = SSPBUF;
        if(adc_buf_ridx < 1000)
        {
            SSPBUF = adc_buf_read_ptr[adc_buf_ridx++];
        }
        else
        {
            RA2 = 0;
            RC1 = 0;
            RC6 = 0;
            start_oscillator_low();
        }
    }
}

void delay_ms(unsigned int ms)
{
  unsigned char i;
  do {
    i = 4;
    do {
      delay_us(164);
       } while(--i);
     } while(--ms);
}
int main(int argc, char** argv)
{
    uint8_t dummy;

    start_oscillator_low();
    delay_ms(5);
    initial_port();
    initial_isr();
    initial_fvr();
    initial_adc();
    initial_ccp_tmr1();
    initial_spi();
    ADCON0bits.ADGO = 1;
    while (1)
    {
        while(ADIF == 0);
        ADIF = 0;
        PIR1bits.CCP1IF = 0;
        
        adc_buffer[adc_buf_wptr][adc_buf_widx++] = ((uint16_t)ADRESL << 8) | ADRESH;
        if(adc_buf_widx >= 500)
        {
            adc_buf_rptr = adc_buf_wptr;
            adc_buf_ridx = 0;
            adc_buf_read_ptr = (uint8_t *)&(adc_buffer[adc_buf_rptr][0]);
            dummy = SSPBUF;
            SSPBUF = adc_buf_read_ptr[adc_buf_ridx++];
            // Interrupt BLE
            RA2 = 1;
            RC1 = 1;
            RC6 = 1;

            start_oscillator_fast();
            adc_buf_widx = 0;
            adc_buf_wptr = (adc_buf_wptr + 1) % 2;
        }
        
        if(adc_buf_widx == 250)
        {
            RA2 = 0;
            RC1 = 0;
            RC6 = 0;
        }
       
    }


    return (EXIT_SUCCESS);
}


