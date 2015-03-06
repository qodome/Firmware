/*
 * File:   main_test.c
 * Author: Administrator
 *
 * Created on January 30, 2015, 5:19 PM
 */

#include <stdio.h>
#include <stdlib.h>

// PIC16F1788 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

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
int num;
void initial_TMR1 (void)
{
    TRISA2 = 0;
    RA2 =0;
    // TMR1 TMR1H TMR1L register initial
    TMR1L = 0x00;
    TMR1H = 0x00;
    // TMR1 configuration T1CON register
    T1CONbits.T1CKPS = 0b01;
    T1CONbits.T1OSCEN = 0;
    T1CONbits.TMR1CS = 0b00;
    T1CONbits.TMR1ON = 1;
    T1CONbits.nT1SYNC = 0;
    // TMR1 configuration T1GCON register
    T1GCONbits.TMR1GE = 0;
    // TMR1 interrupt
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    PIE1bits.TMR1IE = 1;  

}
void initial_ADC (void)
{
    // I/O configuration
    TRISA2 = 0;
    PORTAbits.RA2 =0;
    //FVR configuration  configure FVR to use 2.048V
    FVRCONbits.ADFVR = 0b10;
    FVRCONbits.FVREN = 1;
    // ADCON0 configuration
    ADCON0bits.ADON = 1;
    ADCON0bits.GO_nDONE = 0;
    ADCON0bits.CHS = 0b01100;
    ADCON0bits.ADRMD = 0;
    //ADCON1 configuration
    ADCON1bits.ADFM = 1;
    ADCON1bits.ADCS = 0b111;
    ADCON1bits.ADNREF = 0;  // Vref - is connected to Vss
    ADCON1bits.ADPREF = 0b11; // Vref+ is connected internally to FVR buffer 1
   // ADCON2 configuration
    ADCON2bits.CHSN = 0b1010;
    ADCON2bits.TRIGSEL = 0b0001;

}

interrupt  void timer0(void)
{
    PIR1bits.TMR1IF = 0;
    num++;
    
}

int main(int argc, char** argv) {
    initial();
    while (1) {
        if (num == 1) {
            RA2 = 1;
        }
        if (num == 2) {
            num = 0;
            RA2 = 0;
        }
    }
    return (EXIT_SUCCESS);
}


