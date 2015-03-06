/*
 * File:   main_ccp.c
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
uint8_t num = 0;
void initial (void)
{
    // system oscillator
    OSCCONbits.SPLLEN = 0;
    OSCCONbits.SCS = 0b10;
    OSCCONbits.IRCF = 0b0011;

    // RA2 I/0 configuration
    TRISAbits.TRISA2 = 0;
    PORTAbits.RA2 = 0;
    TMR2 = 0;
    PR2 = 0xFF;
    //TMR2 regisger configuration
    
    T2CONbits.T2CKPS = 0b11;
    T2CONbits.T2OUTPS = 0b0000;
    T2CONbits.TMR2ON = 1;

   //TMR2  interrupt
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    PIE1bits.TMR2IE = 1;

}

interrupt void timer2 (void)
{
    PIR1bits.TMR2IF = 0;
    TMR2 = 0;
    PR2 = 0xFF;
    num++;

}
int main(int argc, char** argv) {
    initial();
    while(1){
        if(num == 1)
        {
            RA2 = 1;
        }
        if(num == 2)
        {
            RA2 = 0;
            num = 0;
        }
    }

    return (EXIT_SUCCESS);
}

