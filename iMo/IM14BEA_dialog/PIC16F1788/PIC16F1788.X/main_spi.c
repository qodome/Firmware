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

#define CS_HIGH()   (RC2 = 1)
#define CS_LOW()    (RC2 = 0)
#define SCK_HIGH()  (RC3 = 1)
#define SCK_LOW()   (RC3 = 0)
#define MO_HIGH()  (RA2 = 1)
#define MO_LOW()   (RA2 = 0)
#define READ_MI()   RA6


void delay()
{
    uint8_t i;
    for (i = 0; i < 2; i++)
    {
       asm("nop");
    }
}

/*
 * Initialize IO port
 */
void initial_spi_port (void)
{
	/*
	 * P0_1/GPIO/SD: chip power down, effective high, initialized to OUT 0
	 * P0_4/MOSI/SDI: input-ouput duplexed data line, initialized to OUT 0
	 * P0_2/SS/CSB: chip select, effective low, initialized to OUT 1
	 * P0_3/SCK/SCL: chip clock, initialized to OUT 0
	 *
	 * Pins not used currently:
	 * P0_5/MISO/SDO: not used, set to OUT
	 * P0_0/INT1/INT1: interrupt input line, set to IN
	 */

    // I/O  spi port configuration
    ANSELAbits.ANSA2 = 0;
    ANSELCbits.ANSC2 = 0;
    ANSELCbits.ANSC3 = 0;
    TRISAbits.TRISA6 = 1;
    TRISAbits.TRISA2 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC2 = 0;
    RA6 = 0; //MISO
    RA2 = 0; // MOSI
    RC3 = 0 ; //SCLK
    RC2 = 0; // CS_MCU

    CS_HIGH();
    SCK_LOW();
    MO_LOW();

    CS_HIGH();
    SCK_HIGH();
    MO_LOW();

}

void SPI_intf_reset (void)
{
    uint8_t i = 0;

    CS_LOW();

    for (i=0; i<34; i++)    // at least 32
    {
        SCK_LOW();
        MO_HIGH();
        delay();
        SCK_HIGH();
        delay();
    }

    CS_HIGH();
}

static uint8_t spi_receive_byte (void)
{
    uint8_t i = 0, inData = 0;
    for(i=0; i<8; i++)
    {
      SCK_LOW();
      delay();
      SCK_HIGH();
      inData <<= 1;
      inData |= READ_MI();
      delay();
    }
    return(inData);
}

static void spi_send_byte (uint8_t outData)
{
    uint8_t i = 0;
    for (i=0; i<8; i++)
    {
        SCK_LOW();
        if((outData<<i)&0x80)
        {
            MO_HIGH();
        }
        else
        {
            MO_LOW();
        }
        delay();
        SCK_HIGH();
        delay();
    }
}

int main(int argc, char** argv)
{
    initial_spi_port();
    SPI_intf_reset();
    
    while (1)
    {
        delay();
        CS_LOW();
        spi_send_byte(0x55);
        delay();
        CS_HIGH();
    }
    return (EXIT_SUCCESS);
}
