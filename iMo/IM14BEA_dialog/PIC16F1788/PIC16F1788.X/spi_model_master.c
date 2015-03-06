/*
 * SPI model 
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
// PIC16F1788 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>
#include <pic.h>
#include "spi_model.h"


#define CS_HIGH()   (RC2 = 1)
#define CS_LOW()    (RC2 = 0)
#define SCK_HIGH()  (RC3 = 1)
#define SCK_LOW()   (RC3 = 0)
#define MO_HIGH()  (RA7 = 1)
#define MO_LOW()   (RA7 = 0)
#define READ_MI() ((P0 & BV(4)) >> 4)


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
    ANSELAbits.ANSA7 = 0;
    ANSELCbits.ANSC2 = 0;
    ANSELCbits.ANSC3 = 0;
    TRISAbits.TRISA6 = 1;
    TRISAbits.TRISA7 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC2 = 0;
    RA6 = 0; //MISO
    RA7 = 0; // MOSI
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
    uint8 i = 0;

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
        delay_L();
        SCK_HIGH();
        delay_L();
    }
}

uint16 spi_read (uint8 addr)
{
    uint16 ret = 0;
    if (addr > 0x07) {
      return ret;
    }

    //PMUX = 0x00;
    //P1_0 = 0;

    CS_LOW();
    if (addr == 0x02 || addr >= 0x04) {
      SPI_sendb(((addr & 0x07) << 3) | 0x40);
      ret = (SPI_rcvb() << 8) | SPI_rcvb();
    } else {
      SPI_sendb(((addr & 0x07) << 3) | 0x40);
      ret = (uint16)SPI_rcvb();
    }
    CS_HIGH();

    //PMUX = 0x08;

    return ret;
}

void spi_write (uint8 addr, uint16 val)
{
  //PMUX = 0x00;
  //P1_0 = 0;

  CS_LOW();
  if (addr == 0x01 || addr == 0x05) {
    SPI_sendb(((addr & 0x07) << 3) & ~(0x40));
    SPI_sendb((uint8)(val & 0xFF));
  } else if (addr == 0x04 || addr == 0x06 || addr == 0x07) {
    SPI_sendb(((addr & 0x07) << 3) & ~(0x40));
    SPI_sendb((uint8)((val >> 8) & 0xFF));
    SPI_sendb((uint8)(val & 0xFF));
  }
  CS_HIGH();

  //PMUX = 0x08;
}

