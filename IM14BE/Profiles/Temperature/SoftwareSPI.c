/*
 * Software SPI interface to work with pressure sensor
 */
#include "SoftwareSPI.h"

void delay_L()
{
    uint8 i;
    for (i = 0; i < 2; i++) {
       asm("nop");
    }
}

/*
 * Initialize IO port
 */
void softwareSPIInit (void)
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
    // All GPIO function
    P0SEL = 0x00;

    // Set output pin IO directions
    P0DIR |= BV(5) | BV(2) | BV(3);
    // Set input pin IO direction
    P0DIR &= ~(BV(4));

    CS_HIGH();
    SCK_LOW();
    MO_LOW();

    CS_HIGH();
    SCK_HIGH();
    MO_LOW();
}

static uint8 SPI_rcvb (void)
{
    uint8 i = 0, inData = 0;
    for(i=0; i<8; i++)
    {
      SCK_LOW();
      delay_L();
      SCK_HIGH();
      inData <<= 1;
      inData |= READ_MI();
      delay_L();
    }
    return(inData);
}

static void SPI_sendb (uint8 outData)
{
    uint8 i = 0;
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
