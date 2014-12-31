/*
 * Software SPI interface to work with pressure sensor
 */
#include "SoftwareSPI.h"

#if defined(OLD_BOARD)

#define CS_HIGH()   (P0_2 = 1)
#define CS_LOW()    (P0_2 = 0)
#define SCK_HIGH()  (P0_3 = 1)
#define SCK_LOW()   (P0_3 = 0)
#define MO_HIGH()  (P0_4 = 1)
#define MO_LOW()   (P0_4 = 0)
#define READ_MI() ((P0 & BV(5)) >> 5)

#else

#define CS_HIGH()   (P0_2 = 1)
#define CS_LOW()    (P0_2 = 0)
#define SCK_HIGH()  (P0_5 = 1)
#define SCK_LOW()   (P0_5 = 0)
#define MO_HIGH()  (P0_3 = 1)
#define MO_LOW()   (P0_3 = 0)
#define READ_MI() ((P0 & BV(4)) >> 4)

#endif

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
#if defined(OLD_BOARD)
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
    P0DIR |= BV(4) | BV(2) | BV(3);
    // Set input pin IO direction
    P0DIR &= ~(BV(5));

    P0DIR &= ~(BV(0) | BV(1));
    
    P0_2 = 1;
    P0_4 = P0_3 = 0;

    CS_HIGH();
    SCK_HIGH();
    MO_LOW();
#else
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
#endif
}

void SPI_intf_reset (void)
{
    CS_LOW();
    
    for (uint8 i=0; i<34; i++)    // at least 32
    {
        SCK_LOW();
        MO_HIGH();
        delay_L();
        SCK_HIGH();
        delay_L();
    }
    
    CS_HIGH();
}

static uint8 SPI_rcvb (void)
{
    uint8 inData = 0;
    for(uint8 i = 0; i < 8; i++) {
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
    for (uint8 i = 0; i < 8; i++) {
        SCK_LOW();
        if ((outData << i) & 0x80) {
            MO_HIGH();
        } else {
            MO_LOW();
        }
        delay_L();
        SCK_HIGH();
        delay_L();
    }
}

uint16 spi_read (uint8 addr)
{
    uint16 ret;

    if (addr > 0x07) {
      return 0;
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

// FIXME: spi_write only writes to 0x01!
void spi_write (uint8 addr, uint16 val)
{
  //PMUX = 0x00;
  //P1_0 = 0;
   
  CS_LOW();
#if 0
  if (addr == 0x01 || addr == 0x05) {
#endif
    SPI_sendb(((addr & 0x07) << 3) & ~(0x40));
    SPI_sendb((uint8)(val & 0xFF));
#if 0
  } else if (addr == 0x04 || addr == 0x06 || addr == 0x07) {
    SPI_sendb(((addr & 0x07) << 3) & ~(0x40));
    SPI_sendb((uint8)((val >> 8) & 0xFF));    
    SPI_sendb((uint8)(val & 0xFF));
  }
#endif
  CS_HIGH();
    
  //PMUX = 0x08;
}
