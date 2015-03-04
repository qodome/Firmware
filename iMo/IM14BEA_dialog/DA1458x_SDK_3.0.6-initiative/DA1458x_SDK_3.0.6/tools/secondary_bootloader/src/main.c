/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief Secondary Bootloader application
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */
#include <stdio.h>
#include "global_io.h"
#include "core_cm0.h"
#include "periph_setup.h"
#include "uart_booter.h"
#include "i2c_eeprom.h"
#include "bootloader.h"
#include "spi_commands.h"
#include "gpio.h"
#include "uart.h"
  
 void Start_run_user_application(void);
__asm void sw_reset(void);

/**
****************************************************************************************
* @brief set system clocks
****************************************************************************************
**/
void set_system_clocks(void)   // set the hclk and pclk
{
    BYTE i;
    BYTE j;

    for (i = 0; i < 4; i++) 
    {        
        SetBits16( CLK_AMBA_REG, PCLK_DIV, i);
        for (j = 0; j < 4; j++)
        {        
           SetBits16( CLK_AMBA_REG, HCLK_DIV, j);
        }
    }
    SetWord16(CLK_AMBA_REG, 0x00); //fastest
} 
 
/**
****************************************************************************************
* @brief sw reset 
****************************************************************************************
*/
__asm void sw_reset(void)
{
  LDR r0,=0x20000000
  LDR r1,[r0,#0]
  MOV sp,r1
  LDR r2,[r0,#4]
  BX r2
}

 /**
****************************************************************************************
* @brief Run the user application after receiving a bimany from uart or reading the binary from a non volatile memory booting 
****************************************************************************************
*/
void Start_run_user_application(void)
{
    volatile unsigned short tmp;
#ifdef SPI_FLASH_SUPPORTED
    spi_release();
    spi_disable();
#endif
    if (*(volatile unsigned*)0x20000000 & 0x20000000)
    {
        tmp = GetWord16(SYS_CTRL_REG);
        tmp&=~0x0003;
        SetWord16(SYS_CTRL_REG,tmp);
        sw_reset();
    }
    tmp = GetWord16(SYS_CTRL_REG);
    //  tmp&=~0x0003;
    tmp|=0x8000;
    SetWord16(SYS_CTRL_REG,tmp);
}
 extern int FwDownload(void); 
int main (void)
{ 
    int ret;
#ifdef UART_SUPPORTED
    int fw_size;
    unsigned short i;
    char* from ; 
    char*  to;
#endif

    SetWord16(SET_FREEZE_REG,FRZ_WDOG);
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP,0);   // exit peripheral power down
    // Power up peripherals' power domain 
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP));  

    #ifdef SUPPORT_1_8_V
        SetBits16(DCDC_CTRL2_REG, DCDC_VBAT3V_LEV, 0x0);   ///--Support 1.8V boot
    #endif

    while (1){
#ifdef UART_SUPPORTED
        if (GPIO_GetPinStatus (GPIO_PORT_0,GPIO_PIN_5)){ 
            fw_size = FwDownload();
            if (fw_size>0){
                from=(char*) (SYSRAM_COPY_BASE_ADDRESS);  
                to=(char*) (SYSRAM_BASE_ADDRESS);         
                for(i=0;i<fw_size;i++)  to[i] = from[i]; 

                SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms active time for initialization!
                SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG
                Start_run_user_application();
            }
        }
#endif
#ifdef SPI_FLASH_SUPPORTED
        ret =spi_loadActiveImage();
#else 
        ret = eeprom_loadActiveImage();
#endif
        if (!ret) {
            SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms active time for initialization!
            SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG
            Start_run_user_application();
        }
    }
} 

