/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "bootloader_util.h"
#include <stdint.h>
#include "nordic_common.h"
#include "bootloader_types.h"
#include "dfu_types.h"


#if __GNUC__ && __ARM_EABI__

#ifdef S110_FLASH
__attribute__ ((section(".bootloader_settings_sect"))) uint8_t m_boot_settings[1024] = {BANK_VALID_APP};
const bootloader_settings_t const * const mp_bootloader_settings = (bootloader_settings_t *) &m_boot_settings[0];   /**< Read only pointer to bootloader settings in flash. */
#endif

/*
static inline void StartApplication(uint32_t start_addr)
{
    __asm volatile("LDR   R2, [R0]\t\n"
          "MSR   MSP, R2\t\n"
          "LDR   R3, [R0, #0x00000004]\t\n"
          "BX    R3\t\n"
          ".ALIGN\t\n");
}
*/

void isr_abort(uint32_t reset_handler)
{
	__asm volatile(
    "LDR   R4, =0xFFFFFFFF	\t\n"
    "LDR   R5, =0xFFFFFFFF	\t\n"
    "MOV   R6, R0			\t\n"
    "LDR   R7, =0x21000000	\t\n"
    "PUSH  {r4-r7}			\t\n"
    "LDR   R4, =0x00000000	\t\n"
    "LDR   R5, =0x00000000	\t\n"
    "LDR   R6, =0x00000000	\t\n"
    "LDR   R7, =0x00000000	\t\n"
    "PUSH  {r4-r7}			\t\n"
    "LDR   R0, =0xFFFFFFF9	\t\n"
    "BX    R0				\t\n");
}

static inline void StartApplication(uint32_t start_addr)
{
    __asm volatile(
    "LDR   R1, [R0]  \t\n"
    "MSR   MSP, R1   \t\n"
    "LDR   R0,[R0, #0x04]  \t\n"

    "LDR   R2, =0x00000000 	\t\n"
    "MRS   R3, IPSR			\t\n"
    "CMP   R2, R3			\t\n"
    "BNE   isr_abort		\t\n"

    "LDR   R4, =0xFFFFFFFF	\t\n"
    "MOV   LR, R4			\t\n"
    "BX    R0				\t\n");
}

#endif


void bootloader_util_app_start(uint32_t start_addr)
{
    StartApplication(start_addr);
}


void bootloader_util_settings_get(const bootloader_settings_t ** pp_bootloader_settings)
{
#ifdef S110_FLASH
    *pp_bootloader_settings = mp_bootloader_settings;
#endif
}

