/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00016000, LENGTH = 0xF000					/* FIXME: each time modified code, check hex file to make sure image size does not exceed limit! */
  bootloader_settings (rwx) : ORIGIN = 0x0003FC00, LENGTH = 0x400
  RAM (rwx) :  ORIGIN = 0x20002000, LENGTH = 0x2000 
}

CodeOrigin = ORIGIN(FLASH);
CodeLength = LENGTH(FLASH);
BootloaderOrigin = 0x00000000;	/* dummy */

INCLUDE "gcc_nrf51_common.ld"
