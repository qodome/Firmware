/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x0001D000, LENGTH = 0xF000 
  bootloader_settings (rwx) : ORIGIN = 0x0003FC00, LENGTH = 0x400
  RAM (rwx) :  ORIGIN = 0x20002800, LENGTH = 0x1800 
}

CodeOrigin = ORIGIN(FLASH);
CodeLength = LENGTH(FLASH);
BootloaderOrigin = 0x00000000;	/* dummy */

INCLUDE "gcc_nrf51_common.ld"
