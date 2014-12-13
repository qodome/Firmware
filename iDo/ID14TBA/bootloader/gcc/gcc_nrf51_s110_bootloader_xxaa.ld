
SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00037800, LENGTH = 0x8400
  bootloader_settings (rwx) : ORIGIN = 0x0003FC00, LENGTH = 0x400
  NRF_UICR_BOOT_START (rwx) : ORIGIN = 0x10001014, LENGTH = 0x4
  RAM (rwx) :  ORIGIN = 0x20002000, LENGTH = 0x2000
}

BootloaderOrigin = ORIGIN(FLASH);

INCLUDE "gcc_nrf51_common.ld"
