TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_IMO

include ./Makefile.custom

C_SOURCE_FILES += main.c
C_SOURCE_FILES += pstorage.c
C_SOURCE_FILES += bootloader.c
C_SOURCE_FILES += dfu_transport_ble_hts.c
C_SOURCE_FILES += dfu_dual_bank.c
C_SOURCE_FILES += app_timer.c
C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += ble_conn_params.c
C_SOURCE_FILES += ble_dfu.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += bootloader_util_gcc.c
C_SOURCE_FILES += nrf_delay.c
C_SOURCE_FILES += crc16.c
C_SOURCE_FILES += hci_mem_pool.c
C_SOURCE_FILES += app_scheduler.c

OUTPUT_FILENAME := bootloader

CFLAGS := -DBLE_STACK_SUPPORT_REQD

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=0

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections

# let linker to dump unused sections
LDFLAGS := -Wl,--gc-sections

#Uncomment correct line if you have s110 programmed on the chip.
DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_SOFTDEVICE := S110_bootloader
#USE_SOFTDEVICE := s210

INCLUDEPATHS += -I"../include/"
INCLUDEPATHS += -I"$(SDK_PATH)Include/s110"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/bootloader_dfu/ble_transport"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/device_manager"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk"
INCLUDEPATHS += -I"$(SDK_PATH)Include/bootloader_dfu"

include Makefile.common
