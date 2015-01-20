TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_NRF6310

include ./Makefile.custom

# application source
C_SOURCE_FILES += main.c

C_SOURCE_FILES += ble_ido.c
C_SOURCE_FILES += ble_bas.c
C_SOURCE_FILES += ble_dis.c
C_SOURCE_FILES += ble_dfu.c
C_SOURCE_FILES += ble_acc.c
C_SOURCE_FILES += ble_time.c
C_SOURCE_FILES += ble_memdump.c
C_SOURCE_FILES += ble_srv_common.c

C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += ble_conn_params.c
C_SOURCE_FILES += crc16.c
C_SOURCE_FILES += app_trace.c
C_SOURCE_FILES += app_gpiote.c
C_SOURCE_FILES += app_timer.c
C_SOURCE_FILES += spi_master.c
C_SOURCE_FILES += app_scheduler.c
C_SOURCE_FILES += UTCtime_convert.c
C_SOURCE_FILES += temp_service.c
C_SOURCE_FILES += temp_state.c
C_SOURCE_FILES += cmd_buffer.c
C_SOURCE_FILES += ble_radio_notification.c
C_SOURCE_FILES += recorder.c
C_SOURCE_FILES += flash_helper.c
C_SOURCE_FILES += persistent.c
C_SOURCE_FILES += battery.c
C_SOURCE_FILES += bootloader_util_gcc.c
C_SOURCE_FILES += dfu_app_handler.c

OUTPUT_FILENAME := ble_app_hts

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_SOFTDEVICE := S110_FLASH
#USE_SOFTDEVICE := S110

CFLAGS := -DS110_FLASH -O2 -DDEBUG_STATS -DDEBUG_NRF_USER -DBLE_STACK_SUPPORT_REQD -DSPI_MASTER_0_ENABLE -DSPI_MASTER_1_ENABLE
#CFLAGS := -O2 -DDEBUG_NRF_USER -DDEBUG_STATS -DBLE_STACK_SUPPORT_REQD -DSPI_MASTER_0_ENABLE -DSPI_MASTER_1_ENABLE

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=0 -D__STACK_SIZE=1024

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections

# let linker to dump unused sections
LDFLAGS := -Wl,--gc-sections

INCLUDEPATHS += -I"../Include/s110"
INCLUDEPATHS += -I"../Include/ble"
INCLUDEPATHS += -I"../Include/ble/ble_services"
INCLUDEPATHS += -I"../Include/app_common"
INCLUDEPATHS += -I"../Include/app"
INCLUDEPATHS += -I"../Include/sd_common"
INCLUDEPATHS += -I"../Include/sdk"
INCLUDEPATHS += -I"../Include/bootloader_dfu"

C_SOURCE_PATHS += ../Source/ble
C_SOURCE_PATHS += ../Source/app_common
C_SOURCE_PATHS += ../Source/app
C_SOURCE_PATHS += ../Source/sd_common

include Makefile.common
