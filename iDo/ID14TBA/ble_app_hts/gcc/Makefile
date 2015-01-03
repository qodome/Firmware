TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_NRF6310

include ./Makefile.custom

# application source
C_SOURCE_FILES += main.c

C_SOURCE_FILES += ble_ido.c
C_SOURCE_FILES += ble_bas_s110.c
C_SOURCE_FILES += ble_dis.c

C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += ble_srv_common.c
C_SOURCE_FILES += ble_sensorsim.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += ble_debug_assert_handler.c
C_SOURCE_FILES += ble_error_log.c
C_SOURCE_FILES += ble_conn_params_s110.c
C_SOURCE_FILES += pstorage.c
C_SOURCE_FILES += crc16.c
C_SOURCE_FILES += app_trace.c
C_SOURCE_FILES += simple_uart.c
C_SOURCE_FILES += app_gpiote.c
C_SOURCE_FILES += app_button.c
C_SOURCE_FILES += app_timer_lock.c
C_SOURCE_FILES += spi_master_io.c
C_SOURCE_FILES += app_scheduler.c
C_SOURCE_FILES += UTCtime_convert.c
C_SOURCE_FILES += temp_service.c
C_SOURCE_FILES += temp_date_time.c
C_SOURCE_FILES += ble_memdump.c
C_SOURCE_FILES += temp_state.c
C_SOURCE_FILES += cmd_buffer.c
C_SOURCE_FILES += ble_radio_notification_off.c
C_SOURCE_FILES += recorder.c
C_SOURCE_FILES += flash_helper.c
C_SOURCE_FILES += persistent.c
C_SOURCE_FILES += battery.c
C_SOURCE_FILES += device_manager_peripheral.c
C_SOURCE_FILES += bootloader_util_gcc_hts.c
C_SOURCE_FILES += ble_dfu.c
C_SOURCE_FILES += dfu_app_handler_hard_reset.c

OUTPUT_FILENAME := ble_app_hts

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_SOFTDEVICE := S110_FLASH
#USE_SOFTDEVICE := S110

CFLAGS := -DS110_FLASH -O2 -DDEBUG_STATS -DDEBUG_NRF_USER -DBLE_STACK_SUPPORT_REQD -DSPI_MASTER_0_ENABLE -DSPI_MASTER_1_ENABLE
#CFLAGS := -O2 -DDEBUG_NRF_USER -DDEBUG_STATS -DBLE_STACK_SUPPORT_REQD -DSPI_MASTER_0_ENABLE -DSPI_MASTER_1_ENABLE

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=0

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections

# let linker to dump unused sections
LDFLAGS := -Wl,--gc-sections

INCLUDEPATHS += -I"$(SDK_PATH)Include/s110"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/device_manager"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk"
INCLUDEPATHS += -I"$(SDK_PATH)Include/bootloader_dfu"

C_SOURCE_PATHS += $(SDK_PATH)Source/ble
C_SOURCE_PATHS += $(SDK_PATH)Source/ble/device_manager
C_SOURCE_PATHS += $(SDK_PATH)Source/app_common
C_SOURCE_PATHS += $(SDK_PATH)Source/sd_common

include Makefile.common
