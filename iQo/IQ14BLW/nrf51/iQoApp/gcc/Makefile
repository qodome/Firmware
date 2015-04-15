TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_IQO

SDK_PATH := ../

# application source
C_SOURCE_FILES += main.c
C_SOURCE_FILES += app_timer.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += ble_memdump.c
C_SOURCE_FILES += app_scheduler.c
C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += ble_conn_params_s130.c
C_SOURCE_FILES += ble_db_discovery.c
C_SOURCE_FILES += nrf_pwm.c
C_SOURCE_FILES += ble_led.c
C_SOURCE_FILES += ble_sensor.c
C_SOURCE_FILES += ble_iqo.c
C_SOURCE_FILES += ble_iqo_c.c
C_SOURCE_FILES += spi_slave.c
C_SOURCE_FILES += intermcu_spi.c
C_SOURCE_FILES += persistent.c
C_SOURCE_FILES += flash_helper.c
C_SOURCE_FILES += bootloader_util_gcc.c
C_SOURCE_FILES += led_service.c
C_SOURCE_FILES += ble_radio_notification.c
C_SOURCE_FILES += ble_flash.c
C_SOURCE_FILES += twi_hw_master.c
C_SOURCE_FILES += sensor_service.c

OUTPUT_FILENAME := iQoApp

DEVICE_VARIANT := xxaa

USE_SOFTDEVICE := S130_FLASH
#USE_SOFTDEVICE := S130

CFLAGS := -DS130_FLASH -O2 -DDEBUG_NRF -DNRF51 -DBLE_STACK_SUPPORT_REQD -DSPI_MASTER_0_ENABLE -DSPI_MASTER_1_ENABLE
#CFLAGS := -O2 -DDEBUG_NRF -DNRF51 -DBLE_STACK_SUPPORT_REQD -DSPI_MASTER_0_ENABLE -DSPI_MASTER_1_ENABLE

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=0 -D__STACK_SIZE=1024

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -O2

# let linker to dump unused sections
LDFLAGS := -Wl,--gc-sections 

INCLUDEPATHS += -I"$(SDK_PATH)Include/s130_0.9.0-1.alpha"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/device_manager"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/bootloader_dfu"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app"

C_SOURCE_PATHS += $(SDK_PATH)Source/ble
C_SOURCE_PATHS += $(SDK_PATH)Source/app_common
C_SOURCE_PATHS += $(SDK_PATH)Source/sd_common

include Makefile.common
