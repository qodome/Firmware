TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_IMO

include ./Makefile.custom

# application source
C_SOURCE_FILES += main.c
C_SOURCE_FILES += uart.c
C_SOURCE_FILES += app_timer_lock.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += nrf_assert.c
C_SOURCE_FILES += ble_memdump.c
C_SOURCE_FILES += app_scheduler.c
C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += ble_conn_params_s130.c
C_SOURCE_FILES += ble_db_discovery_s130.c
C_SOURCE_FILES += ble_hts_c.c
C_SOURCE_FILES += nrf_pwm.c
C_SOURCE_FILES += ble_led.c
C_SOURCE_FILES += spi_slave.c
C_SOURCE_FILES += intermcu_spi.c

OUTPUT_FILENAME := s130_demo_app

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_SOFTDEVICE := S130

CFLAGS := -DDEBUG_NRF -DNRF51 -DPCA10001 -DBLE_STACK_SUPPORT_REQD -DSPI_MASTER_0_ENABLE -DSPI_MASTER_1_ENABLE

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=0 -D__STACK_SIZE=1024

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -O2

# let linker to dump unused sections
LDFLAGS := -Wl,--gc-sections 

INCLUDEPATHS += -I"$(SDK_PATH)Include/s130"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/device_manager"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk"

C_SOURCE_PATHS += $(SDK_PATH)Source/ble
C_SOURCE_PATHS += $(SDK_PATH)Source/app_common
C_SOURCE_PATHS += $(SDK_PATH)Source/sd_common

include $(SDK_PATH)Source/templates/gcc/Makefile.common
