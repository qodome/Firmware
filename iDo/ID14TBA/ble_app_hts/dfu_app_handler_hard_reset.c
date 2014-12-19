/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
 
#include "dfu_app_handler.h"
#include "bootloader_util.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "persistent.h"
#include "flash_helper.h"
#include <stdlib.h>
#include <string.h>

#define IRQ_ENABLED             0x01                                            /**< Field identifying if an interrupt is enabled. */
#define MAX_NUMBER_INTERRUPTS   32                                              /**< Maximum number of interrupts available. */

static void                     dfu_app_reset_prepare(void);                    /**< Forward declare of default reset handler. */
static dfu_app_reset_prepare_t  m_reset_prepare = dfu_app_reset_prepare;        /**< Callback function to application to prepare for system reset. Allows application to cleanup of service and memory prior to reset. */


/**@brief Default reset prepare handler if application hasn't registered a handler.
 */
static void dfu_app_reset_prepare(void)
{
    // Reset prepare should be handled by application.
    // This function can be extended to include default handling if application does not implement
    // own handler.
}


/**@brief Function for disabling all interrupts before jumping from bootloader to application.
 */
static void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint32_t irq = 0; // We start from first interrupt, i.e. interrupt 0.

    // Fetch the current interrupt settings.
    interrupt_setting_mask = NVIC->ISER[0];
    
    for (; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (IRQ_ENABLED << irq))
        {
            // The interrupt was enabled, and hence disable it.
            NVIC_DisableIRQ((IRQn_Type)irq);
        }
    }        
}

/**@brief Function for preparing the reset, disabling SoftDevice and jump to the bootloader.
 */
static void bootloader_start(void)
{
	uint16_t loop_cnt = 0;
	uint8_t boot_magic[8] = {'B', 'O', 'O', 'T', 'O', 'A', 'D', 0};
	uint8_t test_read[8] = {0};
	uint32_t err_code = 0;

	// Signal bootloader
	do {
		err_code = sd_flash_page_erase((uint32_t)persistent_flash_page());
		APP_ERROR_CHECK(sd_app_evt_wait());
	} while (err_code == NRF_ERROR_BUSY);

	do {
		err_code = sd_flash_write((uint32_t *)((uint32_t)persistent_flash_page() * 1024), (uint32_t *)boot_magic, 2);
	} while (err_code == NRF_ERROR_BUSY);

    loop_cnt = 0;
    do {
    	APP_ERROR_CHECK(sd_app_evt_wait());
    	HalFlashRead(persistent_flash_page(), 0, test_read, 8);
    	if (memcmp(boot_magic, test_read, 8) == 0) {
    		break;
    	}
    	loop_cnt++;
    } while (loop_cnt < 100);

    m_reset_prepare();

    err_code = sd_power_gpregret_set(BOOTLOADER_DFU_START);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_softdevice_disable();
    APP_ERROR_CHECK(err_code);

    interrupts_disable();

    err_code = sd_softdevice_vector_table_base_set(NRF_UICR->BOOTLOADERADDR);
    APP_ERROR_CHECK(err_code);

    // Watchdog will knock us down
    while (1);

    //bootloader_util_app_start(NRF_UICR->BOOTLOADERADDR);
}


void dfu_app_on_dfu_evt(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
{
    switch (p_evt->ble_dfu_evt_type)
    {
        uint32_t err_code;
        case BLE_DFU_START:
            // Starting the bootloader - will cause reset.
            bootloader_start();
            break;

        case BLE_DFU_VALIDATE:
        case BLE_DFU_ACTIVATE_N_RESET:
        case BLE_DFU_SYS_RESET:
        case BLE_DFU_RECEIVE_INIT_DATA:
        case BLE_DFU_RECEIVE_APP_DATA:
        case BLE_DFU_PACKET_WRITE:
        case BLE_DFU_PKT_RCPT_NOTIF_ENABLED:
        case BLE_DFU_PKT_RCPT_NOTIF_DISABLED:
        case BLE_DFU_BYTES_RECEIVED_SEND:
        default:
            // Unsupported event received from DFU Service. 
            // Send back BLE_DFU_RESP_VAL_NOT_SUPPORTED message to peer.
            err_code = ble_dfu_response_send(p_dfu,
                                             BLE_DFU_START_PROCEDURE,
                                             BLE_DFU_RESP_VAL_NOT_SUPPORTED);
            APP_ERROR_CHECK(err_code);
            break;
    }
}


void dfu_app_reset_prepare_set(dfu_app_reset_prepare_t reset_prepare_func)
{
    m_reset_prepare = reset_prepare_func;
}
