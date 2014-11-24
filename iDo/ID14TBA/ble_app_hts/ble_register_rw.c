/* Copyright (c) 2012 Nordic Semiconductor. All Rigreg Reserved.
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

/* Attention!
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile
*  qualification listings, this section of source code must not be modified.
*/

#include "ble_register_rw.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_reg       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_reg_t * p_reg, ble_evt_t * p_ble_evt)
{
    p_reg->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_reg       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_reg_t * p_reg, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_reg->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_reg       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_reg_t * p_reg, ble_evt_t * p_ble_evt)
{
	uint16_t len = sizeof(ble_reg_value_t);
	ble_reg_value_t *reg_p = NULL;

    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_reg->reg_handles.value_handle)
    {
    	reg_p = (ble_reg_value_t *)&(p_evt_write->data[0]);
    	spi_write((uint8_t)(reg_p->addr & 0xFF), reg_p->value);

    	reg_p->value = spi_read((uint8_t)(reg_p->addr & 0xFF));
        // Update database
        sd_ble_gatts_value_set(p_reg->reg_handles.value_handle,
                                          0,
                                          &len,
                                          (uint8_t *)reg_p);

    }
}

void ble_reg_on_ble_evt(ble_reg_t * p_reg, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_reg, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_reg, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_reg, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding Temperature Type characteristics.
 *
 * @param[in]   p_reg        Health Thermometer Service structure.
 * @param[in]   p_reg_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t reg_value_char_add(ble_reg_t * p_reg, const ble_reg_init_t * p_reg_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    ble_reg_value_t             init_reg;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, 0x5555);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.read_perm  = p_reg_init->reg_type_attr_md.read_perm;
    attr_md.write_perm = p_reg_init->reg_type_attr_md.write_perm;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    init_reg.addr = 0;
    init_reg.value = spi_read(0x00);

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof (ble_reg_value_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof (ble_reg_value_t);
    attr_char_value.p_value   = (uint8_t *)&init_reg;

    return sd_ble_gatts_characteristic_add(p_reg->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_reg->reg_handles);
}

uint32_t ble_reg_init(ble_reg_t * p_reg, const ble_reg_init_t * p_reg_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_reg->evt_handler = p_reg_init->evt_handler;
    p_reg->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, 0x5554);// BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HEALTH_THERMOMETER_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_reg->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add measurement characteristic
    err_code = reg_value_char_add(p_reg, p_reg_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}







