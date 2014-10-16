#include "ble_ecg.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ecg       ECG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_ecg_t * p_ecg, ble_evt_t * p_ble_evt)
{
    p_ecg->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ecg       ECG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_ecg_t * p_ecg, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_ecg->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the ECG value characteristic.
 *
 * @param[in]   p_ecg         ECG Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_cccd_write(ble_ecg_t * p_ecg, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update indication state
        if (p_ecg->evt_handler != NULL)
        {
            ble_ecg_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_ECG_EVT_NOTIFY_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_ECG_EVT_NOTIFY_DISABLED;
            }

            p_ecg->evt_handler(p_ecg, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ecg       ECG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_ecg_t * p_ecg, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_ecg->ecg_handles.cccd_handle)
    {
        on_cccd_write(p_ecg, p_evt_write);
    }
}

void ble_ecg_on_ble_evt(ble_ecg_t * p_ecg, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ecg, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ecg, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ecg, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding ECG value characteristics.
 *
 * @param[in]   p_ecg        ECG Service structure.
 * @param[in]   p_ecg_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t ecg_v_char_add(ble_ecg_t * p_ecg, const ble_ecg_init_t * p_ecg_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_ecg_v_t      initial_ecg;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    cccd_md.write_perm = p_ecg_init->ecg_attr_md.cccd_write_perm;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = &cccd_md;
    char_md.p_sccd_md           = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ECG_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.read_perm  = p_ecg_init->ecg_attr_md.read_perm;
    attr_md.write_perm = p_ecg_init->ecg_attr_md.write_perm;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    memset(&initial_ecg, 0, sizeof(initial_ecg));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(initial_ecg);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(initial_ecg);
    attr_char_value.p_value   = (uint8_t *)&initial_ecg;

    return sd_ble_gatts_characteristic_add(p_ecg->service_handle, &char_md, &attr_char_value, &p_ecg->ecg_handles);
}

uint32_t ble_ecg_init(ble_ecg_t * p_ecg, const ble_ecg_init_t * p_ecg_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_ecg->evt_handler = p_ecg_init->evt_handler;
    p_ecg->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ECG_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ecg->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add ECG characteristic
    err_code = ecg_v_char_add(p_ecg, p_ecg_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t ble_ecg_v_send(ble_ecg_t * p_ecg, ble_ecg_v_t * p_ecg_v)
{
    uint32_t err_code;

    // Send value if connected
    if (p_ecg->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        hvx_len = sizeof(ble_ecg_v_t);

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ecg->ecg_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = (uint8_t *)p_ecg_v;

        err_code = sd_ble_gatts_hvx(p_ecg->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


uint32_t ble_ecg_is_notify_enabled(ble_ecg_t * p_ecg, bool * p_notify_enabled)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    uint16_t len = BLE_CCCD_VALUE_LEN;

    err_code = sd_ble_gatts_value_get(p_ecg->ecg_handles.cccd_handle, 0, &len, cccd_value_buf);
    if (err_code == NRF_SUCCESS)
    {
        *p_notify_enabled = ble_srv_is_notification_enabled(cccd_value_buf);
    }
    return err_code;
}
