/*
 * Connects to iQo peer
 */
#include <stdint.h>
#include "ble_iqo_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_srv_common.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "ble_gattc.h"
#include "app_util.h"
#include "app_trace.h"
#include "app_error.h"

#define TX_BUFFER_MASK          0x07
#define TX_BUFFER_SIZE          (TX_BUFFER_MASK + 1)

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN

typedef enum {
    READ_REQ,
    WRITE_REQ,
} tx_request_t;

typedef struct {
    uint8_t gattc_value[WRITE_MESSAGE_LENGTH];
    ble_gattc_write_params_t gattc_params;
} write_params_t;

typedef struct {
    uint16_t conn_handle;
    tx_request_t type;
    union {
        uint16_t read_handle;
        write_params_t write_req;
    } req;
} tx_message_t;

tx_message_t  m_tx_buffer[TX_BUFFER_SIZE];  /* Transmit buffer for messages to be transmitted to the central. */
uint32_t m_tx_insert_index = 0;        /* Current index in the transmit buffer where the next message should be inserted. */
uint32_t m_tx_index = 0;               /* Current index in the transmit buffer from where the next message to be transmitted resides. */

uint8_t iqo_c_status = 0;

extern ble_iqo_c_t *peripheral_get_iqo_c(uint16_t conn_handle);
extern ble_db_discovery_t *peripheral_get_db(uint16_t conn_handle);

/* Function for passing any pending request from the buffer to the stack. */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index) {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ) {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        } else {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS) {
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
    }
}

static void on_write_rsp(ble_iqo_c_t * p_ble_iqo_c, const ble_evt_t * p_ble_evt)
{
    tx_buffer_process();
}

static void on_hvx(ble_iqo_c_t * p_ble_iqo_c, const ble_evt_t * p_ble_evt)
{
    // Check if this is temperature notify or acc notify.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_iqo_c->iqo_c_temp_handle) {
        ble_iqo_c_evt_t ble_iqo_c_evt;
        uint8_t *ptr = NULL;

        ble_iqo_c_evt.evt_type = BLE_IQO_C_EVT_IT_NOTIFY;
        ptr = (uint8_t *)&(p_ble_evt->evt.gattc_evt.params.hvx.data[1]);
        ble_iqo_c_evt.params.temp.temp_data = ((uint32_t)ptr[3] << 24) | ((uint32_t)ptr[2] << 16) | ((uint32_t)ptr[1] << 8) | ((uint32_t)ptr[0]);
        p_ble_iqo_c->evt_handler(p_ble_iqo_c, &ble_iqo_c_evt);
    } else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_iqo_c->iqo_c_acc_handle) {
        ble_iqo_c_evt_t ble_iqo_c_evt;

        ble_iqo_c_evt.evt_type = BLE_IQO_C_EVT_ACC_NOTIFY;
        iqo_c_status = 3;
        p_ble_iqo_c->evt_handler(p_ble_iqo_c, &ble_iqo_c_evt);
    }
}

static void db_discover_evt_handler(ble_db_discovery_evt_t * p_evt)
{
    static ble_iqo_c_t * p_ble_iqo_c;

    // Check if TEMP was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_HEALTH_THERMOMETER_SERVICE &&
        p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_BLE) {
        p_ble_iqo_c = peripheral_get_iqo_c(p_evt->conn_handle);

        if (p_ble_iqo_c != NULL) {
            for (uint16_t i = 0; i < p_evt->params.discovered_db.char_count; i++) {
                if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == BLE_UUID_INTERMEDIATE_TEMPERATURE_CHAR) {
                    p_ble_iqo_c->iqo_c_temp_cccd_handle = p_evt->params.discovered_db.charateristics[i].cccd_handle;
                    p_ble_iqo_c->iqo_c_temp_handle = p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                    break;
                }
            }

            ble_iqo_c_evt_t evt;
            evt.evt_type = BLE_IQO_C_EVT_DISCOVERY_TEMP_COMPLETE;
            p_ble_iqo_c->evt_handler(p_ble_iqo_c, &evt);
        }
    } else if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == 0x1110 /* ACC service */ &&
        p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_BLE) {
        p_ble_iqo_c = peripheral_get_iqo_c(p_evt->conn_handle);

        if (p_ble_iqo_c != NULL) {
            for (uint16_t i = 0; i < p_evt->params.discovered_db.char_count; i++) {
                if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == 0x1212 /* ACC notify */) {
                    p_ble_iqo_c->iqo_c_acc_cccd_handle = p_evt->params.discovered_db.charateristics[i].cccd_handle;
                    p_ble_iqo_c->iqo_c_acc_handle = p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                    break;
                }
            }

            ble_iqo_c_evt_t evt;
            evt.evt_type = BLE_IQO_C_EVT_DISCOVERY_ACC_COMPLETE;
            p_ble_iqo_c->evt_handler(p_ble_iqo_c, &evt);
            iqo_c_status = 1;
        }
    } else if (p_evt->evt_type == BLE_DB_DISCOVERY_ERROR) {

    } else if (p_evt->evt_type == BLE_DB_DISCOVERY_SRV_NOT_FOUND) {
    }
}

uint32_t ble_iqo_c_init(uint16_t conn_handle, ble_iqo_c_t * p_ble_iqo_c, ble_iqo_c_init_t * p_ble_iqo_c_init)
{
    if ((p_ble_iqo_c == NULL) || (p_ble_iqo_c_init == NULL)) {
        return NRF_ERROR_NULL;
    }

    p_ble_iqo_c->evt_handler = p_ble_iqo_c_init->evt_handler;
    p_ble_iqo_c->conn_handle = conn_handle;
    p_ble_iqo_c->iqo_c_temp_cccd_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_iqo_c->iqo_c_temp_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_iqo_c->iqo_c_acc_cccd_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_iqo_c->iqo_c_acc_handle = BLE_GATT_HANDLE_INVALID;

    return NRF_SUCCESS;
}

void ble_iqo_c_on_ble_evt(ble_iqo_c_t * p_ble_iqo_c, const ble_evt_t * p_ble_evt)
{
    if ((p_ble_iqo_c == NULL) || (p_ble_evt == NULL)) {
        return;
    }

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
        	// conn_handle has been configure at init
            break;

        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_iqo_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_iqo_c, p_ble_evt);
            break;

        default:
            break;
    }
}

static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable)
{
    tx_message_t * p_msg;
    uint16_t cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    p_msg = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = WRITE_MESSAGE_LENGTH;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}

uint32_t ble_iqo_c_temp_enable(ble_iqo_c_t * p_ble_iqo_c)
{
    if (p_ble_iqo_c == NULL) {
        return NRF_ERROR_NULL;
    }

    return cccd_configure(p_ble_iqo_c->conn_handle, p_ble_iqo_c->iqo_c_temp_cccd_handle, true);
}

uint32_t ble_iqo_c_acc_enable(ble_iqo_c_t * p_ble_iqo_c)
{
    if (p_ble_iqo_c == NULL) {
        return NRF_ERROR_NULL;
    }

    iqo_c_status = 2;

    return cccd_configure(p_ble_iqo_c->conn_handle, p_ble_iqo_c->iqo_c_acc_cccd_handle, true);
}

// Setup DB discovery
uint32_t ble_iqo_c_setup(void)
{
    uint32_t ret;
    ble_uuid_t uuid;

    uuid.type = BLE_UUID_TYPE_BLE;
    uuid.uuid = BLE_UUID_HEALTH_THERMOMETER_SERVICE;

    ret = ble_db_discovery_evt_register(&uuid, db_discover_evt_handler);
    if (ret != NRF_SUCCESS) {
        return ret;
    }

    uuid.type = BLE_UUID_TYPE_BLE;
    uuid.uuid = 0x1110;             // ACC service

    return ble_db_discovery_evt_register(&uuid, db_discover_evt_handler);
}

uint8_t ble_iqo_c_status(void)
{
	return iqo_c_status;
}

