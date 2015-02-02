#include <stdint.h>
#include "ble_hts_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_srv_common.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "ble_gattc.h"
#include "app_util.h"
#include "app_trace.h"
#include "app_error.h"

//extern void uart_logf(const char *fmt, ...);

//#define LOG(F, ...)             (uart_logf(F "\r\n", ##__VA_ARGS__))
#define TX_BUFFER_MASK          0x07        /**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE          (TX_BUFFER_MASK + 1)  /**< Size of send buffer, which is 1 higher than the mask. */

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */
#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */

typedef enum
{
    READ_REQ,  /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ  /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

typedef struct
{
    uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< GATTC parameters for this message. */
} write_params_t;

/* Structure for holding data to be transmitted to the connected peer */
typedef struct
{
    uint16_t     conn_handle;  /* Connection handle to be used when transmitting this message. */
    tx_request_t type;         /* Type of this message, i.e. read or write message. */
    union
    {
        uint16_t       read_handle;  /* Read request message. */
        write_params_t write_req;    /* Write request message. */
    } req;
} tx_message_t;


static tx_message_t  m_tx_buffer[TX_BUFFER_SIZE];  /* Transmit buffer for messages to be transmitted to the central. */
static uint32_t      m_tx_insert_index = 0;        /* Current index in the transmit buffer where the next message should be inserted. */
static uint32_t      m_tx_index = 0;               /* Current index in the transmit buffer from where the next message to be transmitted resides. */

extern ble_hts_c_t *peripheral_get_hts_c(uint16_t conn_handle);

/* Function for passing any pending request from the buffer to the stack. */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS)
        {
            //LOG("[HTS_C]: SD Read/Write API returns Success..\r\n");
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
        else
        {
            //LOG("[HTS_C]: SD Read/Write API returns error. This message sending will be "
            //    "attempted again..\r\n");
        }
    }
}

static void on_write_rsp(ble_hts_c_t * p_ble_hts_c, const ble_evt_t * p_ble_evt)
{
    // Check if there is any message to be sent across to the peer and send it.
    tx_buffer_process();
}

static void on_hvx(ble_hts_c_t * p_ble_hts_c, const ble_evt_t * p_ble_evt)
{
    // Check if this is a TM indication.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_hts_c->tm_handle)
    {
        ble_hts_c_evt_t ble_hts_c_evt;
        uint8_t *ptr = NULL;

        ble_hts_c_evt.evt_type = BLE_HTS_C_EVT_TM_INDICATION;
        ptr = (uint8_t *)&(p_ble_evt->evt.gattc_evt.params.hvx.data[1]);
        ble_hts_c_evt.params.temp.tm_value = ((uint32_t)ptr[3] << 24) | ((uint32_t)ptr[2] << 16) | ((uint32_t)ptr[1] << 8) | ((uint32_t)ptr[0]);
        p_ble_hts_c->evt_handler(p_ble_hts_c, &ble_hts_c_evt);

        APP_ERROR_CHECK(sd_ble_gattc_hv_confirm(p_ble_evt->evt.gattc_evt.conn_handle, p_ble_evt->evt.gattc_evt.params.hvx.handle));
    }
}

static void db_discover_evt_handler(ble_db_discovery_evt_t * p_evt)
{
    static ble_hts_c_t * p_ble_hts_c; 

    // Check if HTS was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_HEALTH_THERMOMETER_SERVICE &&
        p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_BLE)
    {
        p_ble_hts_c = peripheral_get_hts_c(p_evt->conn_handle);

        if (p_ble_hts_c != NULL) {
            // Find the CCCD Handle of the HTS TM characteristic.
            uint32_t i;

            for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
            {
                if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                        BLE_UUID_TEMPERATURE_MEASUREMENT_CHAR)
                {
                    //LOG("[HTS_C]: Health Therometer Service discovered at peer.\r\n");
                    p_ble_hts_c->hts_cccd_handle =
                        p_evt->params.discovered_db.charateristics[i].cccd_handle;
                    p_ble_hts_c->tm_handle      =
                        p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                    break;
                }
            }

            ble_hts_c_evt_t evt;

            evt.evt_type = BLE_HTS_C_EVT_DISCOVERY_COMPLETE;

            p_ble_hts_c->evt_handler(p_ble_hts_c, &evt);
        }
    }
}


uint32_t ble_hts_c_init(uint16_t conn_handle, ble_hts_c_t * p_ble_hts_c, ble_hts_c_init_t * p_ble_hts_c_init)
{
    if ((p_ble_hts_c == NULL) || (p_ble_hts_c_init == NULL))
    {
        return NRF_ERROR_NULL;
    }

    ble_uuid_t hts_uuid;

    hts_uuid.type = BLE_UUID_TYPE_BLE;
    hts_uuid.uuid = BLE_UUID_HEALTH_THERMOMETER_SERVICE;

    p_ble_hts_c->evt_handler     = p_ble_hts_c_init->evt_handler;
    p_ble_hts_c->conn_handle     = conn_handle;
    p_ble_hts_c->hts_cccd_handle = BLE_GATT_HANDLE_INVALID;

    return ble_db_discovery_evt_register(&hts_uuid,
                                         db_discover_evt_handler);
}


void ble_hts_c_on_ble_evt(ble_hts_c_t * p_ble_hts_c, const ble_evt_t * p_ble_evt)
{
    if ((p_ble_hts_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	// conn_handle has been configure at init
            break;

        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_hts_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_hts_c, p_ble_evt);
            break;

        default:
            break;
    }
}


/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable)
{
    //LOG("[HTS_C]: Configuring CCCD. CCCD Handle = %d, Connection Handle = %d\r\n",
    //    handle_cccd,conn_handle);

    tx_message_t * p_msg;
    uint16_t       cccd_val = enable ? BLE_GATT_HVX_INDICATION : 0;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
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


uint32_t ble_hts_c_tm_idct_enable(ble_hts_c_t * p_ble_hts_c)
{
    if (p_ble_hts_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return cccd_configure(p_ble_hts_c->conn_handle, p_ble_hts_c->hts_cccd_handle, true);
}
