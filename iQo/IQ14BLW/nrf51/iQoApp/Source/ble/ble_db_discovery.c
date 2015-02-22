/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

#include "ble_db_discovery.h"
#include <stdlib.h>
#include <string.h>
#include "nrf_error.h"
#include "ble.h"
#include "app_trace.h"
#include "app_error.h"
#include "app_timer.h"
#include "nordic_common.h"

#define SRV_DISC_START_HANDLE  0x0001
#define DB_DISCOVERY_MAX_USERS BLE_DB_DISCOVERY_MAX_SRV

static struct {
    ble_uuid_t srv_uuid;
    ble_db_discovery_evt_handler_t evt_handler;
} m_registered_handlers[DB_DISCOVERY_MAX_USERS];

static uint8_t m_num_of_handlers_reg;
static uint8_t m_discover_in_progress = 0;
uint8_t debug_cnt[16] = {0};

extern bool is_central(uint16_t conn_handle);
extern void disconnect_peer(void);

static void ble_db_handle_error(void)
{
    disconnect_peer();
}

static ble_db_discovery_evt_handler_t registered_handler_get(ble_uuid_t *p_srv_uuid)
{
    for (uint8_t i = 0; i < m_num_of_handlers_reg; i++) {
        if (BLE_UUID_EQ(&(m_registered_handlers[i].srv_uuid), p_srv_uuid)) {
            return (m_registered_handlers[i].evt_handler);
        }
    }
    return NULL;
}

static uint32_t registered_handler_set(const ble_uuid_t *const p_srv_uuid,
                                       ble_db_discovery_evt_handler_t p_evt_handler)
{
    if (m_num_of_handlers_reg < DB_DISCOVERY_MAX_USERS) {
        m_registered_handlers[m_num_of_handlers_reg].srv_uuid = *p_srv_uuid;
        m_registered_handlers[m_num_of_handlers_reg].evt_handler = p_evt_handler;
        m_num_of_handlers_reg++;
        return NRF_SUCCESS;
    } else {
        return NRF_ERROR_NO_MEM;
    }
}

static void discovery_complete_evt_trigger(ble_db_discovery_t *const p_db_discovery,
                                           bool is_srv_found)
{
    ble_db_discovery_evt_handler_t p_evt_handler;
    ble_db_discovery_srv_t *p_srv_being_discovered;
    ble_db_discovery_evt_t evt;

    for (uint8_t idx = 0; idx < p_db_discovery->srv_count; idx++) {
        p_srv_being_discovered = &(p_db_discovery->services[idx]);
        p_evt_handler = registered_handler_get(&(p_srv_being_discovered->srv_uuid));
        if (p_evt_handler != NULL) {
            evt.conn_handle = p_db_discovery->conn_handle;
            if (is_srv_found) {
                evt.evt_type = BLE_DB_DISCOVERY_COMPLETE;
                evt.params.discovered_db = *p_srv_being_discovered;
            } else {
                evt.evt_type = BLE_DB_DISCOVERY_SRV_NOT_FOUND;
            }
            p_evt_handler(&evt);
        }
    }
    m_discover_in_progress = 0;
}

static uint32_t characteristics_discover(ble_db_discovery_t *const p_db_discovery)
{
    ble_gattc_char_t *p_prev_char;
    ble_db_discovery_srv_t *p_srv_being_discovered;
    ble_gattc_handle_range_t handle_range;
    uint8_t prev_char_ind;

    if (p_db_discovery->curr_srv_ind < p_db_discovery->srv_count) {
        p_srv_being_discovered = &(p_db_discovery->services[p_db_discovery->curr_srv_ind]);

        if (p_db_discovery->curr_char_ind != 0) {
            prev_char_ind = p_db_discovery->curr_char_ind - 1;
            p_prev_char = &(p_srv_being_discovered->charateristics[prev_char_ind].characteristic);
            handle_range.start_handle = p_prev_char->handle_value + 1;
        } else {
            handle_range.start_handle = p_srv_being_discovered->handle_range.start_handle;
        }
        handle_range.end_handle = p_srv_being_discovered->handle_range.end_handle;
        return sd_ble_gattc_characteristics_discover(p_db_discovery->conn_handle, &handle_range);
    } else {
        return NRF_ERROR_INVALID_DATA;
    }
}

static bool is_desc_discovery_reqd(ble_db_discovery_t *p_db_discovery,
                                   ble_db_discovery_char_t *p_curr_char,
                                   ble_db_discovery_char_t *p_next_char,
                                   ble_gattc_handle_range_t *p_handle_range)
{
    if (p_next_char == NULL) {
        if (p_curr_char->characteristic.handle_value == p_db_discovery->services[p_db_discovery->curr_srv_ind].handle_range.end_handle) {
            return false;
        }
        p_handle_range->start_handle = p_curr_char->characteristic.handle_value + 1;
        p_handle_range->end_handle = p_db_discovery->services[p_db_discovery->curr_srv_ind].handle_range.end_handle;
        return true;
    }

    if ((p_curr_char->characteristic.handle_value + 1) == p_next_char->characteristic.handle_decl) {
        return false;
    }

    p_handle_range->start_handle = p_curr_char->characteristic.handle_value + 1;
    p_handle_range->end_handle   = p_next_char->characteristic.handle_decl - 1;

    return true;
}

static uint32_t descriptors_discover(ble_db_discovery_t *const p_db_discovery,
                                     bool *p_raise_discov_complete)
{
    ble_gattc_handle_range_t handle_range;
    ble_db_discovery_char_t *p_curr_char_being_discovered;
    ble_db_discovery_srv_t *p_srv_being_discovered;
    bool is_discovery_reqd = false;    
    ble_db_discovery_char_t *p_next_char;

    p_srv_being_discovered = &(p_db_discovery->services[p_db_discovery->curr_srv_ind]);
    p_curr_char_being_discovered = &(p_srv_being_discovered->charateristics[p_db_discovery->curr_char_ind]);
    if ((p_db_discovery->curr_char_ind + 1) == p_srv_being_discovered->char_count) {
        is_discovery_reqd = is_desc_discovery_reqd(p_db_discovery,
                                                   p_curr_char_being_discovered,
                                                   NULL,
                                                   &handle_range);
    } else {
        for (uint8_t i = p_db_discovery->curr_char_ind; i < p_srv_being_discovered->char_count; i++) {
            if (i == (p_srv_being_discovered->char_count - 1)) {
                p_next_char = NULL;
            } else {
                p_next_char = &(p_srv_being_discovered->charateristics[i + 1]);
            }

            if (is_desc_discovery_reqd(p_db_discovery,
                                       p_curr_char_being_discovered,
                                       p_next_char,
                                       &handle_range)) {
                is_discovery_reqd = true;
                break;
            } else {
                p_curr_char_being_discovered = p_next_char;
                p_db_discovery->curr_char_ind++;
            }
        }
    }

    if (!is_discovery_reqd) {
    	p_db_discovery->curr_srv_ind++;
    	if (p_db_discovery->curr_srv_ind < p_db_discovery->srv_count) {
    		p_db_discovery->curr_char_ind = 0;

    		return descriptors_discover(p_db_discovery, p_raise_discov_complete);
    	} else {
    	    *p_raise_discov_complete = true;
            return NRF_SUCCESS;
    	}
    }
    *p_raise_discov_complete = false;
    return sd_ble_gattc_descriptors_discover(p_db_discovery->conn_handle, &handle_range);
}

static int service_interested(const ble_gattc_service_t *prim_svc_p)
{
    for (uint8_t idx = 0; idx < m_num_of_handlers_reg; idx++) {
    	if (m_registered_handlers[idx].srv_uuid.uuid == prim_svc_p->uuid.uuid) {
    		return 1;
    	}
    }
    return 0;
}

static void on_primary_srv_discovery_rsp(ble_db_discovery_t * const    p_db_discovery,
                                         const ble_gattc_evt_t * const p_ble_gattc_evt)
{
    uint32_t err_code;
    const ble_gattc_evt_prim_srvc_disc_rsp_t *p_prim_srvc_disc_rsp_evt;

    if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS) {
        // Gather results
        p_prim_srvc_disc_rsp_evt = &(p_ble_gattc_evt->params.prim_srvc_disc_rsp);

        for (uint8_t idx = 0; idx < p_prim_srvc_disc_rsp_evt->count; idx++) {
            if (service_interested(&(p_prim_srvc_disc_rsp_evt->services[idx]))) {
                p_db_discovery->services[p_db_discovery->srv_count].srv_uuid = p_prim_srvc_disc_rsp_evt->services[idx].uuid;
                memcpy((uint8_t *)&(p_db_discovery->services[p_db_discovery->srv_count].handle_range),
                        (uint8_t *)&(p_prim_srvc_disc_rsp_evt->services[idx].handle_range),
                        sizeof(ble_gattc_handle_range_t));
                p_db_discovery->srv_count++;
            }
        }

        err_code = sd_ble_gattc_primary_services_discover(p_db_discovery->conn_handle,
                        (p_prim_srvc_disc_rsp_evt->services[p_prim_srvc_disc_rsp_evt->count - 1].handle_range.end_handle + 1), NULL);
        if (err_code != NRF_SUCCESS) {
            APP_ERROR_CHECK(err_code);
            ble_db_handle_error();
            return;
        }
    } else if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_ATTERR_INVALID_HANDLE) {
        err_code = characteristics_discover(p_db_discovery);
        if (err_code != NRF_SUCCESS) {
            APP_ERROR_CHECK(err_code);
            ble_db_handle_error();
            return;
        }
    } else {
        ble_db_handle_error();
    }
}

static void on_characteristic_discovery_rsp(ble_db_discovery_t *const p_db_discovery,
                                            const ble_gattc_evt_t *const p_ble_gattc_evt)
{
    uint32_t err_code;
    ble_db_discovery_srv_t *p_srv_being_discovered;
    const ble_gattc_evt_char_disc_rsp_t *p_char_disc_rsp_evt;
    bool perform_desc_discov = false;    
    bool raise_discov_complete;

    p_srv_being_discovered = &(p_db_discovery->services[p_db_discovery->curr_srv_ind]);
    if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS) {
        p_char_disc_rsp_evt = &(p_ble_gattc_evt->params.char_disc_rsp);
        for (uint8_t i = 0; i < p_char_disc_rsp_evt->count; i++) {
            memcpy((uint8_t *)&(p_srv_being_discovered->charateristics[p_srv_being_discovered->char_count + i].characteristic),
                    (uint8_t *)&(p_char_disc_rsp_evt->chars[i]), sizeof(ble_gattc_char_t));
            p_srv_being_discovered->charateristics[p_srv_being_discovered->char_count + i].cccd_handle = BLE_GATT_HANDLE_INVALID;
        }
        p_srv_being_discovered->char_count += p_char_disc_rsp_evt->count;

        if (p_srv_being_discovered->charateristics[p_srv_being_discovered->char_count - 1].characteristic.handle_value >=
                p_db_discovery->services[p_db_discovery->curr_srv_ind].handle_range.end_handle) {
            p_db_discovery->curr_srv_ind++;
            if (p_db_discovery->curr_srv_ind < p_db_discovery->srv_count) {
                p_db_discovery->curr_char_ind = 0;

                err_code = characteristics_discover(p_db_discovery);
                if (err_code != NRF_SUCCESS) {
                    APP_ERROR_CHECK(err_code);
                    ble_db_handle_error();
                    return;
                }
            } else {
                perform_desc_discov = true;
            }
        } else {
            p_db_discovery->curr_char_ind = p_srv_being_discovered->char_count;
            err_code = characteristics_discover(p_db_discovery);
            if (err_code != NRF_SUCCESS) {
                APP_ERROR_CHECK(err_code);
                ble_db_handle_error();
                return;
            }
        }
    } else if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND &&
    		p_db_discovery->curr_srv_ind == 1) {
    	p_db_discovery->curr_srv_ind = 0;
    	perform_desc_discov = true;
    } else {
        ble_db_handle_error();
        return;
    }

    if (perform_desc_discov) {
        p_db_discovery->curr_char_ind = 0;
        err_code = descriptors_discover(p_db_discovery, &raise_discov_complete);
        if (err_code != NRF_SUCCESS) {
            APP_ERROR_CHECK(err_code);
            ble_db_handle_error();
            return;
        }
        if (raise_discov_complete) {
            discovery_complete_evt_trigger(p_db_discovery, true);
        }
    }
}

static void on_descriptor_discovery_rsp(ble_db_discovery_t *const p_db_discovery,
                                        const ble_gattc_evt_t *const p_ble_gattc_evt)
{
    const ble_gattc_evt_desc_disc_rsp_t *p_desc_disc_rsp_evt;
    ble_db_discovery_srv_t *p_srv_being_discovered;
    ble_db_discovery_char_t *p_char_being_discovered;
    bool raise_discov_complete = false;
    uint32_t err_code;

    p_desc_disc_rsp_evt = &(p_ble_gattc_evt->params.desc_disc_rsp);
    p_srv_being_discovered = &(p_db_discovery->services[p_db_discovery->curr_srv_ind]);
    p_char_being_discovered = &(p_srv_being_discovered->charateristics[p_db_discovery->curr_char_ind]);

    if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS) {
        for (uint8_t i = 0; i < p_desc_disc_rsp_evt->count; i++) {
            if (p_desc_disc_rsp_evt->descs[i].uuid.uuid == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG) {
                p_char_being_discovered->cccd_handle = p_desc_disc_rsp_evt->descs[i].handle;
                break;
            }
        }
    } else {
        ble_db_handle_error();
        return;
    }

    if ((p_db_discovery->curr_char_ind + 1) == p_srv_being_discovered->char_count) {
        raise_discov_complete = true;
    } else {
        p_db_discovery->curr_char_ind++;
        err_code = descriptors_discover(p_db_discovery, &raise_discov_complete);
        if (err_code != NRF_SUCCESS) {
            APP_ERROR_CHECK(err_code);
            ble_db_handle_error();
            return;
        }
    }

    if (raise_discov_complete) {
        discovery_complete_evt_trigger(p_db_discovery, true);
    }
}

uint32_t ble_db_discovery_init(void)
{
    m_num_of_handlers_reg = 0;
    return NRF_SUCCESS;
}

uint32_t ble_db_discovery_evt_register(const ble_uuid_t *const p_uuid,
                                       const ble_db_discovery_evt_handler_t evt_handler)
{
    if ((p_uuid == NULL) || (evt_handler == NULL)) {
        return NRF_ERROR_NULL;
    }
    if (m_num_of_handlers_reg == DB_DISCOVERY_MAX_USERS) {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    return registered_handler_set(p_uuid, evt_handler);
}

uint32_t ble_db_discovery_start(ble_db_discovery_t * const p_db_discovery,
                                uint16_t                   conn_handle)
{
    uint32_t err_code;

    if (p_db_discovery == NULL) {
        return NRF_ERROR_NULL;
    }

    m_discover_in_progress = 1;
    memset((uint8_t *)p_db_discovery, 0, sizeof(ble_db_discovery_t));
    p_db_discovery->conn_handle  = conn_handle;

    err_code = sd_ble_gattc_primary_services_discover(p_db_discovery->conn_handle,
                                                      SRV_DISC_START_HANDLE, NULL);
    if (err_code != NRF_SUCCESS) {
        ble_db_handle_error();
        return err_code;
    }

    return NRF_SUCCESS;
}

void ble_db_discovery_on_ble_evt(ble_db_discovery_t * const p_db_discovery,
                                 const ble_evt_t * const    p_ble_evt)
{
    if ((p_db_discovery == NULL) || (p_ble_evt == NULL)) {
        return;
    }

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            if (p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_CENTRAL) {
                p_db_discovery->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                APP_ERROR_CHECK(ble_db_discovery_start(p_db_discovery, p_ble_evt->evt.gap_evt.conn_handle));
            }
            break;
        
        case BLE_GAP_EVT_DISCONNECTED:
            if (!is_central(p_ble_evt->evt.gap_evt.conn_handle)) {
                memset(p_db_discovery, 0, sizeof(ble_db_discovery_t));
                p_db_discovery->conn_handle = BLE_CONN_HANDLE_INVALID;
            }
            break;

        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
            on_primary_srv_discovery_rsp(p_db_discovery, &(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GATTC_EVT_CHAR_DISC_RSP:
            on_characteristic_discovery_rsp(p_db_discovery, &(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GATTC_EVT_DESC_DISC_RSP:
            on_descriptor_discovery_rsp(p_db_discovery, &(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            if (m_discover_in_progress != 0) {
                ble_db_handle_error();
                debug_cnt[2]++;
            }
        	break;

        default:
            break;
    }
}
