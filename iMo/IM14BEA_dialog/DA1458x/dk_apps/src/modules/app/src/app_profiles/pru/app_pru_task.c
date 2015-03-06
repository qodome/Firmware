/**
 ****************************************************************************************
 *
 * @file app_task.c
 *
 * @brief A4WP WPT Power Transmit Unit (PTU) implementation file used for
 * handling of ble events and responses that coming from WPTC (Wireless Power
 * Transfer Client) module.
 *
 * Copyright (C) 2014. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */
 
#include "rwip_config.h"             // SW configuration

#include "gapm_task.h"
#include "gapc.h"
#include "gapc_task.h"

//#include "wpts_task.h" 
#include "wpts.h"

#include "disc.h"
#include "disc_task.h"
#include "smpc_task.h" 

/* for basic APP API, eg. connecting, scanning...*/
#include "app.h"
#include "app_task.h"

#if (BLE_APP_PRU)
#include "app_pru.h"
#include "app_pru_task.h"
#include "app_pru_hooks.h"



/**
 ****************************************************************************************
 * @brief Handles Proximity Monitor profile enable confirmation.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int pru_enable_cfm_handler(ke_msg_id_t msgid, void *param,
        ke_task_id_t dest_id, ke_task_id_t src_id)
{
    return (KE_MSG_CONSUMED);
}

int pru_create_db_cfm_handler(ke_msg_id_t const msgid,
                              struct wpts_create_db_cfm const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    // If state is not idle, ignore the message
    if (ke_state_get(dest_id) == APP_DB_INIT)
    {
         
        // Inform the Application Manager
        struct app_module_init_cmp_evt *cfm = KE_MSG_ALLOC(APP_MODULE_INIT_CMP_EVT,
                                                           TASK_APP, TASK_APP,
                                                           app_module_init_cmp_evt);

        cfm->status = param->status;

        ke_msg_send(cfm);
    }

    return (KE_MSG_CONSUMED);
}

int pru_disable_ind_handler(ke_msg_id_t const msgid,
                            struct wpts_disable_ind const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{    
    return (KE_MSG_CONSUMED);
}

int pru_alert_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                     struct wpts_alert_cfg_indntf_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

int pru_ctrl_change_ind_handler(ke_msg_id_t const msgid,
                                struct wpts_ctrl_change_ind const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
#if (BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU)
    struct app_pru_ext_pru_control_ntf *ntf = KE_MSG_ALLOC(APP_PRU_EXT_PRU_CONTROL_NTF,
                                                           TASK_GTL, TASK_APP,
                                                           app_pru_ext_pru_control_ntf);
    ntf->enables    = param->enables;
    ntf->permission = param->permission;
    ntf->time_set   = param->time_set;
    ntf->rfu_1      = param->rfu_1;
        
    ke_msg_send(ntf);
#endif // BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU

    return (KE_MSG_CONSUMED);
}

int pru_ptu_static_val_ind_handler(ke_msg_id_t const msgid,
                                   struct wpts_ptu_static_val_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
#if (BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU)
    struct app_pru_ext_ptu_static_ntf *ntf = KE_MSG_ALLOC(APP_PRU_EXT_PTU_STATIC_NTF,
                                                          TASK_GTL, TASK_APP,
                                                          app_pru_ext_ptu_static_ntf);
    ntf->validity               = param->valid;
    ntf->power                  = param->power;
    ntf->max_src_impedance      = param->max_impedance;
    ntf->max_load_resistance    = param->max_resistance;
    ntf->rfu_1                  = param->rfu_1;
    ntf->ptu_class              = param->class;
    ntf->hr_ver                 = param->hw_ver;
    ntf->sw_ver                 = param->sw_ver;
    ntf->prtcl_ver              = param->prtcl_ver;
    ntf->max_dev                = param->max_dev;
    ntf->rfu_2                  = param->rfu_2;
    ntf->rfu_3                  = param->rfu_3;
    ntf->rfu_4                  = param->rfu_4;

    ke_msg_send(ntf);
#endif // BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU

    return (KE_MSG_CONSUMED);
}

int pru_alert_send_cfm_handler(ke_msg_id_t const msgid,
                               struct wpts_alert_send_cfm const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
#if (BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU)
    struct app_pru_ext_send_pru_alert_cfm *cfm = KE_MSG_ALLOC(APP_PRU_EXT_SEND_PRU_ALERT_CFM,
                                                              TASK_GTL, TASK_APP,
                                                              app_pru_ext_send_pru_alert_cfm);
    cfm->status = param->status;
    
    ke_msg_send(cfm);
#endif // BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU

    return (KE_MSG_CONSUMED);
}

int pru_static_val_cfm_handler(ke_msg_id_t const msgid,
                               struct wpts_pru_static_val_cfm const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

int pru_dynamic_val_cfm_handler(ke_msg_id_t const msgid,
                                struct wpts_pru_dynamic_val_cfm const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

// *** INTEGRATED API *********

#if BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU

int app_pru_ext_start_adv_ind_handler(ke_msg_id_t const msgid,
                                      uint8_t const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    
    // --- response ---
    struct app_pru_ext_msg_status_rsp* rsp = KE_MSG_ALLOC(APP_PRU_EXT_START_ADV_RSP,
                                                          TASK_GTL, TASK_APP,
                                                          app_pru_ext_msg_status_rsp);
    
    //if (APP_CONNECTED != state)
    {
        memcpy(&app_pru_env.ext_adv.interval, param, 2);
        memcpy(&app_pru_env.ext_adv.rssi, &param[2], 1);
        memcpy(&app_pru_env.ext_adv.adv_flags, &param[3], 1);
        if (app_pru_env.ext_adv.interval >= 0x20 &&  app_pru_env.ext_adv.interval <= 0x4000)
        {
            memcpy(app_pru_env.ext_adv.dev_name, &param[4], 16);
            // make sure string is ended by 0x00
            app_pru_env.ext_adv.dev_name[16] = 0x00;
            app_adv_start();
            
            rsp->status = 0x00;
        }
        else
        {
            rsp->status = PRF_APP_ERROR;
        }
    }
    //else
    //{
    //    rsp->status = PRF_APP_ERROR;
    //}
    
    ke_msg_send(rsp);
    
    return (KE_MSG_CONSUMED);
}

int app_pru_ext_stop_adv_ind_handler(ke_msg_id_t const msgid,
                                     void const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    app_adv_stop();
    
    // --- response ---
    struct app_pru_ext_msg_status_rsp* rsp = KE_MSG_ALLOC(APP_PRU_EXT_STOP_ADV_RSP,
                                                          TASK_GTL, TASK_APP,
                                                          app_pru_ext_msg_status_rsp);
    rsp->status = 0x00;
    ke_msg_send(rsp);
    
    return (KE_MSG_CONSUMED);
}

int app_pru_ext_set_pru_static_ind_handler(ke_msg_id_t const msgid,
                                           uint8_t const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    struct wpts_pru_static_val_req* req = KE_MSG_ALLOC(WPTS_PRU_STATIC_VAL_UPD_DB_REQ,
                                                       TASK_WPTS, TASK_APP,
                                                       wpts_pru_static_val_req);
    
    memcpy(req, param, WPTS_PRU_STATIC_CHAR_LEN);
    ke_msg_send(req);
    
    // --- response ---
    struct app_pru_ext_msg_status_rsp* rsp = KE_MSG_ALLOC(APP_PRU_EXT_SET_PRU_STATIC_RSP,
                                                          TASK_GTL, TASK_APP,
                                                          app_pru_ext_msg_status_rsp);
    
    rsp->status = 0x00;
    ke_msg_send(rsp);
    
    return (KE_MSG_CONSUMED);
}

int app_pru_ext_set_pru_dynamic_ind_handler(ke_msg_id_t const msgid,
                                            uint8_t const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
    struct wpts_pru_dynamic_val_req* req = KE_MSG_ALLOC(WPTS_PRU_DYNAMIC_VAL_UPD_DB_REQ,
                                                        TASK_WPTS, TASK_APP,
                                                        wpts_pru_dynamic_val_req);
 
    // has to be cast due to pragma pack
    uint8_t *data = (uint8_t*)req;
    uint16_t conhdl = gapc_get_conhdl(wpts_env.con_info.conidx);
    
    memcpy(&data[0], &conhdl, sizeof(conhdl));
    memcpy(&data[2], param, WPTS_PRU_DYNAMIC_CHAR_LEN);
    ke_msg_send(req);
    
    // --- response ---
    struct app_pru_ext_msg_status_rsp* rsp = KE_MSG_ALLOC(APP_PRU_EXT_SET_PRU_DYNAMIC_RSP,
                                                          TASK_GTL, TASK_APP,
                                                          app_pru_ext_msg_status_rsp);
    
    rsp->status = 0x00;
    ke_msg_send(rsp);
    
    return (KE_MSG_CONSUMED);
}

int app_pru_ext_send_pru_alert_req_handler(ke_msg_id_t const msgid,
                                           struct app_pru_ext_send_pru_alert_req const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    struct wpts_alert_send_req* req = KE_MSG_ALLOC(WPTS_ALERT_SEND_REQ,
                                                   TASK_WPTS, TASK_APP,
                                                   wpts_alert_send_req);

    req->conhdl = gapc_get_conhdl(wpts_env.con_info.conidx);
    req->is_ind = param->nft_ind;
    req->alert_val = param->alert;
    memcpy(req->dev_address, param->bt_address, 6);
    
    ke_msg_send(req);
    
    return (KE_MSG_CONSUMED);
}

int app_pru_ext_send_pru_disconnection_req_handler(ke_msg_id_t const msgid,
                                                   void const *param,
                                                   ke_task_id_t const dest_id,
                                                   ke_task_id_t const src_id)
{
    app_disconnect();
    return (KE_MSG_CONSUMED);
}

#endif //BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

#endif // BLE_APP_PRU
