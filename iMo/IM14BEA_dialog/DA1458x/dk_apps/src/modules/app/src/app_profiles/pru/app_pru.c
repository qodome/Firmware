/**
 ****************************************************************************************
 *
 * @ile app_ptu.c
 *
 * @brief A4WP Wireless Power Transfer PTU role application.
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

#include "uart.h"

#include "gap.h"
#include "gapc_task.h"
#include "gapm_task.h"

#include "disc.h"
#include "disc_task.h"
#include "smpc_task.h"
#include "app.h"
#include "app_task.h"

#if (BLE_APP_PRU)
#include "app_pru.h"
#include "app_pru_task.h"
#include "app_pru_hooks.h"
#include "wpts.h"
#include "wpts_task.h"

struct app_pru_env_tag app_pru_env __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

/**
 ****************************************************************************************
 * @brief Set central master mode.
 *
 * @return void.
 ****************************************************************************************
 */
void app_pru_set_mode(void)
{
/*
    struct gapm_set_dev_config_cmd *msg = KE_MSG_ALLOC(
            GAPM_SET_DEV_CONFIG_CMD ,TASK_GAPM, TASK_APP,
            gapm_set_dev_config_cmd );

    msg->operation = GAPM_SET_DEV_CONFIG;
    // Device Role
    msg->role = GAP_CENTRAL_MST;

    ke_msg_send((void *) msg);
*/
    return;
}

/**
 ****************************************************************************************
 * @brief Send enable request to WPTS profile task.
 *
 * @return void.
 ****************************************************************************************
 */
void app_pru_enable(uint16_t conhdl)
{
    // Allocate the message
    struct wpts_enable_req * req = KE_MSG_ALLOC(WPTS_ENABLE_REQ, TASK_WPTS, TASK_APP,
                                                 wpts_enable_req);
    
    req->conhdl             = conhdl;
    req->sec_lvl            = PERM(SVC, ENABLE);
    
    // Send the message
    ke_msg_send(req);
}

void app_pru_connect_complete(void)
{
#if (BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU)
    app_pru_ext_send_connected_ind();
#endif //BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU
}

void app_pru_disconnect_complete(struct gapc_disconnect_ind const *disc_ind_param)
{
#if (BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU)
    app_pru_ext_send_disconnected_ind();
#endif //BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU
}

// This is triggered when disconnection command initiated by PRU completes
void app_pru_gapc_disc_cmplt(uint8_t conn_idx, uint8_t status)
{
#if (BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU)
    app_pru_ext_disconnection_cfm(status);
#endif //BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU
}
/**
 ****************************************************************************************
 * @brief Application's main function.
 *
 ****************************************************************************************
 */
void app_pru_init(void)
{
    // Reset the environment
    //memset(&app_ptu_env, 0, sizeof(app_ptu_env));
}

void app_pru_create_db_send(void)
{
    // Add DIS in the database
    struct wpts_create_db_req *req = KE_MSG_ALLOC(WPTS_CREATE_DB_REQ,
                                                  TASK_WPTS, TASK_APP,
                                                  wpts_create_db_req);
    req->features = 0;
    // Send the message
    ke_msg_send(req);
}

/**
 * Called every time after device reset & configuration (this is done not only
 * after init but also when device scan has to be re-called after registration
 * problem (timeout, bad characteristic value during registration)
 */
void app_pru_configuration_finished(void)
{

}

// ----- INTEGRATED API ------
void app_pru_ext_dev_ready_ntf(void)
{
    struct app_pru_ext_dev_ready_ntf* ntf = KE_MSG_ALLOC(APP_PRU_EXT_DEV_READY_NTF,
                                                         TASK_GTL, TASK_APP,
                                                         app_pru_ext_dev_ready_ntf);
    
    memset(ntf->status, 0, 4);
    ke_msg_send(ntf);
}

void app_pru_ext_send_connected_ind(void)
{
    void* evt = ke_msg_alloc(APP_PRU_EXT_CONNECTED_NTF,
                             TASK_GTL, TASK_APP,
                             0);
    ke_msg_send(evt);
}

void app_pru_ext_send_disconnected_ind(void)
{
    void* evt = ke_msg_alloc(APP_PRU_EXT_DISCONNECTED_NTF,
                             TASK_GTL, TASK_APP,
                             0);
    ke_msg_send(evt);
}

void app_pru_ext_disconnection_cfm(uint8_t status)
{
    struct app_pru_ext_send_pru_disconnection_cfm* cfm = KE_MSG_ALLOC(APP_PRU_EXT_SEND_DISCONNECTION_CMF,
                                                                      TASK_GTL, TASK_APP,
                                                                      app_pru_ext_send_pru_disconnection_cfm);
    
    cfm->status = status;
    ke_msg_send(cfm);
}

#endif // BLE_APP_PRU
