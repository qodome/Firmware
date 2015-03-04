/**
 ****************************************************************************************
 *
 * @file app_ptu_task.h
 *
 * @brief A4WP WPT Power Transmit Unit (PTU) header file used for
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

#ifndef APP_PRU_TASK_H
#define APP_PRU_TASK_H

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "ke_task.h"         // kernel task
#include "ke_msg.h"          // kernel message
#include "wpts_task.h"
//#include "gapm_task.h"
//#include "gapc_task.h"
#include "app_pru.h"
#include <stdint.h>          // standard integer

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */
//extern struct app_ptu_env_tag app_ptu_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

int pru_enable_cfm_handler(ke_msg_id_t msgid, void *param,
                           ke_task_id_t dest_id, ke_task_id_t src_id);

int pru_create_db_cfm_handler(ke_msg_id_t const msgid,
                              struct wpts_create_db_cfm const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

int pru_disable_ind_handler(ke_msg_id_t const msgid,
                            struct wpts_disable_ind const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id);

int pru_alert_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                     struct wpts_alert_cfg_indntf_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id);

int pru_ctrl_change_ind_handler(ke_msg_id_t const msgid,
                                struct wpts_ctrl_change_ind const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

int pru_ptu_static_val_ind_handler(ke_msg_id_t const msgid,
                                   struct wpts_ptu_static_val_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
                                   
int pru_alert_send_cfm_handler(ke_msg_id_t const msgid,
                               struct wpts_alert_send_cfm const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);
                               
int pru_static_val_cfm_handler(ke_msg_id_t const msgid,
                               struct wpts_pru_static_val_cfm const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

int pru_dynamic_val_cfm_handler(ke_msg_id_t const msgid,
                                struct wpts_pru_dynamic_val_cfm const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);
                               
// Integrated host GTL for PRU APP

                                /*
 * API MESSAGES STRUCTURES
 ****************************************************************************************
 */

#if BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU
#pragma pack(push)
#pragma pack(1)
struct app_pru_ext_ptu_static_ntf
{
    uint8_t validity;
    uint8_t power;
    uint8_t max_src_impedance;
    uint8_t max_load_resistance;
    uint16_t rfu_1;
    uint8_t ptu_class;
    uint8_t hr_ver;
    uint8_t sw_ver;
    uint8_t prtcl_ver;
    uint8_t max_dev;
    uint16_t rfu_2;
    uint16_t rfu_3;
    uint16_t rfu_4;
};
struct app_pru_ext_pru_control_ntf
{
    uint8_t enables;
    uint8_t permission;
    uint8_t time_set;
    uint16_t rfu_1;
};
struct app_pru_ext_dev_ready_ntf
{
    uint8_t status[4];
};
struct app_pru_ext_start_adv_ind
{
    uint16_t interval;
    uint8_t rssi;
    uint8_t adv_flags;
    uint8_t dev_name[16];
};
struct app_pru_ext_send_pru_alert_req
{
    uint8_t nft_ind;
    uint8_t alert;
    uint8_t bt_address[6];
};
struct app_pru_ext_send_pru_alert_cfm
{
    uint8_t status;
};

struct app_pru_ext_send_pru_disconnection_cfm
{
    uint8_t status;
};

struct app_pru_ext_msg_status_rsp
{
    uint8_t status;
};
#pragma pack(pop)
                                
int app_pru_ext_start_adv_ind_handler(ke_msg_id_t const msgid,
                                      uint8_t const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

int app_pru_ext_stop_adv_ind_handler(ke_msg_id_t const msgid,
                                     void const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id);

int app_pru_ext_set_pru_static_ind_handler(ke_msg_id_t const msgid,
                                           uint8_t const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id);
                                     
int app_pru_ext_set_pru_dynamic_ind_handler(ke_msg_id_t const msgid,
                                            uint8_t const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id);
                                            
int app_pru_ext_send_pru_alert_req_handler(ke_msg_id_t const msgid,
                                           struct app_pru_ext_send_pru_alert_req const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id);

int app_pru_ext_send_pru_disconnection_req_handler(ke_msg_id_t const msgid,
                                                   void const *param,
                                                   ke_task_id_t const dest_id,
                                                   ke_task_id_t const src_id);                                           


#endif // BLE_INTEGRATED_HOST_GTL && BLE_APP_PRU
                                           
/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

//extern const struct ke_state_handler app_ptu_default_handler;
//extern ke_state_t app_ptu_state[APP_PTU_IDX_MAX];

/// @} APPTASK

#endif // APP_PRU_TASK_H
