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

#ifndef APP_PTU_TASK_H
#define APP_PTU_TASK_H

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "ke_task.h"         // kernel task
#include "ke_msg.h"          // kernel message#include "gapm_task.h"
#include "gapc_task.h"
#include "wptc_task.h"
#include "app_ptu.h"
#include <stdint.h>          // standard integer

#define APP_PTU_IDX_MAX (1)

/*
 * DEFINES
 ****************************************************************************************
 */

/// WTP Client enable command structure
struct ptu_read_pru_static_param_req
{
    /// Connection handle
    uint8_t client_idx;
};

/// States for PTU APP TASK
enum {
    APP_PTU_CONFIGURATION_STATE,
    APP_PTU_POWER_SAVE_STATE,
    APP_PTU_LOW_POWER_STATE,
    APP_PTU_POWER_TRANSFER_STATE,
    APP_PTU_LATCHING_FAULT_STATE,
    APP_PTU_LOCAL_FAULT_STATE,
    APP_PTU_STATE_MAX
};

enum {
    APP_PTU_SUB_POWER_STATE_1, // All PRUs are in Optimum Voltage Sub-state
    APP_PTU_SUB_POWER_STATE_2, // One or more PRUs are in Low Voltage Sub-state.
    APP_PTU_SUB_POWER_STATE_3, // One or more PRUs are in High Voltage Sub-state.
};

/// List of connected pru devices state on app side
enum {
    DEV_PRU_DISCONNECTED,
    /// IDLE state
    DEV_PRU_IDLE,
    /// State for reading PRU Static Value
    DEV_PRU_PRUST_RD,
    /// State for writing PTU Static Value
    DEV_PRU_PTUST_WR,
    /// State for reading PRU Dynamic Value first time
    DEV_PRU_DYN_RD1,
    /// ADJUST POWER State for changing charging parameters during registration (eg. shared mode set to 66.6% or 33.3% of Pmax)
    DEV_PRU_PWDADJ_NEW_REG_WR,
    /// ADJUST POWER State for reading PRU Dynamic Value to confirm, that PRU is fine with low power adjustment
    DEV_PRU_PWADJ_NEW_REG_DYN_RD_CFM,
    /// ADJUST POWER State for writing PRU Control value to indicate that we are waiting for power adjust from other PRU's
    DEV_PRU_PWADJ_PRU_CTRL_WAIT_FOR_POWER_PERM_WR,
    /// ADJUST POWER Dummy state for waiting for other PRU devices to finish their power adjusting
    DEV_PRU_PWADJ_WAIT_FOR_OTHER_PRU_POW_ADJUST,
    /// State for writing PRU Control value to start charging
    DEV_PRU_CTRL_START_CHARGING,
    /// State for registering for PRU Alert notifications
    DEV_PRU_ALERT_NOTIF_REGISTER,
    /// State for periodic reading of PRU Dynamic parameter
    DEV_PRU_DYN_READ_CHARGING,
    /// ADJUST POWER State for changing charging parameters during power transfer (eg. shared mode set to 66.6% or 33.3% of Pmax)
    DEV_PRU_PWADJ_IN_POWER_TRANSFER_PRU_CTRL_WR,
    /// ADJUST POWER State for confirming power adjust command via Dynamic Read Operation
    DEV_PRU_PWADJ_IN_POWER_TRANSFERR_DYN_RD_CFM,
    /// State for stopping charging process
    DEV_PRU_CTRL_STOP_CHARGING,
    /// State for rejecting newly connected PRU eg. because of limited charging power
    DEV_PRU_CTRL_ERR_REJECT_PRU_CHARGING,
    /// State for used to notify that device is now disconnecting (eg. because of some error)
    DEV_PRU_DISCONNECTING
};

//Discovery event messages for integrated app
enum {
    APP_PTU_DISC_CORRECT_PRU_DISCOVERED = 0,
    APP_PTU_DISC_ERR_NO_GAP_LIMITED_DISC,
    APP_PTU_DISC_ERR_BAD_WPTS_PAYLOAD,
    APP_PTU_DISC_ERR_RSSI_WEAK,
    APP_PTU_DISC_ERR_ALREADY_CONNECTED,
    APP_PTU_DISC_ERR_BAD_PTU_STATE,
    APP_PTU_DISC_TOO_MANY_CONN,
};


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */
//extern struct app_ptu_env_tag app_ptu_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

int ptu_enable_cfm_handler(ke_msg_id_t msgid, struct wptc_enable_cfm *param,
        ke_task_id_t dest_id, ke_task_id_t src_id);

int ptu_rd_char_rsp_handler(ke_msg_id_t msgid, struct wptc_rd_char_rsp *param,
        ke_task_id_t dest_id, ke_task_id_t src_id);

int ptu_wr_char_rsp_handler(ke_msg_id_t msgid, struct wptc_wr_char_rsp *param,
        ke_task_id_t dest_id, ke_task_id_t src_id);

int handle_alert_ntf_handler(ke_msg_id_t msgid,
        struct wptc_alert_report_ntf *param, ke_task_id_t dest_id,
        ke_task_id_t src_id);

int app_ptu_timer_dynamic_param_0_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

int app_ptu_timer_dynamic_param_1_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

int app_ptu_timer_dynamic_param_2_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

int app_ptu_timer_dynamic_param_3_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

int app_ptu_timer_registration_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

int app_ptu_timer_ptxin_expired_handler(ke_msg_id_t const msgid,
                                        void const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id);

int app_ptu_timer_power_adjust_expire_handler(ke_msg_id_t const msgid,
                                        void const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id);

int app_ptu_timer_power_adjust_rollback_expire_handler(ke_msg_id_t const msgid,
                                                        void const *param,
                                                        ke_task_id_t const dest_id,
                                                        ke_task_id_t const src_id);

int app_ptu_timer_self_latching_clear_handler(ke_msg_id_t const msgid,
                                        void const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id);

int app_ptu_timer_rssi_weak_adv_wait_handler(ke_msg_id_t const msgid,
                                        void const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id);

int app_ptu_mode_transition_empty_timer_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

int app_ptu_mode_transition_addr_timer_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

bool app_ptu_is_enough_power_for_all_devices(void);

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern const struct ke_state_handler app_ptu_default_handler;
extern ke_state_t app_ptu_state[APP_PTU_IDX_MAX];

/// @} APPTASK

#endif // APP_TASK_H_
