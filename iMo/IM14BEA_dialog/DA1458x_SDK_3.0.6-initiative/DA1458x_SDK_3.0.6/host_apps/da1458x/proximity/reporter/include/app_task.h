/**
 ****************************************************************************************
 *
 * @file app_task.h
 *
 * @brief Header file - APPTASK.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: 5767 $
 *
 ****************************************************************************************
 */

#ifndef APP_TASK_H_
#define APP_TASK_H_


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "ke_task.h"         // kernel task
#include "ke_msg.h"          // kernel message
#include <stdint.h>          // standard integer

/*
 * DEFINES
 ****************************************************************************************
 */
/// number of APP Process
#define APP_IDX_MAX                                 0x01

/// states of APP task
/*Wei
enum
{
    /// Idle state
    APP_IDLE,
    /// Scanning state
    APP_SCAN,
    /// Connected state
    APP_CONNECTED,
    /// Number of defined states.
    APP_STATE_MAX
};*/

enum
{
    /// Idle state
    APP_IDLE,
    /// Scanning state
    APP_CONNECTABLE,
    /// Connected state
    APP_CONNECTED,
    /// Number of defined states.
    APP_STATE_MAX,
	/// Scanning state
    APP_SCAN,
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */
extern struct app_env_tag app_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

// int gap_ready_evt_handler(ke_msg_id_t msgid,
//                                struct gap_ready_evt *param,
//                                ke_task_id_t dest_id,
//                                ke_task_id_t src_id);

// int gap_dev_inq_req_cmp_evt_handler(ke_msg_id_t msgid,
//                                       struct gap_dev_inq_req_cmp_evt *param,
//                                       ke_task_id_t dest_id,
//                                       ke_task_id_t src_id);

// int gap_dev_inq_result_handler(ke_msg_id_t msgid,
//                                       struct gapm_adv_report_ind *param,
//                                       ke_task_id_t dest_id,
//                                       ke_task_id_t src_id);

// int gap_le_create_conn_req_ind_handler(ke_msg_id_t msgid,
// 											struct gapc_connection_req_ind *param,
// 											ke_task_id_t dest_id,
// 											ke_task_id_t src_id);

// int gap_read_rssi_cmp_evt_handler(ke_msg_id_t msgid,
//                                   struct gapc_con_rssi_ind   *param,
//                                   ke_task_id_t dest_id,
//                                   ke_task_id_t src_id);
// // int gap_bond_req_ind_handler(ke_msg_id_t msgid,
// //                                     struct gapc_bond_req_ind *param,
// //                                     ke_task_id_t dest_id,
// //                                     ke_task_id_t src_id);
// int gap_bond_ind_handler(ke_msg_id_t msgid,
//                                     struct gapc_bond_ind *param,
//                                     ke_task_id_t dest_id,
//                                     ke_task_id_t src_id);
// int gap_discon_cmp_evt_handler(ke_msg_id_t msgid,
//                                   struct gapc_disconnect_ind *param,
//                                   ke_task_id_t dest_id,
//                                   ke_task_id_t src_id);

// // int gapm_set_mode_req_evt_handler(ke_msg_id_t msgid,
// //                                             struct gapm_cmp_evt *param,
// //                                             ke_task_id_t  dest_id,
// //                                             ke_task_id_t src_id);

// int gapm_reset_req_evt_handler(ke_msg_id_t msgid,
//                                     struct gapm_cmp_evt *param,
//                                     ke_task_id_t dest_id,
//                                     ke_task_id_t src_id);

// int gapc_encrypt_req_ind_handler(ke_msg_id_t const msgid,
// 									struct gapc_encrypt_req_ind * param,
// 									ke_task_id_t const dest_id,
// 									ke_task_id_t const src_id);

// int gapc_encrypt_ind_handler(ke_msg_id_t const msgid,
//                              struct gapc_encrypt_ind *param,
// 						     ke_task_id_t const dest_id,
// 							 ke_task_id_t const src_id);

// int app_create_db_cfm_handler(ke_task_id_t const dest_id);

// void app_proxr_alert_ind(ke_msg_id_t msgid,
//                          struct proxr_alert_ind *param,
//                          ke_task_id_t dest_id,
//                          ke_task_id_t src_id);
/// @} APPTASK

#endif // APP_TASK_H_
