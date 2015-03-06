/**
****************************************************************************************
*
* @file app_ptu.h
*
* @brief Header file Main A4WP Wireless Power Transfer PTU application.
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

#ifndef APP_PRU_H_
#define APP_PRU_H_

#include "co_bt.h"
#include "smpc_task.h"
#include "disc.h"

#include "wpt_common.h"
#include "wpts_task.h"
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>            // standard integer

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

 /// Advertising data used by Simplified Api configuration
struct app_pru_ext_adv_data
{
    uint16_t interval;
    uint8_t rssi;
    uint8_t adv_flags;
    uint8_t dev_name[17]; // ended with zero
};

/// Application environment structure
struct app_pru_env_tag
{
#if BLE_INTEGRATED_HOST_GTL
    struct app_pru_ext_adv_data ext_adv;
#endif // BLE_INTEGRATED_HOST_GTL
};

extern struct app_pru_env_tag app_pru_env;

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
 
 
/**
 ****************************************************************************************
 * @brief Initialize the BLE demo application.
 *
 ****************************************************************************************
 */

/// Funtions called from app_ptu_task.c
void app_pru_init(void);

void app_pru_create_db_send(void);

void app_pru_set_mode(void);

void app_pru_configuration_finished(void);

void app_pru_enable(uint16_t conhdl);

void app_pru_connect_complete(void);

void app_pru_disconnect_complete(struct gapc_disconnect_ind const *disc_ind_param);

void app_pru_gapc_disc_cmplt(uint8_t conn_idx, uint8_t status);

// ----- INTEGRATED API ------

void app_pru_ext_dev_ready_ntf(void);

void app_pru_ext_send_connected_ind(void);

void app_pru_ext_send_disconnected_ind(void);

void app_pru_ext_disconnection_cfm(uint8_t status);

/// @} APP

#endif // APP_PRU_H_
