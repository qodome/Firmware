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

#include "uart.h"

#include "gap.h"
#include "gapc_task.h"
#include "gapm_task.h"

#include "disc.h"
#include "disc_task.h"
#include "smpc_task.h"

#include "wptc.h"
#include "wptc_task.h"

#include "app.h"
#include "app_task.h"

#if (BLE_APP_PTU)
#include "app_ptu.h"
#include "app_ptu_task.h"
#include "app_ptu_gtl_task.h"
#include "app_ptu_hooks.h"



/// Health Thermometer Application Task Descriptor
static const struct ke_task_desc TASK_DESC_APP_PTU = {NULL, &app_ptu_default_handler, app_ptu_state, APP_PTU_STATE_MAX, APP_PTU_IDX_MAX};

struct app_ptu_env_tag app_ptu_env __attribute__((section("exchange_mem_case1"))); //@WIKRETENTION MEMORY

/**
 ****************************************************************************************
 * @brief Set central master mode.
 *
 * @return void.
 ****************************************************************************************
 */
void app_ptu_set_mode(void) {

    struct gapm_set_dev_config_cmd /*gap_set_mode_req*/*msg = KE_MSG_ALLOC(
            GAPM_SET_DEV_CONFIG_CMD /*GAP_SET_MODE_REQ*/,TASK_GAPM, TASK_APP,
            gapm_set_dev_config_cmd /*gap_set_mode_req*/);

    msg->operation = GAPM_SET_DEV_CONFIG;
    // Device Role
    msg->role = GAP_CENTRAL_MST;

    ke_msg_send((void *) msg);

    return;
}

static int get_idx_free_wpts_struct(void)
{
    int i;
    for (i = 0;i < PTU_IMPL_MAX_SUPPORTED_DEV;++i)
    {
        if (!(app_ptu_env.wpts_devices[i].used))
            return i;
    }

    return -1;
}

static uint8_t get_category_from_pru_adv_data(uint8_t adv_data)
{
    if ((adv_data & WPT_ADV_PAYLOAD_IMP_SHIFT_CAT5) == WPT_ADV_PAYLOAD_IMP_SHIFT_CAT5)
    {
        return APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_5;
    }
    else if ((adv_data & WPT_ADV_PAYLOAD_IMP_SHIFT_CAT4) == WPT_ADV_PAYLOAD_IMP_SHIFT_CAT4)
    {
        return APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_4;
    }
    else if ((adv_data & WPT_ADV_PAYLOAD_IMP_SHIFT_CAT3) == WPT_ADV_PAYLOAD_IMP_SHIFT_CAT3)
    {
        return APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_3;
    }
    else if ((adv_data & WPT_ADV_PAYLOAD_IMP_SHIFT_CAT2) == WPT_ADV_PAYLOAD_IMP_SHIFT_CAT2)
    {
        return APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_2;
    }
    else if ((adv_data & WPT_ADV_PAYLOAD_IMP_SHIFT_CAT1) == WPT_ADV_PAYLOAD_IMP_SHIFT_CAT1)
    {
        return APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_1;
    }
    else if ((adv_data & WPT_ADV_PAYLOAD_IMP_SHIFT_RFU2) == 0)
    {
        return APP_PTU_ADV_IMPEDANCE_SHIFT_NEVER;
    }

    return APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_RFU;
}

/**
 * Used for checking bdaddress of device in current connected devices, to not
 * allow for 2nd fast conn attempt for connected device when disconnecting has
 * not yet finished on PTU side
 */
bool app_ptu_check_connecting_possible(struct bd_addr const *peer_addr, uint8_t pru_adv_flags)
{
    int i;

    ke_state_t app_state = ke_state_get(TASK_APP_PTU);

    // rejecting indication when no load detected, exception is mode transition in
    // which we are assuming that load is detected all the time when mode transition timer
    // is active
    if (!app_ptu_env.load_detected
            && !ke_timer_active(APP_PTU_TIMER_MODE_TRANSITION_EMPTY, TASK_APP)
            && !ke_timer_active(APP_PTU_TIMER_MODE_TRANSITION_ADDR, TASK_APP) // for mode transition impedance change is not needed
            && get_category_from_pru_adv_data(pru_adv_flags) != APP_PTU_ADV_IMPEDANCE_SHIFT_NEVER) // for impedance shift never, detecting impedance is not needed
    {
        // TODO: write here condition to allow mode transition addr device to pass
        // only and only if there reconnect address matches curr address
        // use find_wdev_in_transition_mode(...)
        return false;
    }

    // Rejecting adv indication when in wrong state
    if (app_state != APP_PTU_POWER_SAVE_STATE && app_state != APP_PTU_POWER_TRANSFER_STATE)
    {
        // NOTE: not sending notify when adv received in bad state - too many events can cause
        // host app to hang when filtering scan result is turned off
        return false;
    }

    // Rejecting when already reached max number of devices
    if (app_ptu_env.conn_count >= PTU_IMPL_MAX_SUPPORTED_DEV)
    {
        return false;
    }

    for (i = 0;i < PTU_IMPL_MAX_SUPPORTED_DEV;++i)
    {
        if ((app_ptu_env.wpts_devices[i].used) &&
             memcmp((app_ptu_env.wpts_devices[i].pub_data.dev_addr.addr), peer_addr->addr, BD_ADDR_LEN) == 0)
        {
            // We found same address in our connected list, so now we have
            // to check if Transition Mode with Addr is active for found device
            // If, so then we know that we found address for reconecct and it is
            // completely VALID

            if (!(ke_timer_active(APP_PTU_TIMER_MODE_TRANSITION_ADDR, TASK_APP)
                    && app_ptu_env.wpts_devices[i].mode_transition_or_reconnect_ongoing))
            {
                // same addr already connected, not in Mode Transition
                app_ptu_gtl_notify_discovery_event(APP_PTU_DISC_ERR_ALREADY_CONNECTED);

                return false;
            }
            else // Mode Transition active
            {
                // even in transition mode we have to check if found address is
                // same as addr used for reconnect. If so, then everything OK,
                // otherwise marking addr as invalid
                if (!memcmp((app_ptu_env.wpts_devices[i].pub_data.dev_addr.addr),
                        peer_addr->addr, BD_ADDR_LEN) == 0)
                {
                    app_ptu_gtl_notify_discovery_event(APP_PTU_DISC_ERR_ALREADY_CONNECTED);

                    return false;
                }

                //address valid
            }
        }
    }

    return true;
}

static void append_ptu_dev(uint16_t conn_hdl, uint8_t conn_idx,
                            struct bd_addr const *peer_addr, uint8_t wpt_adv, uint8_t rssi_flags_adv)
{
    wpts_dev *dev;
    uint8_t idx;

    idx = get_idx_free_wpts_struct();
    ASSERT_ERR(idx != -1);
    dev = &(app_ptu_env.wpts_devices[idx]);

    dev->pub_data.ble_conn_handle = conn_hdl;
    dev->ble_conn_idx = conn_idx;
    dev->pub_data.ptu_dev_state = DEV_PRU_IDLE;
    dev->used = true;
    dev->pub_data.imp_shift_category_adv = get_category_from_pru_adv_data(wpt_adv);
    dev->pub_data.time_set_supported_adv = (wpt_adv & WPT_ADV_PAYLOAD_TIME_SET_SUPPORTED);
    dev->pub_data.reboot_with_no_reset_adv = (wpt_adv & WPT_ADV_PAYLOAD_REBOOT_WITH_NO_RESET);
    dev->pub_data.ovp_status_supported = (wpt_adv & WPT_ADV_PAYLOAD_OVP_SUPPORTED);
    dev->pub_data.pru_gain_adv_val = (rssi_flags_adv & WPT_ADV_PAYLOAD_RSSI_GAIN_MASK);
    dev->pub_data.pru_pwr_adv_val = (rssi_flags_adv & WPT_ADV_PAYLOAD_RSSI_PWR_MASK);
    memcpy(&(dev->pub_data.dev_addr), peer_addr, BD_ADDR_LEN);

    // each connected device needs to have its own timer for reading dynamic
    // parameter
    dev->dyn_timer_id = (APP_PTU_TIMER_DYNAMIC_PARAM_0 + idx);

    //arch_printf("Appending PRU-device, conn %d |", conn_hdl);
    app_ptu_env.conn_count++;
}

static wpts_dev * find_wdev_in_transition_mode(const struct bd_addr *peer_addr)
{
    int i;

    for (i = 0;i < PTU_IMPL_MAX_SUPPORTED_DEV;++i)
    {
        if ((app_ptu_env.wpts_devices[i].used)
                && app_ptu_env.wpts_devices[i].mode_transition_or_reconnect_ongoing
                && memcmp((app_ptu_env.wpts_devices[i].reconnect_addr.addr),
                           peer_addr->addr, BD_ADDR_LEN) == 0)
        {
            return &(app_ptu_env.wpts_devices[i]);
        }
    }

    // nothing found
    return NULL;
}

static void handle_conn_in_mode_transition(wpts_dev *dev, uint16_t conhdl_now, uint8_t connidx_now)
{
    // connhdl and connidx may be different after re-connection
    dev->pub_data.ble_conn_handle = connidx_now;
    dev->ble_conn_idx = connidx_now;
    dev->pub_data.ptu_dev_state = DEV_PRU_IDLE;
    dev->wptc_trans_in_prog = false;
}

static void send_gtl_notify_device_count(void)
{
    struct app_ext_ptu_conn_devices_count_ntf *msg = KE_MSG_ALLOC(APP_PTU_EXT_CONN_DEVICES_COUNT_NTF,
                                                                 TASK_GTL, TASK_APP,
                                                                 app_ext_ptu_conn_devices_count_ntf);
     msg->device_count = app_ptu_env.conn_count;

     ke_msg_send(msg);
}

/**
 ****************************************************************************************
 * @brief Send enable request to WPTC profile task.
 *
 * @return void.
 ****************************************************************************************
 */
void app_ptu_enable(uint16_t conhdl, uint8_t connidx, uint16_t wpt_service_start_hdl,
                    uint8_t pru_rssi, uint8_t wpt_adv,
                    struct gapc_connection_req_ind const *param)
{
    // Allocate the message
    struct wptc_offline_enable_req * req;
    wpts_dev *dev_in_trans_mode = NULL;

    if (ke_timer_active(APP_PTU_TIMER_MODE_TRANSITION_ADDR, TASK_APP))
    {
        dev_in_trans_mode = find_wdev_in_transition_mode(&(param->peer_addr));
    }

    // NOTE: In transition mode-addr we won't increase connected device counter
    // because there is already connected device in our cache
    if ((app_ptu_env.conn_count + 1) <= PTU_IMPL_MAX_SUPPORTED_DEV || dev_in_trans_mode)
    {
        req = KE_MSG_ALLOC(WPTC_OFFLINE_ENABLE_REQ, KE_BUILD_ID(TASK_WPTC, connidx),
                   TASK_APP, wptc_offline_enable_req);

        // Fill in the parameter structure
        req->conhdl = conhdl;
        req->wpt_uuid_start = wpt_service_start_hdl;

        // TRANSITION MODE HANDLING
        if (dev_in_trans_mode)
        {
            handle_conn_in_mode_transition(dev_in_trans_mode, conhdl, connidx);
        }
        else
        {
            // normal appending dev procedure
            append_ptu_dev(conhdl, connidx, &(param->peer_addr), wpt_adv, pru_rssi);

            // need to finish registration in TIMER_REGISTRATION_VAL
            if (!ke_timer_active(APP_PTU_TIMER_REGISTRATION, TASK_APP))
            {
                ke_timer_set(APP_PTU_TIMER_REGISTRATION, TASK_APP, PTU_REGISTRATION_TIMER_VAL);
            }
        }

        // when we started to connect with dev, then we have to go to LOW POWER
        // @A4WP spec 5.2.4
        if (ke_state_get(TASK_APP_PTU) == APP_PTU_POWER_SAVE_STATE)
            app_ptu_state_set_with_notify(APP_PTU_LOW_POWER_STATE);

        // Send the message
        ke_msg_send((void *) req);
    }

    // Sending notification about current connected device count
    send_gtl_notify_device_count();
    app_ptu_gtl_send_update_of_wdev_tab();
}

/**
 ****************************************************************************************
 * @brief Send read request for PRU Dynamic Param characteristic to WPT Client profile task.
 *
 * @return void.
 ****************************************************************************************
 */
void app_ptu_read_pru_dynamic_param(wpts_dev *wdev)
{
    struct wptc_rd_pru_dynamic_param_req * req = KE_MSG_ALLOC(
            WPTC_RD_PRU_DYNAMIC_PARAMETER_REQ, KE_BUILD_ID(TASK_WPTC, wdev->ble_conn_idx), TASK_APP,
            wptc_rd_pru_dynamic_param_req);

    ASSERT_ERR(wdev != NULL);

    wdev->last_char = WTPC_RD_PRU_DYNAMIC_PARAM;
    req->conhdl = wdev->pub_data.ble_conn_handle;

    wdev->wptc_trans_in_prog = true;
    ke_msg_send((void *) req);
}

/**
 ****************************************************************************************
 * @brief Send read request for PRU Dynamic Param characteristic to WPT Client profile task.
 *
 * @return void.
 ****************************************************************************************
 */
static void app_ptu_read_pru_static_param(wpts_dev *wdev)
{
    struct wptc_rd_pru_static_param_req * req = KE_MSG_ALLOC(
            WPTC_RD_PRU_STATIC_PARAMETER_REQ, KE_BUILD_ID(TASK_WPTC, wdev->ble_conn_idx), TASK_APP,
            wptc_rd_pru_static_param_req);

    ASSERT_ERR(wdev != NULL);

    wdev->last_char = WTPC_RD_PRU_STATIC_PARAM;
    req->conhdl = wdev->pub_data.ble_conn_handle;

    wdev->wptc_trans_in_prog = true;
    ke_msg_send((void *) req);
}

/**
 */
static void app_ptu_ext_send_validate_time_set_ntf(uint16_t connhdl, uint8_t time_set_val)
{
    struct app_ext_ptu_time_set_check_req *msg = KE_MSG_ALLOC(APP_PTU_EXT_VALIDATE_TIME_SET_NTF,
                                                         TASK_GTL, TASK_APP,
                                                         app_ext_ptu_time_set_check_req);
    msg->connhdl = connhdl;
    msg->time_set_val = time_set_val;

    ke_msg_send(msg);
}

/**
 ****************************************************************************************
 * @brief Send write request to WPTC profile task.
 *
 ****************************************************************************************
 */
static void app_ptu_write_pru_control(wpts_dev *wdev, uint8_t enables_val, uint8_t permission_val)
{

    struct wptc_wr_pru_control_req * req = KE_MSG_ALLOC(WPTC_WR_PRU_CONTROL_REQ,
            KE_BUILD_ID(TASK_WPTC, wdev->ble_conn_idx), TASK_APP, wptc_wr_pru_control_req);

    req->conhdl = wdev->pub_data.ble_conn_handle;

    req->pru_control_val.enables = enables_val;
    req->pru_control_val.permission = permission_val;
    // Sending time set field val only if enabling PRU output and only if it is supported
    // TimeSet supported information is taken from advertising packet
    if (enables_val && app_ptu_env.time_set_default_val)
    {
        req->pru_control_val.time_set = app_ptu_env.time_set_default_val;
			
				// if time set is supported send notification about validation
				if(wdev->pub_data.time_set_supported_adv)
				{
						// send notification for device about time set validation
						app_ptu_ext_send_validate_time_set_ntf(wdev->pub_data.ble_conn_handle, req->pru_control_val.time_set);
				}
    }

    wdev->wptc_trans_in_prog = true;
    wdev->last_char = WPTC_WR_PRU_CONTROL_PARAM;

    ke_msg_send((void *) req);
}

/**
 ****************************************************************************************
 * @brief Send write request to proximity monitor profile task.
 *
 *  @param[in] chr Characteristic to be written.
 *  @param[in] val Characteristic's Value.
 *
 * @return true if adresses are equal / false if not.
 ****************************************************************************************
 */
static void app_ptu_write_ptu_static_param(wpts_dev *wdev)
{
    struct wptc_wr_ptu_static_req * req = KE_MSG_ALLOC(
            WPTC_WR_PTU_STATIC_PARAMETER_REQ, KE_BUILD_ID(TASK_WPTC, wdev->ble_conn_idx), TASK_APP,
            wptc_wr_ptu_static_req);

    req->conhdl = wdev->pub_data.ble_conn_handle;

    memcpy(&(req->ptu_static_val), &(app_ptu_env.ptu_stat_val), WPTS_PTU_STATIC_CHAR_LEN);

    wdev->last_char = WPTC_WR_PTU_STATIC_PARAM;
    wdev->wptc_trans_in_prog = true;

    ke_msg_send((void *) req);
}

/**
 ****************************************************************************************
 * @brief Send write request to proximity monitor profile task.
 *
 *  @param[in] chr Characteristic to be written.
 *  @param[in] val Characteristic's Value.
 *
 * @return true if adresses are equal / false if not.
 ****************************************************************************************
 */
void app_ptu_write_enable_alert_notifications(wpts_dev *wdev, unsigned char cfg_ntf_val)
{
    struct wptc_alert_cfg_indntf_req * req = KE_MSG_ALLOC(
            WPTC_SEND_ALERT_CFG_INDNTF_REQ, KE_BUILD_ID(TASK_WPTC, wdev->ble_conn_idx),
            TASK_APP,
            wptc_alert_cfg_indntf_req);
    req->conhdl = wdev->pub_data.ble_conn_handle;
    req->cfg_val = cfg_ntf_val;

    wdev->last_char = WPTC_WR_ENABLE_NOTIFICATIONS;
    wdev->wptc_trans_in_prog = true;
    ke_msg_send((void *) req);
}

static void do_disconnect(wpts_dev *dev)
{
    dev->pub_data.ptu_dev_state = DEV_PRU_DISCONNECTING;

//        arch_printf("Disconnect REQ device with idx %d, conn hdl %d | ",
//                dev->ble_conn_idx, dev->ble_conn_handle);
    dev->disconnect_init_by_ptu = true;
    app_disconnect_connidx(dev->ble_conn_idx);
}

static void dev_state_handler(wpts_dev *wdev)
{

    if (!wdev->used)
        return;

    switch (wdev->pub_data.ptu_dev_state)
    {
        case DEV_PRU_DISCONNECTED:
        {

        }
        break;
        case DEV_PRU_IDLE:
        {

        }
        break;
        case DEV_PRU_PRUST_RD:
        {
            // in this state we have to retrieve PRU Static value
            if (!wdev->wptc_trans_in_prog)
            {
                app_ptu_read_pru_static_param(wdev);
            }
        }
        break;
        case DEV_PRU_PTUST_WR:
        {
            if (!wdev->wptc_trans_in_prog)
                    app_ptu_write_ptu_static_param(wdev);
        }
        break;
        case DEV_PRU_DYN_RD1:
        {
            // in this state we have to read PRU Dynamic value
            if (!wdev->wptc_trans_in_prog)
                     app_ptu_read_pru_dynamic_param(wdev);
        }
        break;
        case DEV_PRU_PWDADJ_NEW_REG_WR:
        {
            // in this state we have to write PRU Control value
            if (!wdev->wptc_trans_in_prog)
                app_ptu_write_pru_control(wdev, wdev->shared_mode_val, WPTS_CTRL_PERM_TIME_AVAIL_POW);
        }
        break;
        case DEV_PRU_PWADJ_NEW_REG_DYN_RD_CFM:
        case DEV_PRU_PWADJ_IN_POWER_TRANSFERR_DYN_RD_CFM:
        {
            // in this state we have to read PRU Dynamic value to confirm that PRU is fine with lower
            // power adjustment than its Prect max
            if (!ke_timer_active(wdev->dyn_timer_id, TASK_APP))
            {
                ke_timer_set(wdev->dyn_timer_id, TASK_APP, PTU_DYNAMIC_TIMER_VAL);
            }
        }
        break;
        case DEV_PRU_PWADJ_PRU_CTRL_WAIT_FOR_POWER_PERM_WR:
        {
            if (!wdev->wptc_trans_in_prog)
                app_ptu_write_pru_control(wdev, WPTS_CTRL_DISABLE_PRU_OUTPUT, WPTS_CTRL_PERM_TIME_AVAIL_POW);
        }
        break;
        case DEV_PRU_PWADJ_WAIT_FOR_OTHER_PRU_POW_ADJUST:
        {
            // only waiting, doing nothing
        }
        break;
        case DEV_PRU_ALERT_NOTIF_REGISTER:
        {
            // in this state we have to register for notifications
            if (!wdev->wptc_trans_in_prog)
            {
                if (!wdev->ntf_ind_set_fail_before)
                {
                    // at the beginning trying to set both indications and notifications
                    app_ptu_write_enable_alert_notifications(wdev, (PRF_CLI_START_NTF|PRF_CLI_START_IND));
                }
                else
                {
                   // if setting both NTF+IND failed, then falling back to normal notifications

                   app_ptu_write_enable_alert_notifications(wdev, (PRF_CLI_START_NTF) );
                }
            }
        }
        break;
        case DEV_PRU_CTRL_START_CHARGING:
        {
            // clearing Mode Transition timer when initialization is now finished
            if (ke_timer_active(APP_PTU_TIMER_MODE_TRANSITION_EMPTY, TASK_APP))
                ke_timer_clear(APP_PTU_TIMER_MODE_TRANSITION_EMPTY, TASK_APP);

            // NOTE: transition mode <<address>> timer is canceled only when target
            // device reconnects - it can't be done here because this timer could
            // be cancelled for other device wrongly

            // in this state we have to read PRU Dynamic 2d time value
            if (!wdev->wptc_trans_in_prog)
                app_ptu_write_pru_control(wdev,
                                 (WPTS_CTRL_ENABLE_PRU_OUTPUT|WPTS_CTRL_ENABLE_PRU_CHARGE_IND|wdev->shared_mode_val),
                                 WPTS_CTRL_PERM_WITHOUT_REASON);
        }
        break;
        case DEV_PRU_DYN_READ_CHARGING:
        {
            // handling continuus read of PRU dynamic param
            if (!ke_timer_active(wdev->dyn_timer_id, TASK_APP))
            {
                ke_timer_set(wdev->dyn_timer_id, TASK_APP, PTU_DYNAMIC_TIMER_VAL);
            }
        }
        break;
        case DEV_PRU_PWADJ_IN_POWER_TRANSFER_PRU_CTRL_WR:
        {
            if (!wdev->wptc_trans_in_prog)
                app_ptu_write_pru_control(wdev,
                                    (WPTS_CTRL_ENABLE_PRU_OUTPUT|WPTS_CTRL_ENABLE_PRU_CHARGE_IND|wdev->shared_mode_val),
                                    WPTS_CTRL_PERM_WITHOUT_REASON);
        }
        break;
        case DEV_PRU_CTRL_STOP_CHARGING:
        {
            // in this state we have to read PRU Dynamic 2d time value
            if (!wdev->wptc_trans_in_prog)
                app_ptu_write_pru_control(wdev, WPTS_CTRL_DISABLE_PRU_OUTPUT, WPTS_CTRL_PERM_WITHOUT_REASON);
        }
        break;
        case DEV_PRU_CTRL_ERR_REJECT_PRU_CHARGING:
        {
            // in this state we have to notify PRU dev that we don't have enough power for charging
            if (!wdev->wptc_trans_in_prog)
                app_ptu_write_pru_control(wdev, WPTS_CTRL_DISABLE_PRU_OUTPUT, wdev->reject_reason);
        }
        break;
        case DEV_PRU_DISCONNECTING:
        {
            if (!wdev->wptc_trans_in_prog)
                do_disconnect(wdev);
        }
        break;

        default:
            break;
        }
}

static void disconnect_all_wdev_in_fault_state(wpts_dev *wdev)
{
    int i;

    if (wdev)
    {
        do_disconnect(wdev);
    }
    else if (app_ptu_env.conn_count > 0) // wdev not passed, have to check manually if all devices
                                            // are disconnected
    {
        for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
        {
            if (app_ptu_env.wpts_devices[i].used
                    && app_ptu_env.wpts_devices[i].pub_data.ptu_dev_state != DEV_PRU_DISCONNECTED
                    && app_ptu_env.wpts_devices[i].pub_data.ptu_dev_state != DEV_PRU_DISCONNECTING)
            {
                do_disconnect(&(app_ptu_env.wpts_devices[i]));
                break; // only disconnecting one at a time
            }
        }
    }
}

void app_ptu_state_handler(wpts_dev *wdev)
{

    int i;

    switch (ke_state_get(TASK_APP_PTU))
    {
        case APP_PTU_CONFIGURATION_STATE:
        {
            
        }
        break;
        case APP_PTU_POWER_SAVE_STATE:
        {
            if (wdev)
            {
                // if correct device is thrown here, in POWER_SAVE, it obviously
                // means some registration error and device should be disconnected
                // immediately
                if (!wdev->wptc_trans_in_prog)
                    do_disconnect(wdev);
            }
            //clearing unused timers in power state
            ke_timer_clear(APP_PTU_TIMER_REGISTRATION, TASK_APP);

        }
        break;
        case APP_PTU_LOW_POWER_STATE:
        {
            for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
            {
                dev_state_handler(& (app_ptu_env.wpts_devices[i]));
            }

        }
        break;
        case APP_PTU_POWER_TRANSFER_STATE:
        {
            if (wdev)
                dev_state_handler(wdev);

        }
        break;
        case APP_PTU_LATCHING_FAULT_STATE:
        {
            // in latching fault we should disconnect everything
            disconnect_all_wdev_in_fault_state(wdev);

            //clearing unused timers for latching state
            ke_timer_clear(APP_PTU_TIMER_REGISTRATION, TASK_APP);
            ke_timer_clear(APP_PTU_TIMER_DYNAMIC_PARAM_0, TASK_APP);
            ke_timer_clear(APP_PTU_TIMER_DYNAMIC_PARAM_1, TASK_APP);
            ke_timer_clear(APP_PTU_TIMER_DYNAMIC_PARAM_2, TASK_APP);
            ke_timer_clear(APP_PTU_TIMER_DYNAMIC_PARAM_3, TASK_APP);
            ke_timer_clear(APP_PTU_TIMER_POWER_ADJUST_EXPIRE, TASK_APP);
            ke_timer_clear(APP_PTU_TIMER_POWER_ADJUST_ROLLBACK_EXPIRE, TASK_APP);

            if (app_ptu_env.conn_count == 0)
            {
                if (app_ptu_env.self_clear_counter < PTU_MAX_SELF_CLEAR_COUNTER)
                {
                    // All devices disconnected now, so if we are in latching fault state
                    // we need to try self-clear-latching procedure
                    // This means going into POWER SAVE State directly
                    app_ptu_env.self_clear_counter++;
                    app_ptu_latching_fault_cleared_handle();
                }
                else // self clear counter reached, user has to clear latching fault
                {
                    void *msg = ke_msg_alloc(APP_PTU_EXT_USER_LATCHING_CLEAR_NEEDED_NTF,
                                            TASK_GTL, TASK_APP, 0);
                    ke_msg_send(msg);

                    ke_timer_clear(APP_PTU_TIMER_SELF_LATCHING_FAULT_CLEAR, TASK_APP);
                }
            }
        }
        break;
        case APP_PTU_LOCAL_FAULT_STATE:
        {
            // in local fault we should disconnect everything
            disconnect_all_wdev_in_fault_state(wdev);

            // clearing unused timers for local fault state
            ke_timer_clear(APP_PTU_TIMER_REGISTRATION, TASK_APP);
        }
        break;
        default:
            break;
        }

}

/**
 * @param wdev - device to be tested
 * @param disc_dev_connhdl - connidx to be tested, GAP_INVALID_CONHDL if unused
 * @param disc_dev_connidx - connidx to be tested, GAP_INVALID_CONIDX if unused
 * @param initiaded_by_ptu - true if device disconnection should be initiated by PTU
 */
static bool is_device_to_remove(wpts_dev *wdev, uint16_t disc_dev_connhdl,
                                uint8_t disc_dev_connidx, bool initiated_by_ptu)
{
    // deleting wdev struct when not needed - only exception is Mode Transition
    // with given address when device is disconnected but we are pretending
    // that still conn is active until reconnect will happen
    if (!wdev->used)
        return false;

    if (wdev->disconnect_init_by_ptu != initiated_by_ptu)
        return false;

    if (disc_dev_connhdl != GAP_INVALID_CONHDL && wdev->pub_data.ble_conn_handle != disc_dev_connhdl)
        return false;

    if (disc_dev_connidx != GAP_INVALID_CONIDX && wdev->ble_conn_idx != disc_dev_connidx)
        return false;

     return true;
}

/**
 * For reconnect we are using kind of mode transition with shorter timer (1.1 second)
 * It is basically identical scenario like mode transition with address. If reconnect
 * fails, then going into latching fault mode.
 */
static void handle_wdev_reconnect(wpts_dev *wdev)
{
    wdev->reconnect_addr = wdev->pub_data.dev_addr;
    ke_timer_set(APP_PTU_TIMER_MODE_TRANSITION_ADDR, TASK_APP, PTU_MODE_TRANSITION_RECONN_1_1S_TIMER_VAL);
    wdev->mode_transition_or_reconnect_ongoing = true;

}

static void handle_wdev_disconnect_predicted(wpts_dev *wdev, uint16_t connhdl, bool disc_with_ptxin_variation)
{
    if (disc_with_ptxin_variation)
    {
        ke_timer_clear(APP_PTU_TIMER_PTXIN_VARIATION_EXPIRE, TASK_APP);
        app_ptu_env.ptxin_variation_detect = false;
        app_ptu_send_ext_error_ntf(APP_PTU_ERR_PRU_REMOVED_FROM_CHARGE_AREA, connhdl);
    }

    app_ptu_internal_clear_wdev_data(wdev); // reseting entire wpts_dev device struct when already disconnected

    // If afterwards, the system registry is empty, the PTU shall enter the PTU Power Save State.
    // A4WP #5.2.2.2
    if (app_ptu_env.conn_count == 0)
    {
        app_ptu_state_set_with_notify(APP_PTU_POWER_SAVE_STATE);
    }

    // NOTE: when device disconnect occured by mode transition empty (disc_with_ptxin_variation = false)
    // then no proceeding with power sharing - there is no time for that and if reconnect fails via mode
    // transition everything will end in latching fault anyway
    if (ke_state_get(TASK_APP_PTU) == APP_PTU_POWER_TRANSFER_STATE && disc_with_ptxin_variation)
        app_ptu_task_handle_power_sharing(NULL, false);
}

// This is triggered when disconnection occurred, no matter if initiated by PTU
// or not
void app_ptu_disconnect_complete(struct gapc_disconnect_ind const *disc_ind_param)
{
    int i;
	bool dev_predicted_remove = false;

    for (i = 0;i < PTU_IMPL_MAX_SUPPORTED_DEV;++i)
    {
        if (is_device_to_remove(&(app_ptu_env.wpts_devices[i]),
                disc_ind_param->conhdl, GAP_INVALID_CONIDX, false))
        {
            // TRANSITION MODE WITH ADDR active for disconnected device
            if (ke_timer_active(APP_PTU_TIMER_MODE_TRANSITION_ADDR, TASK_APP)
                     && app_ptu_env.wpts_devices[i].mode_transition_or_reconnect_ongoing)
            {
                // in case of dev in transition mode disconnected only invalidating
                // connhdl and connidx - after reconnection addresses will be different
                // NOTE: this handling has to be only in this disconnect handler, because
                // we never initiate disconnect for Transition Mode device
                app_ptu_env.wpts_devices[i].pub_data.ble_conn_handle = GAP_INVALID_CONHDL;
                app_ptu_env.wpts_devices[i].ble_conn_idx = GAP_INVALID_CONIDX;
            }
            else if (ke_timer_active(APP_PTU_TIMER_MODE_TRANSITION_EMPTY, TASK_APP))// re-connect thingies only when NO mode transition exsits
            {
                // mode transition with no address - we have predicted this disconnect,
                // so removing dev data and NO need to get into latching fault
                dev_predicted_remove = true;
                handle_wdev_disconnect_predicted(&(app_ptu_env.wpts_devices[i]),
                                                  disc_ind_param->conhdl, false);
            }
            else
            {
                if (app_ptu_env.ptxin_variation_detect)
                {
                    // If the PTU link supervision timer expires with greater than or equal to 2W of PTX_IN variation
                    // before the timer expires, then the PRU shall be removed from the system registry.
                    // A4WP #5.2.2.2
                    dev_predicted_remove = true;
                    handle_wdev_disconnect_predicted(&(app_ptu_env.wpts_devices[i]),
                                                               disc_ind_param->conhdl, true);

                }
                else
                {
                    // initializing kind of mode transition with address to allow reconnect
                    handle_wdev_reconnect(&(app_ptu_env.wpts_devices[i]));
                }
            }

            // going to latching fault when unpredicted disconnect occurred (on device
            // that we NOT initated disconnec
            // exceptions
            // - disconnect was 'predicted' with ptxin variation > 2W to mark device removal
            // - mode is transition ongoing (address or without address)
            if (!dev_predicted_remove
                       && disc_ind_param->reason != CO_ERROR_CON_TERM_BY_LOCAL_HOST
                       && !ke_timer_active(APP_PTU_TIMER_MODE_TRANSITION_EMPTY, TASK_APP)
                       && !ke_timer_active(APP_PTU_TIMER_MODE_TRANSITION_ADDR, TASK_APP))
            {
                // Remote side disconnected or disconnect error
                app_ptu_send_ext_error_ntf(APP_PTU_ERR_LINK_LOSS, disc_ind_param->conhdl);

                app_ptu_state_set_with_notify(APP_PTU_LATCHING_FAULT_STATE);
            }

            break;
        }
    }

//    arch_printf("|| Disc ind complete, curr conn %d, conn_hdl %d ||",
//                app_ptu_env.conn_count, disc_ind_param->conhdl );

    app_ptu_state_handler(NULL);
}

// This is triggered when disconnection command initiated by PTU completes
void app_ptu_gapc_disc_cmplt(uint8_t conn_idx, uint8_t status)
{
    int i;

    for (i = 0;i < PTU_IMPL_MAX_SUPPORTED_DEV;++i)
    {
        // when disconnection initiated by PTU then we have to do cleanup here,
        // because this event is triggered later than regular event handled by
        // app_ptu_disconnect_complete(..).
        if (is_device_to_remove(&(app_ptu_env.wpts_devices[i]),
                GAP_INVALID_CONHDL, conn_idx, true)) // only removing if disc initiated by ptu (true)
        {
            // reseting entire wpts_dev device struct when already disconnected
            app_ptu_internal_clear_wdev_data(&(app_ptu_env.wpts_devices[i]));

            break;
        }
    }

    //arch_printf("|| GAPC DISC ind complete, curr conn %d, conn_idx %d ||", app_ptu_env.conn_count, conn_idx );

    app_ptu_state_handler(NULL);

    // after finished disconnect, if we are still in POWER_TRANSFER_STATE we need
    // to check if power drawn by remaining PRU's can be increased
    if (ke_state_get(TASK_APP_PTU) == APP_PTU_POWER_TRANSFER_STATE)
        app_ptu_task_handle_power_sharing(NULL, false);
}
/**
 ****************************************************************************************
 * @brief Application's main function.
 *
 ****************************************************************************************
 */
void app_ptu_init(void)
{
    // Reset the environment
    memset(&app_ptu_env, 0, sizeof(app_ptu_env));

    // Create APP_HT task
    ke_task_create(TASK_APP_PTU, &TASK_DESC_APP_PTU);

    // Go to disabled state
    ke_state_set(TASK_APP_PTU, APP_PTU_CONFIGURATION_STATE);

    // Sending notification about init completed to GTL Task in Host App
    struct app_ext_ptu_generic_rsp *msg = KE_MSG_ALLOC(APP_PTU_EXT_INIT_COMPLETE_NTF,
                                                                 TASK_GTL, TASK_APP,
                                                                 app_ext_ptu_generic_rsp);
    msg->status = APP_PTU_NO_ERROR;

    ke_msg_send(msg);
}

/**
 * Called every time after device reset & configuration (this is done not only
 * after init but also when device scan has to be re-called after registration
 * problem (timeout, bad characteristic value during registration)
 */
void app_ptu_configuration_finished(void)
{
    // after reset there should not be active devices
    app_ptu_env.conn_count = 0;
    app_ptu_env.ptxin_variation_detect = false;
    app_ptu_env.reg_counter = 0;
    app_ptu_env.time_set_default_val = WPTS_CTRL_TIME_10_MS; // need to set this, because '0' is val for RFU
    memset(app_ptu_env.wpts_devices, 0, (sizeof(wpts_dev) * PTU_IMPL_MAX_SUPPORTED_DEV));

    app_ptu_state_set_with_notify(APP_PTU_POWER_SAVE_STATE);
    app_ptu_gtl_send_update_of_wdev_tab();
}

void app_ptu_load_detected_handle(void)
{
    app_ptu_env.load_detected = true;
}

void app_ptu_latching_fault_cleared_handle(void)
{

    ke_timer_set(APP_PTU_TIMER_SELF_LATCHING_FAULT_CLEAR, TASK_APP, PTU_SELF_LATCHING_CLEAR_TIMER_VAL);
    // Going from Latching directly to Power Save after latching is cleared, #5.2.8.2
    if (ke_state_get(TASK_APP_PTU) == APP_PTU_LATCHING_FAULT_STATE)
    {
        app_ptu_configuration_finished();
    }

}

void app_ptu_local_fault_detected_handle(uint8_t local_fault_err)
{
    // caching info if current state was latching before going to Local Fault
    app_ptu_env.was_state_latching = (ke_state_get(TASK_APP_PTU) == APP_PTU_LATCHING_FAULT_STATE);
    app_ptu_state_set_with_notify(APP_PTU_LOCAL_FAULT_STATE);
}

void app_ptu_local_fault_cleared_handle(void)
{
    if (app_ptu_env.was_state_latching) // going to latching if that state was before Local Fault
        app_ptu_state_set_with_notify(APP_PTU_LATCHING_FAULT_STATE);
    else
    {
        // NOTE: GOING INTO POWER SAVE instead
        // CONFIGURATION, basically it is the same because nothing being done in configuration state
        app_ptu_configuration_finished();// <- triggers 'Go to Power save state'
    }

}

/**
 * sets state for APP TASK and informs TASK GTL about state change
 */
void app_ptu_state_set_with_notify(ke_state_t const state_id)
{
    struct app_ext_ptu_state_ntf *msg = KE_MSG_ALLOC(APP_PTU_EXT_STATE_CHANGED_NTF,
                                                             TASK_GTL, TASK_APP,
                                                             app_ext_ptu_state_ntf);
    msg->state_new = state_id;

    ke_msg_send(msg);

    ke_state_set(TASK_APP_PTU, state_id);
}

/**
 * sets state for APP TASK and informs TASK GTL about state change
 */
void app_ptu_gtl_send_update_of_wdev_tab(void)
{
    int i, tab_msg_idx;
    struct wpts_pub_data tmp_active_wpts_dev[PTU_IMPL_MAX_SUPPORTED_DEV];

    tab_msg_idx = 0;
    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        if (app_ptu_env.wpts_devices[i].used)
        {
            tmp_active_wpts_dev[tab_msg_idx] = app_ptu_env.wpts_devices[i].pub_data;
            tab_msg_idx++;
        }
    }

    const uint16_t dyn_tab_size = tab_msg_idx * sizeof(struct wpts_pub_data);
    struct app_ext_ptu_pru_dev_ntf *msg = KE_MSG_ALLOC_DYN(APP_PTU_EXT_CONN_DEVICES_CHG_NTF,
                                                       TASK_GTL, TASK_APP,
                                                       app_ext_ptu_pru_dev_ntf,
                                                       dyn_tab_size);
    msg->wpts_dev_count = tab_msg_idx;
    memcpy(&(msg->wpts_devices), &(tmp_active_wpts_dev), dyn_tab_size);

    ke_msg_send(msg);
}

/**
 * @param err_ptu_val PTU error enum message
 * @param connhdl connection handle, pass GAP_INVALID_CONHDL if handle not available for
 * error
 */
void app_ptu_send_ext_error_ntf(uint8_t err_ptu_val, uint16_t connhdl)
{
    struct app_ext_ptu_pru_error_ntf *msg = KE_MSG_ALLOC(APP_PTU_EXT_PRU_ERROR_NTF,
                                                         TASK_GTL, TASK_APP,
                                                         app_ext_ptu_pru_error_ntf);
    msg->err_val = err_ptu_val;
    msg->connhdl = connhdl;

    ke_msg_send(msg);
}

void app_ptu_internal_clear_wdev_data(wpts_dev *wdev)
{
    memset(wdev, 0, sizeof (wpts_dev));
    if (app_ptu_env.conn_count > 0)
        app_ptu_env.conn_count--;

    // Sending notification about current connected device count
    send_gtl_notify_device_count();
    app_ptu_gtl_send_update_of_wdev_tab();
}

void app_ptu_gtl_notify_discovery_event(uint8_t discovery_event)
{
    struct app_ext_ptu_generic_rsp *msg = KE_MSG_ALLOC(APP_PTU_EXT_DISCOVERY_EVENT_NTF,
                                                            TASK_GTL, TASK_APP,
                                                            app_ext_ptu_generic_rsp);
    msg->status = discovery_event;

    ke_msg_send(msg);
}

static void set_state_all_devices(uint8_t state)
{
    int i;

    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        if (app_ptu_env.wpts_devices[i].used)
        {
            app_ptu_env.wpts_devices[i].pub_data.ptu_dev_state = state;

            //app_ptu_env.wpts_devices[i].wptc_trans_in_prog = false;
        }
    }
}

void app_ptu_proceed_to_latching_fault(void)
{
    set_state_all_devices(DEV_PRU_IDLE);
    app_ptu_state_set_with_notify(APP_PTU_LATCHING_FAULT_STATE);

    app_ptu_state_handler(NULL);
}

void app_ptu_power_share_adjustment_complete(uint8_t status)
{
    struct app_ext_ptu_power_share_adjustment_complete_ntf *msg = KE_MSG_ALLOC(
                                                        APP_PTU_EXT_POWER_SHARE_ADJUSTMENT_COMPLETE_NTF,
                                                        TASK_GTL, TASK_APP,
                                                        app_ext_ptu_power_share_adjustment_complete_ntf);

    msg->status = status;
    ke_msg_send(msg);
}	

bool app_ptu_is_addr_in_weak_rssi_cache(struct bd_addr *pru_addr)
{
     return (memcmp((app_ptu_env.rssi_weak_cache.dev_addr.addr),
             pru_addr->addr, BD_ADDR_LEN) == 0 );
}

void app_ptu_reset_rssi_weak_cache(struct bd_addr * addr)
{
    if (app_ptu_is_addr_in_weak_rssi_cache(addr))
    {
        memset(&(app_ptu_env.rssi_weak_cache.dev_addr), 0, sizeof (struct bd_addr));
        app_ptu_env.rssi_weak_cache.valid_timeout = false;
        app_ptu_env.rssi_weak_cache.count = 0;

        ke_timer_clear(APP_PTU_TIMER_RSSI_WEAK_ADV_WAIT, TASK_APP); //if other packet came in the middle
    }
}
#endif //(BLE_APP_PTU)
