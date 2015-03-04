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

#include "app_task.h"
#include "gapm_task.h"
#include "gapc_task.h"
#include "disc.h"
#include "disc_task.h"
#include "smpc_task.h"
#include "wptc.h"
#include "wptc_task.h" 

/* for basic APP API, eg. connecting, scanning...*/
#include "app.h"
#include "app_task.h"

#if (BLE_APP_PTU)
#include "app_ptu.h"
#include "app_ptu_task.h"
#include "app_ptu_hooks.h"

#define APP_PTU_POWER_TAB_LEN (140)

/**
 * @brief table with Watts values for 'ptu_power' parameter in PTU Static Value Characteristic
 */
const float app_ptu_static_power_tab[APP_PTU_POWER_TAB_LEN] = { 0.0f,  0.1f,  0.2f,  0.3f,  0.4f,  0.5f,  0.6f,  0.7f,  0.8f,  0.9f,  1.0f,  1.1f,  1.2f,  1.3f,  1.4f,  1.5f,  1.6f,  1.7f,  1.8f,  1.9f, // 0 -19};
                                                                2.0f,  2.2f,  2.4f,  2.6f,  2.8f,  3.0f,  3.2f,  3.4f,  3.6f,  3.8f,  4.0f,  4.2f,  4.4f,  4.6f,  4.8f,  5.0f,  5.2f,  5.4f,  5.6f,  5.8f, //20 - 39
                                                                6.0f,  6.3f,  6.6f,  6.9f,  7.2f,  7.5f,  7.8f,  8.1f,  8.4f,  8.7f,  9.0f,  9.3f,  9.6f,  9.9f, 10.2f, 10.5f, 10.8f, 11.1f, 11.4f, 11.7f, //40 - 59
                                                               12.0f, 12.4f, 12.8f, 13.2f, 13.6f, 14.0f, 14.4f, 14.8f, 15.2f, 15.6f, 16.0f, 16.4f, 16.9f, 17.2f, 17.6f, 18.0f, 18.4f, 18.8f, 19.2f, 19.6f, //60 - 79
                                                               20.0f, 20.5f, 21.0f, 21.5f, 22.0f, 22.5f, 23.0f, 23.5f, 24.0f, 24.5f, 25.0f, 25.5f, 26.0f, 26.5f, 27.0f, 27.5f, 28.0f, 28.5f, 29.0f, 29.5f, //80 - 99
                                                               30.0f, 30.6f, 31.2f, 31.8f, 32.4f, 33.0f, 33.6f, 34.2f, 34.8f, 35.4f, 36.0f, 36.6f, 37.2f, 37.8f, 38.4f, 39.0f, 39.6f, 40.2f, 40.8f, 41.4f, //100 - 119
                                                               42.0f, 43.0f, 44.0f, 45.0f, 46.0f, 47.0f, 48.0f, 49.0f, 50.0f, 51.0f, 52.0f, 53.0f, 54.0f, 55.0f, 56.0f, 57.0f, 58.0f, 59.0f, 60.0f, 61.0f, //120 - 139
                                                                };

static bool is_power_adjustment_finished(void)
{
    int i;

    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        wpts_dev *wdev = &(app_ptu_env.wpts_devices[i]);
        if (wdev->used &&
                (wdev->pub_data.ptu_dev_state == DEV_PRU_PWDADJ_NEW_REG_WR
                        ||wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_NEW_REG_DYN_RD_CFM
                        || wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_IN_POWER_TRANSFER_PRU_CTRL_WR
                        || wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_IN_POWER_TRANSFERR_DYN_RD_CFM))
        {
            // still there are devices that are waiting for
            return false;
        }
    }

    return true;
}

/**
 * @brief Used to find device that is currently in registration phase - this info
 * is needed to find which device has to be processed when registration timer will
 * be ended
 */
static wpts_dev * get_dev_in_registration_phase(void)
{
    int i;

    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        if (app_ptu_env.wpts_devices[i].used &&
                app_ptu_env.wpts_devices[i].pub_data.ptu_dev_state < DEV_PRU_DYN_READ_CHARGING)
        {
            return &(app_ptu_env.wpts_devices[i]);
        }
    }

    return NULL;
}

static wpts_dev * get_wpts_dev_by_connhdl(uint16_t connhdl)
{
    int i;

    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        if (app_ptu_env.wpts_devices[i].used &&
                app_ptu_env.wpts_devices[i].pub_data.ble_conn_handle == connhdl)
        {
            return & (app_ptu_env.wpts_devices[i]);
        }
    }

    return NULL;
}

static wpts_dev * get_wpts_dev_by_tab_timer_id(ke_msg_id_t timer_id)
{
    int i = (timer_id - APP_PTU_TIMER_DYNAMIC_PARAM_0);

    if (i < PTU_IMPL_MAX_SUPPORTED_DEV && app_ptu_env.wpts_devices[i].used)
        return & (app_ptu_env.wpts_devices[i]);

    return NULL;
}

static void handle_registration_error(wpts_dev *wdev, uint8_t err)
{
    app_ptu_env.reg_counter++;

    // 5.2.9.10 => going to 'Latching fault' when 3 times initialization failed
    if (app_ptu_env.reg_counter >= 3)
    {
        app_ptu_send_ext_error_ntf(APP_PTU_ERR_3RD_CONSECUTIVE_REG_ERROR, wdev->pub_data.ble_conn_handle);
        app_ptu_proceed_to_latching_fault();
        return;
    }

    app_ptu_send_ext_error_ntf(err, wdev->pub_data.ble_conn_handle);

    // NOTE if we want to disconnect with device, then only we have to set
    // ptu_dev_state to DEV_PTU_DISCONNECTING. APP_PTU will handle that
    // and then will proceed disconnect
    if (ke_state_get(TASK_APP_PTU) == APP_PTU_LOW_POWER_STATE)
    {
        //arch_printf("<Reg error in APP_PTU_LOW_POWER > ");

        wdev->pub_data.ptu_dev_state = DEV_PRU_DISCONNECTING;
        app_ptu_state_set_with_notify(APP_PTU_POWER_SAVE_STATE);

        // repeating whole initialization process when device is still connected
    } else if (ke_state_get(TASK_APP_PTU) == APP_PTU_POWER_TRANSFER_STATE)
    {
        //arch_printf("<Reg error in POWER TRANSFER > ");
        wdev->pub_data.ptu_dev_state = DEV_PRU_DISCONNECTING;
    }

    app_ptu_state_handler(wdev);
}

static bool is_bdaddr_zeroes(struct bd_addr *dev_addr)
{
    int i;

    for (i = 0; i < BD_ADDR_LEN; ++i)
    {
        if (dev_addr->addr[i] != 0)
            return false;
    }

    return true;
}

static void handle_mode_transition(wpts_dev *wdev, uint8_t transition_mode)
{
    uint16_t timer_val;
    switch (transition_mode)
    {
        case APP_PTU_ERR_ALERT_MODE_TRANSITION_2s:
            timer_val = PTU_MODE_TRANSITION_2S_TIMER_VAL;
            break;
        case APP_PTU_ERR_ALERT_MODE_TRANSITION_3s:
            timer_val = PTU_MODE_TRANSITION_3S_TIMER_VAL;
            break;
        case APP_PTU_ERR_ALERT_MODE_TRANSITION_6s:
            timer_val = PTU_MODE_TRANSITION_6S_TIMER_VAL;
            break;
        default:
            return;
    }

    if (!is_bdaddr_zeroes(&(wdev->reconnect_addr)))
    {
        ke_timer_set(APP_PTU_TIMER_MODE_TRANSITION_ADDR, TASK_APP, timer_val);
        wdev->mode_transition_or_reconnect_ongoing = true;
    }
    else
    {
        // for zeroes address doing nothing more than setting timer, normal
        // disconnect device handling
        // will process scenario properly
        ke_timer_set(APP_PTU_TIMER_MODE_TRANSITION_EMPTY, TASK_APP, timer_val);
    }

}

static float get_power_draw(uint8_t p_rect_max, uint8_t shared_mode_val)
{
    float pru_power = ((float)p_rect_max * 0.1f); // MAX possible power in Watts
    switch(shared_mode_val)
        {
        case WPTS_CTRL_ADJ_POW_66:
            pru_power = 0.66f * pru_power;
            break;
        case WPTS_CTRL_ADJ_POW_33:
            pru_power = 0.33f * pru_power;
            break;
        case WPTS_CTRL_ADJ_POW_2_5_W:
            pru_power = 2.5f;
            break;
        case WPTS_CTRL_ADJ_POW_MAX:
        default:
            break; //, MAX power is used
        }

    return pru_power;
}

static float get_pru_power_draw(wpts_dev *wdev)
{
    return get_power_draw(wdev->pub_data.pru_static_param.p_rect_max, wdev->shared_mode_val);
}

bool app_ptu_is_enough_power_for_all_devices(void)
{
    int i;
    float ptu_power_avail;

    if (app_ptu_env.ptu_stat_val.ptu_power >= APP_PTU_POWER_TAB_LEN)
        return false; //invalid PTU_STATIC_VALUE

    ptu_power_avail = app_ptu_static_power_tab[app_ptu_env.ptu_stat_val.ptu_power];
    // here we have power usage all devices for all devices that are already registered
    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        wpts_dev *wdev = &(app_ptu_env.wpts_devices[i]);
        if (wdev->used)
        {
            wdev->pub_data.curr_power_draw = get_pru_power_draw(wdev);
            ptu_power_avail -= wdev->pub_data.curr_power_draw;
        }
    }

    return (ptu_power_avail >= 0.0f);
}

static bool is_pru_shared_mode_supported(wpts_dev *wdev)
{
    return ((wdev->pub_data.pru_static_param.information & WPTS_STATIC_INFO_ADJUST_POWER_SUPPORTED) == WPTS_STATIC_INFO_ADJUST_POWER_SUPPORTED);
}

/**
 * Sets correct mode of sharing power mode in wpts_dev struct for all devices when
 * needed - it only sets value in cached data - real sending request to PRU's to limit
 * power is being done later
 *
 * @return true if not further adjustments is needed, false otherwise (eg. when
 * setting power to 66.6% was not enough then false will be returned. Later we
 * have to try with 33.3% power setting for all PRU's)
 */
static bool limit_all_pru_drain(uint8_t sharing_mode_val)
{
    int i;

    // limiting power drain 66% of original Pmax until there will be enough power
    // for all devices
    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        if (app_ptu_env.wpts_devices[i].used
                && is_pru_shared_mode_supported(&app_ptu_env.wpts_devices[i]))
        {
            app_ptu_env.wpts_devices[i].shared_mode_val = sharing_mode_val;

            if (app_ptu_is_enough_power_for_all_devices())
            {
                // no need to decrease power anymore
                return true;
            }

        }
    }

    return false;
}

//static void update_wdev_power_drain(wpts_dev *wdev)
//{
//    if (wdev->pub_data.ptu_dev_state == DEV_PRU_DYN_READ_CHARGING
//            ||wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_IN_POWER_TRANSFER_PRU_CTRL_WR
//            ||wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_IN_POWER_TRANSFERR_DYN_RD_CFM
//       )
//    {
//        wdev->pub_data.curr_power_draw = get_pru_power_draw(wdev);
//    }
//    else
//        wdev->pub_data.curr_power_draw = 0; // don't want to confuse external app, power draw should
//                                            // be visible only when real power transfer is happening
//}

// changing state AND re-counting current power draw to make sure that host
// app receives correct value
static void set_wdev_power_sharing_state(wpts_dev *wdev, uint8_t shared_mode_param)
{
    wdev->shared_mode_val = wdev->prev_shared_mode_val;

    // on state change power have to recalculate power
    // drain - otherwise Host App will receive incorrect value
    // even if power sharing state is correct on PTU app side
    wdev->pub_data.curr_power_draw = get_pru_power_draw(wdev);
}

// returns true if there is still power margin, false when definitely reached
// max during this iteration
static bool increase_all_pru_drain(uint8_t sharing_mode_val)
{
    int i;

    // increasing power load until no more power cannot be set
    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        wpts_dev *wdev = &(app_ptu_env.wpts_devices[i]);
        if (wdev->used
                && is_pru_shared_mode_supported(wdev))
        {
               uint8_t prev_shared_val = wdev->shared_mode_val;
               set_wdev_power_sharing_state(wdev, sharing_mode_val);

               if (!app_ptu_is_enough_power_for_all_devices())
               {
                   // no possibility to increase power anymore, reverting last change
                   set_wdev_power_sharing_state(wdev, prev_shared_val);

                   return false;
               }

        }
    }

    return true;
}

static wpts_dev * get_wpts_dev_by_dev_state(uint8_t state)
{
    int i;

    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        if (app_ptu_env.wpts_devices[i].used
                && app_ptu_env.wpts_devices[i].pub_data.ptu_dev_state == state)
        {
            return &(app_ptu_env.wpts_devices[i]);
        }
    }

    return NULL;
}

static void update_prev_shared_values_all_wdevs(void)
{
    int i;

    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        if (app_ptu_env.wpts_devices[i].used)
        {
            app_ptu_env.wpts_devices[i].prev_shared_mode_val = app_ptu_env.wpts_devices[i].shared_mode_val;
        }
    }
}

// Special case - possibility to optimize power using 2.5 Watts mode
static void optimize_drain_for_2_5_watt_scenario(void)
{
    int i = 0;

    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        // checking if power, that is result of to 66% or 33% of Pmax, is lower
        // than 2.5W - this is special case and maybe going into 2.5W may be more
        // optimal solution
        wpts_dev *wdev = &(app_ptu_env.wpts_devices[i]);
        if (wdev->used && is_pru_shared_mode_supported(wdev)
            && wdev->shared_mode_val != WPTS_CTRL_ADJ_POW_2_5_W
            && get_pru_power_draw(wdev) < 2.5f)
        {
            uint8_t prev_shared_val = wdev->shared_mode_val;
            wdev->shared_mode_val = WPTS_CTRL_ADJ_POW_2_5_W;

            if (!app_ptu_is_enough_power_for_all_devices())
            {
                // no possibility for 2.5W scenario, reverting last change
                set_wdev_power_sharing_state(wdev, prev_shared_val);
            }
            // if there is enough power, then leaving with 2.5W
        }
    }
}

static bool adjust_optimal_power_all_wdevs_low(void)
{
    bool power_available = false;
    // limiting power drain 66% of original Pmax until there will be enough power
    // for all devices
    power_available = limit_all_pru_drain(WPTS_CTRL_ADJ_POW_66);

    // now trying 33% of base power
    if (!power_available)
        power_available = limit_all_pru_drain(WPTS_CTRL_ADJ_POW_33);

    // and now last try, lowering power to 2.5 Watts
    if (!power_available)
           power_available = limit_all_pru_drain(WPTS_CTRL_ADJ_POW_2_5_W);

    if (power_available)
        optimize_drain_for_2_5_watt_scenario();

    return power_available;
}

static void adjust_optimal_power_all_wdevs_up(void)
{
    bool power_available = false;

    increase_all_pru_drain(WPTS_CTRL_ADJ_POW_2_5_W);

    increase_all_pru_drain(WPTS_CTRL_ADJ_POW_33);

    power_available = increase_all_pru_drain(WPTS_CTRL_ADJ_POW_66);

    if (power_available)
        increase_all_pru_drain(WPTS_CTRL_ADJ_POW_MAX);

    // optimizing for 2.5 watts - sometimes it may be larger than 33% or even 66% of pmax
    optimize_drain_for_2_5_watt_scenario();
}

static void proceed_reject_charging_state(wpts_dev *wdev, uint8_t reason)
{
    wdev->pub_data.ptu_dev_state = DEV_PRU_CTRL_ERR_REJECT_PRU_CHARGING;
    wdev->reject_reason = reason;
}

/**
 * @returns true if adjust power has been successful, false otherwise (not possible to drain lower)
 */
bool app_ptu_task_handle_power_sharing(wpts_dev *added_wdev, bool adjust_low)
{
    bool power_available = false;
    int i;

    // with power sharing mode, registration process will take much more time than
    // usual so canceling timer after successful registration
    ke_timer_clear(APP_PTU_TIMER_REGISTRATION, TASK_APP);

    // saving values for power sharing mode, that are set currently - it will be
    // used if something will go wrong and power adjusting process will be reverted
    update_prev_shared_values_all_wdevs();

    if (adjust_low) // lowering power consumption on all devs
        power_available = adjust_optimal_power_all_wdevs_low();
    else // power increase
    {
        // probably some device disconnected, so we can increase power usage by PRU's that left
        adjust_optimal_power_all_wdevs_up();
        power_available = true; // for increasing power always should be true (no device will be rejected)
                                // if power adjust was really done is determined later in this func

        // starting timer for rollback operation. If it will be raised then we won't be waiting forever
        // for adjust power confirmation from PRU that was already connected and assume that power has
        // been adjusted properly
        ke_timer_set(APP_PTU_TIMER_POWER_ADJUST_ROLLBACK_EXPIRE, TASK_APP, PTU_PRU_ADJUST_POWER_EXPIRE_TIMER_VAL);
    }

    if (power_available)
    {
        if (adjust_low)
        {
            ke_timer_set(APP_PTU_TIMER_POWER_ADJUST_EXPIRE, TASK_APP, PTU_PRU_ADJUST_POWER_EXPIRE_TIMER_VAL);

            if (added_wdev &&
                    (!is_pru_shared_mode_supported(added_wdev)
                            || added_wdev->shared_mode_val == WPTS_CTRL_ADJ_POW_MAX))
            {
                // PRU should wait for available power until every device responds for
                // power adjust
                added_wdev->pub_data.ptu_dev_state = DEV_PRU_PWADJ_PRU_CTRL_WAIT_FOR_POWER_PERM_WR;
            }
        }

        // Setting correct state for all devices that are touched by power adjustment
        for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
        {
            wpts_dev *wdev = &(app_ptu_env.wpts_devices[i]);
            if (wdev->used && wdev->shared_mode_val != wdev->prev_shared_mode_val)
            {
                if (wdev->pub_data.ptu_dev_state < DEV_PRU_DYN_READ_CHARGING)
                    wdev->pub_data.ptu_dev_state = DEV_PRU_PWDADJ_NEW_REG_WR;
                else
                    wdev->pub_data.ptu_dev_state = DEV_PRU_PWADJ_IN_POWER_TRANSFER_PRU_CTRL_WR;

                // here we need to trigger changes only for devices that are different that
                // one that is being currently processed (for this dev ptu_state_handler will
                // be called anyway) Otherwise double writing PRU Control field can happen
                if (adjust_low && added_wdev
                        && wdev->pub_data.ble_conn_handle != added_wdev->pub_data.ble_conn_handle)
                    app_ptu_state_handler(wdev);
            }
        }

        return true;
    }
    else if (adjust_low) // not enough power
    {
        // resetting previous shared mode values, since no power adjust will be processed
        for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
        {
            wpts_dev *wdev = &(app_ptu_env.wpts_devices[i]);
            if (wdev->used)
            {
                set_wdev_power_sharing_state(wdev, wdev->prev_shared_mode_val);
            }
        }

        if (added_wdev)
        {
            // not enough power, need to notify PRU that we don't have enough power
            // and disconnect
            proceed_reject_charging_state(added_wdev, WPTS_CTRL_PERM_DENIED_LIMIT_POW);

            app_ptu_send_ext_error_ntf(APP_PTU_ERR_PTU_NOT_ENOUGH_POWER_FOR_NEW_PRU,
                                        added_wdev->pub_data.ble_conn_handle);
        }

    }

    return false;// adjusting power was unsuccessfull
}

static void handle_dev_state_chgs(wpts_dev *wdev, uint8_t err)
{
    switch(wdev->pub_data.ptu_dev_state)
    {
        case DEV_PRU_IDLE:
        {

        }
        break;
        case DEV_PRU_PRUST_RD:
        {
            wdev->dev_data_changed = true;

            if (err == APP_PTU_NO_ERROR)
            {
                wdev->pub_data.ptu_dev_state = DEV_PRU_PTUST_WR;
            }
            else
            {
                app_ptu_send_ext_error_ntf(err, wdev->pub_data.ble_conn_handle);
                proceed_reject_charging_state(wdev, WPTS_CTRL_PERM_DENIED_CLASS_SUPPORT);
                //handle_registration_error(wdev, err);
            }
        }
        break;
        case DEV_PRU_PTUST_WR:
        {
            wdev->dev_data_changed = true;
            if (err == APP_PTU_NO_ERROR)
            {
               //arch_printf(">conn %dAPP_PTU_TASK  =>> DEV_PTU_DYN_RD1 \n", wdev->ble_conn_handle);
               wdev->pub_data.ptu_dev_state = DEV_PRU_DYN_RD1;
            }
            else
            {
                handle_registration_error(wdev, err);
            }
        }
        break;
        case DEV_PRU_DYN_RD1:
        {
            wdev->dev_data_changed = true;
            if (err == APP_PTU_NO_ERROR || err == APP_PTU_ERR_ALERT_CHARGE_PORT_ACTIVATED)
            {
                // NOTE: here validating data mostly based on PTU Static param, it could
                // be done on earlier stage (right after PRU static was read) but we may
                // encounter situation when power sharing was activated without need, because
                // PRU not allowed for charging after reading PTU static param etc. so better
                // to do it now when we are 100% sure that PRU allows charging

                if (!app_ptu_is_enough_power_for_all_devices())
                {
                    app_ptu_task_handle_power_sharing(wdev, true); // lowering power usage
                }
                else
                {
                    // everything is fine, can enable PRU charging
                    wdev->pub_data.ptu_dev_state = DEV_PRU_ALERT_NOTIF_REGISTER;
                }
            }
            else if (err == APP_PTU_ERR_ALERT_CHARGE_COMPLETE)
            {
                wdev->pub_data.ptu_dev_state = DEV_PRU_CTRL_STOP_CHARGING;
            }
            else
            {
                handle_registration_error(wdev, err);
            }
        }
        break;
        case DEV_PRU_PWDADJ_NEW_REG_WR:
        {
            if (err == APP_PTU_NO_ERROR)
            {
                wdev->pub_data.ptu_dev_state = DEV_PRU_PWADJ_NEW_REG_DYN_RD_CFM;
            }
            else
            {
                handle_registration_error(wdev, err);
            }
        }
        break;
        case DEV_PRU_PWADJ_NEW_REG_DYN_RD_CFM:
        {
            if (err == APP_PTU_ERR_ALERT_DYN_POWER_ADJUST_OK)
            {
                // marking that power adjust has been finished for this newly added dev
                wdev->dev_data_changed = true;
                wdev->pub_data.ptu_dev_state = DEV_PRU_PWADJ_WAIT_FOR_OTHER_PRU_POW_ADJUST;

                // if other devices are being charged and they need also power adjust
                // we need to check if they finished power adjusting. If power adjust
                // for one of OTHER devices is pending, then doing nothing now - after
                // one of other devices finishes power adjust, then we will re-check again
                // and allow for charging for all devices
                if (is_power_adjustment_finished())
                {
                    // Power adjust finished on all devices - it is OK to go into charging
                    // phase now
                    wdev->dev_data_changed = true;
                    wdev->pub_data.ptu_dev_state = DEV_PRU_CTRL_START_CHARGING;

                    // moving to POWER TRANSFER state (needed if only one device is
                    // currently connected and it needs power adjust before charging start)
                    app_ptu_state_set_with_notify(APP_PTU_POWER_TRANSFER_STATE);

                    //resetting prev shared values
                    update_prev_shared_values_all_wdevs();
                    ke_timer_clear(APP_PTU_TIMER_POWER_ADJUST_EXPIRE, TASK_APP);
										
                    // sending notification about succes of adjustment
                    app_ptu_power_share_adjustment_complete(APP_PTU_NO_ERROR); 
                }
            }
            else if (err != APP_PTU_NO_ERROR && err != APP_PTU_ERR_ALERT_CHARGE_PORT_ACTIVATED)
            {
                // for NO_ERROR and CHARGE_PORT_ACTIVATED doing nothing, staying
                // in the same state until confirmation that
                // power draw is lowered comes from PRU

                // otherwise going to registration eror
                handle_registration_error(wdev, err);
            }
        }
        break;
        case DEV_PRU_PWADJ_PRU_CTRL_WAIT_FOR_POWER_PERM_WR:
        {
            if (err == APP_PTU_NO_ERROR)
            {
                // for OK scenario going into dummy state for waiting for other
                // PRU's (otherwise PRU control would be written all the time),
                // from this state we can escape only if all other devices adjusted power correctly
                // NOTE: not need to cancel Registration Timer because it was
                // cancelled on entering to sharing power mode
                wdev->pub_data.ptu_dev_state = DEV_PRU_PWADJ_WAIT_FOR_OTHER_PRU_POW_ADJUST;
            }
            else
            {
                handle_registration_error(wdev, err);
            }
        }
        break;
        case DEV_PRU_ALERT_NOTIF_REGISTER:
        {
            wdev->dev_data_changed = true;
            if (err == APP_PTU_NO_ERROR)
            {
                //arch_printf("APP_PTU_TASK pru notif wr cfm =>> DEV_PTU_DYN_READ_CHARGING\n");
                wdev->pub_data.ptu_dev_state = DEV_PRU_CTRL_START_CHARGING;
                app_ptu_state_set_with_notify(APP_PTU_POWER_TRANSFER_STATE);
            }
            else
            {
                // NOTE: some PRU's may interpret spec too strict, and do not allow
                // for both ind+ntf to be set. In that case, trying to set notifications only
                if (wdev->ntf_ind_set_fail_before)
                    handle_registration_error(wdev, err);
                else
                {
                    wdev->ntf_ind_set_fail_before = true;
                }
            }
        }
        break;
        case DEV_PRU_CTRL_START_CHARGING:
        {
            wdev->dev_data_changed = true;
            if (err == APP_PTU_NO_ERROR)
            {

                wdev->pub_data.ptu_dev_state = DEV_PRU_DYN_READ_CHARGING;

                // canceling timer after successfull registration
                ke_timer_clear(APP_PTU_TIMER_REGISTRATION, TASK_APP);
            }
            else
            {
                handle_registration_error(wdev, err);
            }
        }
        break;
        case DEV_PRU_DYN_READ_CHARGING:
        {
            // NOTE: 'self protect' and 'charge port activated' errors do not indicate
            // that something bad happened, they practically acting as notification to user
            // eg. self protect indicates that PRU device is not charging at full power
            if (err == APP_PTU_NO_ERROR || err == APP_PTU_ERR_ALERT_CHARGE_PORT_ACTIVATED
                    || err == APP_PTU_ERR_ALERT_SELF_PROTECT)
            {
                // After successful read and positive validation PRU Dynamic
                // Previously here was called callback for applying ITX coil, but
                // this should be done on separate processor app that observes
                // PTU state changes
            }
            else if (err == APP_PTU_ERR_ALERT_MODE_TRANSITION_2s ||
                    err == APP_PTU_ERR_ALERT_MODE_TRANSITION_3s ||
                    err == APP_PTU_ERR_ALERT_MODE_TRANSITION_6s)
            {
                // TRANSITION MODE STARTING
                wdev->dev_data_changed = true;
                handle_mode_transition(wdev, err);
            }
            else if (err == APP_PTU_ERR_ALERT_CHARGE_COMPLETE)
            {
                wdev->pub_data.ptu_dev_state = DEV_PRU_CTRL_STOP_CHARGING;
                wdev->dev_data_changed = true;
            }
            else if (err == APP_PTU_ERR_ALERT_DYN_POWER_ADJUST_OK)
            {
                // NOTE, Power Adjustment err bit should be disabled by PRU after 1st read,
                // but we tolerate that behavior for better compatibility and only send warning message
                // to Host PTU app
                app_ptu_send_ext_error_ntf(APP_PTU_ERR_PRU_ADJ_PWR_RESP_BIT_WRONGLY_SET, wdev->pub_data.ble_conn_handle);
            }
            else
            {
                app_ptu_send_ext_error_ntf(err, wdev->pub_data.ble_conn_handle);

                app_ptu_proceed_to_latching_fault();
                wdev->dev_data_changed = true;
            }
        }
        break;
        case DEV_PRU_PWADJ_IN_POWER_TRANSFER_PRU_CTRL_WR:
        {
            wdev->dev_data_changed = true;
            if (err == APP_PTU_NO_ERROR)
            {
                wdev->pub_data.ptu_dev_state = DEV_PRU_PWADJ_IN_POWER_TRANSFERR_DYN_RD_CFM;
            }
            else
            {
                handle_registration_error(wdev, err);
            }
        }
        break;
        case DEV_PRU_PWADJ_IN_POWER_TRANSFERR_DYN_RD_CFM:
        {
            if (err == APP_PTU_ERR_ALERT_DYN_POWER_ADJUST_OK)
            {
                wdev->dev_data_changed = true;
                // going into dynamic read charging, because PRU CTRL with correct
                // val has been written
                wdev->pub_data.ptu_dev_state = DEV_PRU_DYN_READ_CHARGING;

                if (is_power_adjustment_finished())
                {
                    wpts_dev *dev_in_reg = get_wpts_dev_by_dev_state(DEV_PRU_PWADJ_WAIT_FOR_OTHER_PRU_POW_ADJUST);
                    if (dev_in_reg)
                    {
                        dev_in_reg->pub_data.ptu_dev_state = DEV_PRU_CTRL_START_CHARGING;
                        app_ptu_state_handler(dev_in_reg);

                        //resetting prev shared values
                        update_prev_shared_values_all_wdevs();
                        ke_timer_clear(APP_PTU_TIMER_POWER_ADJUST_EXPIRE, TASK_APP);
                    }
                    // sending notification about succes of adjustment
                    app_ptu_power_share_adjustment_complete(APP_PTU_NO_ERROR); 
                }
            }
            else if (err != APP_PTU_NO_ERROR && err != APP_PTU_ERR_ALERT_CHARGE_PORT_ACTIVATED)
            {
                // for NO_ERROR and CHARGE_PORT_ACTIVATED doing nothing, staying
                // in the same state until confirmation that
                // power draw is lowered comes from PRU
                wdev->dev_data_changed = true;

                // otherwise proceeding to latching fault
                app_ptu_send_ext_error_ntf(err, wdev->pub_data.ble_conn_handle);
                app_ptu_proceed_to_latching_fault();
            }
        }
        break;
        case DEV_PRU_CTRL_STOP_CHARGING:
        case DEV_PRU_CTRL_ERR_REJECT_PRU_CHARGING:
        {
            // always going to IDLE after charging is finished

            wdev->pub_data.ptu_dev_state = DEV_PRU_DISCONNECTING;
            wdev->dev_data_changed = true;
            // when having one device and it has just charged => POWER_SAVE,
            // 5.2.9.6, => POWER_SAVE if no PTU reported local fault
            if (app_ptu_env.conn_count == 1)
            {
                app_ptu_state_set_with_notify(APP_PTU_POWER_SAVE_STATE);
            }
            // else staying in POWER TRANSFER STATE TO CHARGE OTHER DEVICES
        }
        break;
    default:
        break;
    }

    //updating device state changes for GTL interface only when data changed
    if (wdev->dev_data_changed)
    {
        wdev->dev_data_changed = false;
        app_ptu_gtl_send_update_of_wdev_tab();
    }
}

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
int ptu_enable_cfm_handler(ke_msg_id_t msgid, struct wptc_enable_cfm *param,
        ke_task_id_t dest_id, ke_task_id_t src_id)
{
    wpts_dev *wdev;

    if (param->status == CO_ERROR_NO_ERROR)
    {
        wdev = get_wpts_dev_by_connhdl(param->conhdl);
        if (wdev)
        {

            if (ke_timer_active(APP_PTU_TIMER_MODE_TRANSITION_ADDR, TASK_APP)
                    && wdev->mode_transition_or_reconnect_ongoing)
            {
                // for transition mode going immediately to writing pru state
                ke_timer_clear(APP_PTU_TIMER_MODE_TRANSITION_ADDR, TASK_APP);
                wdev->mode_transition_or_reconnect_ongoing = false;

                wdev->pub_data.ptu_dev_state = DEV_PRU_CTRL_START_CHARGING;
            }
            else
            {
                // device registration needed
                wdev->pub_data.ptu_dev_state = DEV_PRU_PRUST_RD;
            }

        }
    }
    else
    {
        wdev =  get_wpts_dev_by_connhdl(param->conhdl);
        if (wdev)
            handle_registration_error(wdev, APP_PTU_ERR_WPTS_PROFILE_NOT_INITIALIZED);
    }

    if (wdev)
        app_ptu_reset_rssi_weak_cache(&(wdev->pub_data.dev_addr)); // resetting weak rssi cache to be double sure that no adv in the middle of connect invalidated cache

    app_ptu_state_handler(wdev);

    return (KE_MSG_CONSUMED);
}

/**
 * @pru_alert_ntf - true if alert came from notification, false if read from
 * PRU_Dynamic_Param
 */
static void handle_pru_alert(wpts_dev *wdev, bool pru_alert_ntf, uint8_t alert_val)
{
    if (alert_val & WPTS_ALERT_OVERVOLTAGE)
    {
        app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_OVERVOLTAGE, wdev->pub_data.ble_conn_handle);

        handle_dev_state_chgs(wdev, APP_PTU_ERR_ALERT_OVERVOLTAGE);
    }
    if (alert_val & WPTS_ALERT_OVERCURRENT)
    {
        app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_OVERCURRENT, wdev->pub_data.ble_conn_handle);

        handle_dev_state_chgs(wdev, APP_PTU_ERR_ALERT_OVERCURRENT);
    }
    if (alert_val & WPTS_ALERT_OVERTEMPERATURE)
    {
        app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_OVERTEMPERATURE, wdev->pub_data.ble_conn_handle);

        handle_dev_state_chgs(wdev, APP_PTU_ERR_ALERT_OVERTEMPERATURE);
    }
    if (alert_val & WPTS_ALERT_SELF_PROTECT)
    {
        // Self Protect
        // This bit, when set, indicates that the PRU is protecting itself by reducing power to its load.
        // The PTU does not need to change states as a result. The PTU may provide feedback to the user
        // via its user interface that one of the PRU’s may not be charging at full rate.
        app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_SELF_PROTECT, wdev->pub_data.ble_conn_handle);

        handle_dev_state_chgs(wdev, APP_PTU_ERR_ALERT_SELF_PROTECT);
    }
    if (alert_val & WPTS_ALERT_CHARGE_COMPLETE)
    {
        app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_CHARGE_COMPLETE, wdev->pub_data.ble_conn_handle);

        handle_dev_state_chgs(wdev, APP_PTU_ERR_ALERT_CHARGE_COMPLETE);
    }
    if (alert_val & WPTS_ALERT_WIRED_CHARGET_DETECT)
    {
        app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_WIRED_CHARGER_DETECT, wdev->pub_data.ble_conn_handle);

        handle_dev_state_chgs(wdev, APP_PTU_ERR_ALERT_WIRED_CHARGER_DETECT);
    }
    if ((alert_val & WPTS_ALERT_CHARGE_PORT) && !pru_alert_ntf) // charge port only used in pru dyn alert
    {
        // NOTE: not sending CHARGE_PORT_ACTIVATED as error condition
        // May be active all the time while charging
        //app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_CHARGE_PORT_ACTIVATED, wdev->ble_conn_handle);

        handle_dev_state_chgs(wdev, APP_PTU_ERR_ALERT_CHARGE_PORT_ACTIVATED);
    }
    if ((alert_val & WPTS_ALERT_ADJ_POWER_COMPLETE_RSP) && !pru_alert_ntf)
    {
        // NOTE: sending notification to host app only if bit is not cleared after being
        // read in hadle_dev_state_changes() func
        //app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_DYN_POWER_ADJUST_OK, wdev->ble_conn_handle);

        handle_dev_state_chgs(wdev, APP_PTU_ERR_ALERT_DYN_POWER_ADJUST_OK);
    }

    // handling mode transition values in alert sent via notification/indication
    if (pru_alert_ntf)
    {
        if ((alert_val & WPTS_ALERT_MODE_TRANSITION_6s) == WPTS_ALERT_MODE_TRANSITION_6s)
        {
            app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_MODE_TRANSITION_6s, wdev->pub_data.ble_conn_handle);

            handle_dev_state_chgs(wdev, APP_PTU_ERR_ALERT_MODE_TRANSITION_6s);
        }
        else if ((alert_val & WPTS_ALERT_MODE_TRANSITION_3s) == WPTS_ALERT_MODE_TRANSITION_3s)
        {
            app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_MODE_TRANSITION_3s, wdev->pub_data.ble_conn_handle);

            handle_dev_state_chgs(wdev, APP_PTU_ERR_ALERT_MODE_TRANSITION_3s);
        }
        else if ((alert_val & WPTS_ALERT_MODE_TRANSITION_2s) == WPTS_ALERT_MODE_TRANSITION_2s)
        {
            app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_MODE_TRANSITION_2s, wdev->pub_data.ble_conn_handle);

            handle_dev_state_chgs(wdev, APP_PTU_ERR_ALERT_MODE_TRANSITION_2s);
        }

    }

    if (alert_val == 0)
        handle_dev_state_chgs(wdev, APP_PTU_NO_ERROR); // NO_ERROR or NO_MODE_TRANSITION
}

// checking if PRU is in optimal voltage state and setting apropriate
// PTU Sub state of Power Transfer State
//static void check_ptu_optimal_voltage_sub_state(wpts_dev * wdev)
//{
//    if (wdev->pub_data.pru_dynamic_param.v_rect_min_dyn < wdev->pub_data.pru_static_param.v_rect_min_static
//            || wdev->pub_data.pru_dynamic_param.v_rect_high_dyn < wdev->pub_data.pru_static_param.v_rect_high_static
//            || wdev->pub_data.pru_dynamic_param.v_rect_set_dyn < wdev->pub_data.pru_static_param.v_rect_set)
//    {
//        // on of PRU under voltage under voltage
//    }
//}

static void handle_pru_dynamic(wpts_dev * wdev)
{

    uint8_t validate_res = app_ptu_validate_pru_dynamic(&(wdev->pub_data.pru_dynamic_param));

    // checking if PRU is in optimal voltage state and setting apropriate
    // PTU Sub state of Power Transfer State
    if (validate_res == APP_PTU_NO_ERROR && wdev->pub_data.pru_dynamic_param.pru_alert != 0)
    {
        handle_pru_alert(wdev, false, wdev->pub_data.pru_dynamic_param.pru_alert);
    }

    handle_dev_state_chgs(wdev, validate_res);
}

static void check_pru_dynamic_changed(wpts_dev *wdev, struct wptc_rd_char_rsp *param)
{
    struct pru_dynamic_value pru_dynamic_fresh;

    memcpy(&(pru_dynamic_fresh), param->info.value,param->info.length);

    if (0 == memcmp(&pru_dynamic_fresh, &wdev->pub_data.pru_dynamic_param, sizeof(struct pru_dynamic_value))
            && wdev->pub_data.ptu_dev_state > DEV_PRU_DYN_RD1) // only print dots in continuus read mode
    {
        // both the same, printing only dot to notify that app is activce
    }
    else
    {
        wdev->dev_data_changed = true;
    }
}
static void handle_rd_pru_dynamic_val(wpts_dev *wdev, struct wptc_rd_char_rsp *param)
{
    if (wdev)
    {
        check_pru_dynamic_changed(wdev, param);
        memcpy(&(wdev->pub_data.pru_dynamic_param), param->info.value,param->info.length);

        handle_pru_dynamic(wdev);
    }
}

static void handle_rd_pru_static_val(wpts_dev *wdev, struct wptc_rd_char_rsp *param)
{
    if (wdev)
    {
        memcpy(&(wdev->pub_data.pru_static_param), &(param->info.value), param->info.length);

        handle_dev_state_chgs(wdev, app_ptu_validate_pru_static(&(wdev->pub_data.pru_static_param)));
    }
}
/**
 ****************************************************************************************
 * @brief Handles Read characteristic response.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int ptu_rd_char_rsp_handler(ke_msg_id_t msgid, struct wptc_rd_char_rsp *param,
        ke_task_id_t dest_id, ke_task_id_t src_id) {

    wpts_dev *wdev = wdev = get_wpts_dev_by_connhdl(param->info.conhdl);
    if (!wdev)
        return (KE_MSG_CONSUMED);

    // NOTE: this has to be unlocked at the beginning, otherwise there will be
    // problems with handling error cases
    wdev->wptc_trans_in_prog = false;
    if (param->info.status == CO_ERROR_NO_ERROR)
    {
        // Checking also for appropriate state - if suddenly we are in not reading
        // state (may happen because of power adjust which turn us into write state)
        // then ignoring read response
        if (wdev->last_char == WTPC_RD_PRU_DYNAMIC_PARAM
                && (wdev->pub_data.ptu_dev_state == DEV_PRU_DYN_RD1
                        || wdev->pub_data.ptu_dev_state == DEV_PRU_DYN_READ_CHARGING
                        || wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_NEW_REG_DYN_RD_CFM
                        || wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_IN_POWER_TRANSFERR_DYN_RD_CFM))
        {
            handle_rd_pru_dynamic_val(wdev, param);
        }
        else if (wdev->last_char == WTPC_RD_PRU_STATIC_PARAM
                    && wdev->pub_data.ptu_dev_state == DEV_PRU_PRUST_RD)
        {
            handle_rd_pru_static_val(wdev, param);
        }
    }
    else
    {
        // read error occurred, have to go to latching fault
        handle_dev_state_chgs(wdev, APP_PTU_ERR_READ_FAILED);
    }

    wdev->last_char = 0xFF;

    app_ptu_state_handler(wdev);
    return (KE_MSG_CONSUMED);
}

static void handle_wr_ptu_static_val(wpts_dev *wdev, struct wptc_wr_char_rsp *param)
{
    handle_dev_state_chgs(wdev,
             param->status == CO_ERROR_NO_ERROR ? APP_PTU_NO_ERROR : APP_PTU_ERR_WRITE_FAILED);
}

static void handle_wr_pru_control_val(wpts_dev *wdev, struct wptc_wr_char_rsp *param)
{
    handle_dev_state_chgs(wdev,
                 param->status == CO_ERROR_NO_ERROR ? APP_PTU_NO_ERROR : APP_PTU_ERR_WRITE_FAILED);
}

static void handle_wr_enable_notif_val(wpts_dev *wdev, struct wptc_wr_char_rsp *param)
{
    handle_dev_state_chgs(wdev,
                 param->status == CO_ERROR_NO_ERROR ? APP_PTU_NO_ERROR : APP_PTU_ERR_WRITE_FAILED);
}
/**
 ****************************************************************************************
 * @brief Handles write characteristic response.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int ptu_wr_char_rsp_handler(ke_msg_id_t msgid, struct wptc_wr_char_rsp *param,
        ke_task_id_t dest_id, ke_task_id_t src_id) {

    wpts_dev *wdev = wdev = get_wpts_dev_by_connhdl(param->conhdl);
    if (!wdev)
        return (KE_MSG_CONSUMED);

    if (param->status == CO_ERROR_NO_ERROR)
    {
        if (wdev->last_char == WPTC_WR_PTU_STATIC_PARAM)
            handle_wr_ptu_static_val(wdev, param);
        else if (wdev->last_char == WPTC_WR_PRU_CONTROL_PARAM)
            handle_wr_pru_control_val(wdev, param);
        else if (wdev->last_char == WPTC_WR_ENABLE_NOTIFICATIONS)
            handle_wr_enable_notif_val(wdev, param);
    }
    else
    {
        // write error occurred, have to go to latching fault
        handle_dev_state_chgs(wdev, APP_PTU_ERR_WRITE_FAILED);
    }

    wdev->wptc_trans_in_prog = false;
    wdev->last_char = 0xff;

    app_ptu_state_handler(wdev);
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles write characteristic response.
 *ptu_alert_ntf_handler(WPTC_PRU_ALERT_REPORT_NTF, (struct wptc_alert_cfg_indntf_req *)
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int handle_alert_ntf_handler(ke_msg_id_t msgid,
        struct wptc_alert_report_ntf *param, ke_task_id_t dest_id,
        ke_task_id_t src_id)
{

    int i;
    wpts_dev *wdev = get_wpts_dev_by_connhdl(param->conhdl);
    if (!wdev)
        return (KE_MSG_CONSUMED);

    handle_pru_alert(wdev, true, param->alert_val);
    wdev->dev_data_changed = true;
    // now have to copy address in reverse order, to be inline with every
    // address format
    for (i = 0; i < BD_ADDR_LEN; ++i)
    {
        wdev->reconnect_addr.addr[BD_ADDR_LEN - 1 - i] = param->device_address[i];
    }

    app_ptu_state_handler(wdev);
    return (KE_MSG_CONSUMED);
}

/// REQUESTS

static void do_handle_dynamic_param_handler(ke_msg_id_t const dyn_timer_id)
{
    //arch_printf("APP_PTU_TASK Timer raised! \n");
    wpts_dev *wdev = get_wpts_dev_by_tab_timer_id(dyn_timer_id);

    if (ke_state_get(TASK_APP_PTU) != APP_PTU_POWER_TRANSFER_STATE
            && ke_state_get(TASK_APP_PTU) != APP_PTU_LOW_POWER_STATE)
        // when only one device needs charging power adjust, then dynamic read is
        // being done in Low Power, so have to handle that as well
    {
        app_ptu_state_handler(wdev);
        return;
    }

    if (wdev &&
            (wdev->pub_data.ptu_dev_state == DEV_PRU_DYN_READ_CHARGING
                    || wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_NEW_REG_DYN_RD_CFM
                    || wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_IN_POWER_TRANSFERR_DYN_RD_CFM))
    {
        if (!wdev->wptc_trans_in_prog)
        {
            app_ptu_read_pru_dynamic_param(wdev);
        }

        ke_timer_set(dyn_timer_id, TASK_APP, PTU_DYNAMIC_TIMER_VAL);
    }

    app_ptu_state_handler(wdev);
}




/**
 ****************************************************************************************
 * @brief Handles PTU dynamic parameter timer for connection with idx '0'
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_ptu_timer_dynamic_param_0_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    do_handle_dynamic_param_handler(APP_PTU_TIMER_DYNAMIC_PARAM_0);


    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles PTU dynamic parameter timer for connection with idx '1'
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_ptu_timer_dynamic_param_1_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    do_handle_dynamic_param_handler(APP_PTU_TIMER_DYNAMIC_PARAM_1);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles PTU dynamic parameter timer for connection with idx '2'
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_ptu_timer_dynamic_param_2_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    do_handle_dynamic_param_handler(APP_PTU_TIMER_DYNAMIC_PARAM_2);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles PTU dynamic parameter timer for connection with idx '0'
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_ptu_timer_dynamic_param_3_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    do_handle_dynamic_param_handler(APP_PTU_TIMER_DYNAMIC_PARAM_3);

    return (KE_MSG_CONSUMED);
}


int app_ptu_timer_registration_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    ke_state_t state = ke_state_get(TASK_APP_PTU);
    wpts_dev *wdev;

    if (state == APP_PTU_LOW_POWER_STATE || state == APP_PTU_POWER_TRANSFER_STATE)
    {
        wdev = get_dev_in_registration_phase();
        if (wdev)
            handle_registration_error(wdev, APP_PTU_ERR_REGISTRATION_TIMER_RAISED);
    }

    return (KE_MSG_CONSUMED);
}

int app_ptu_timer_ptxin_expired_handler(ke_msg_id_t const msgid,
                                        void const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    app_ptu_env.ptxin_variation_detect = false;

    return (KE_MSG_CONSUMED);
}

int app_ptu_timer_power_adjust_expire_handler(ke_msg_id_t const msgid,
                                        void const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    int i;
    wpts_dev *wdev;

    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        wdev = &(app_ptu_env.wpts_devices[i]);
        if (wdev->used)
        {
            // PRU in registration phase in the middle of power adjust
            if (wdev->pub_data.ptu_dev_state == DEV_PRU_PWDADJ_NEW_REG_WR
                    || wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_NEW_REG_DYN_RD_CFM
                    || wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_PRU_CTRL_WAIT_FOR_POWER_PERM_WR
                    || wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_WAIT_FOR_OTHER_PRU_POW_ADJUST)
            {
                // rejecting charging because PRU could not adjust its power to lower value in 1 second
                proceed_reject_charging_state(wdev, WPTS_CTRL_PERM_DENIED_LIMIT_POW);
                
                // sending notification about timeout 
                app_ptu_power_share_adjustment_complete(APP_PTU_ERR_POWER_SHARING_ADJUST_TIMEOUT); 
            }
            // any other PRU that we made power adjustment, state doesn't really matter (may be in dynamic read after power adjust)
            else if (wdev->shared_mode_val != wdev->prev_shared_mode_val)
            {
                // doing roll-back of power adjustment change. NOTE: PRU also
                // has to accept change back to previous power value
                {
                    set_wdev_power_sharing_state(wdev, wdev->prev_shared_mode_val);

                    wdev->pub_data.ptu_dev_state = DEV_PRU_PWADJ_IN_POWER_TRANSFER_PRU_CTRL_WR;
                }
            }

            app_ptu_state_handler(wdev);
        }
    }

    return (KE_MSG_CONSUMED);
}

int app_ptu_timer_power_adjust_rollback_expire_handler(ke_msg_id_t const msgid,
                                                        void const *param,
                                                        ke_task_id_t const dest_id,
                                                        ke_task_id_t const src_id)
{
    int i;

    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        wpts_dev *wdev = &(app_ptu_env.wpts_devices[i]);
        if (wdev->used && wdev->pub_data.ptu_dev_state == DEV_PRU_PWADJ_IN_POWER_TRANSFERR_DYN_RD_CFM)
        {
            // rejecting charging because PRU could not adjust its power to lower value in 1 second
            wdev->pub_data.ptu_dev_state = DEV_PRU_DYN_READ_CHARGING;
            wdev->dev_data_changed = true; // to be sure that host app will be notified asap
            app_ptu_state_handler(wdev);
        }
    }

    return (KE_MSG_CONSUMED);
}

int app_ptu_timer_self_latching_clear_handler(ke_msg_id_t const msgid,
                                        void const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    // after Self Latching Clear timer expires, we have proof that for long time there weren't
    // any serious errors so it is safe to clear latching fault error counter

    app_ptu_env.self_clear_counter = 0;

    return (KE_MSG_CONSUMED);
}

int app_ptu_timer_rssi_weak_adv_wait_handler(ke_msg_id_t const msgid,
                                        void const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    // after "rssi weak adv timer" expires, we are marking timer stored address
    // as valid
    app_ptu_env.rssi_weak_cache.valid_timeout = true;

    return (KE_MSG_CONSUMED);
}

int app_ptu_mode_transition_empty_timer_handler(ke_msg_id_t const msgid,
                                                void const *param,
                                                ke_task_id_t const dest_id,
                                                ke_task_id_t const src_id)
{
    // NOTE: passing 0xff as param to notify that connhdl is non-existent
    app_ptu_send_ext_error_ntf(APP_PTU_ERR_ALERT_RECONNECT_TIMER_EXPIRED, GAP_INVALID_CONHDL);

    //when no registration has been given during desired time, then going into
    // latching fault
    app_ptu_proceed_to_latching_fault();

    return (KE_MSG_CONSUMED);
}

int app_ptu_mode_transition_addr_timer_handler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    int i;

    for (i = 0; i < PTU_IMPL_MAX_SUPPORTED_DEV; ++i)
    {
        if (app_ptu_env.wpts_devices[i].used &&
                app_ptu_env.wpts_devices[i].mode_transition_or_reconnect_ongoing)
        {
            app_ptu_internal_clear_wdev_data(&(app_ptu_env.wpts_devices[i]));
            break;
        }
    }

    // currently same handling when both timer expire
    return app_ptu_mode_transition_empty_timer_handler(msgid, param, dest_id, src_id);
}

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler app_ptu_default_handler;// = KE_STATE_HANDLER(app_ptu_default_state);
/// Defines the place holder for the states of all the task instances.
ke_state_t app_ptu_state[APP_PTU_IDX_MAX] __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

#endif // (BLE_APP_PTU)
