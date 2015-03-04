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

#ifndef APP_PTU_H_
#define APP_PTU_H_

#include "co_bt.h"
#include "smpc_task.h"
#include "disc.h"

#include "wpt_common.h"
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>            // standard integer

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

#define PTU_IMPL_MAX_SUPPORTED_DEV (4) //4 devices at max, this should not be larger than BLE_CONNECTION_MAX_USER value from da14580_config
#define PTU_MAX_SELF_CLEAR_COUNTER (3) // 3 times can try to self-clear latching fault before user interaction is needed
#define PTU_REGISTRATION_TIMER_VAL (50)
#define PTU_DYNAMIC_TIMER_VAL (25) // 250 miliseconds
#define PTU_MODE_TRANSITION_RECONN_1_1S_TIMER_VAL (110) // RECONNNECTION TIMER AFTER LINK LOSS(1.1s)
#define PTU_MODE_TRANSITION_2S_TIMER_VAL (200)
#define PTU_MODE_TRANSITION_3S_TIMER_VAL (300)
#define PTU_MODE_TRANSITION_6S_TIMER_VAL (600)
#define PTU_RSSI_WEAK_VALID_TIMER_VAL    (170) // 1.7 second, time that is needed to accept weak rssi signal
#define PTU_PTXIN_VARIATION_EXPIRE_TIMER_VAL (200) // 2 seconds -received Ptx_in variation is valid until this timeout
#define PTU_PRU_ADJUST_POWER_EXPIRE_TIMER_VAL (1800) //  //TODO: for ease of testing currently 8 secs, should be 1
#define PTU_SELF_LATCHING_CLEAR_TIMER_VAL (3000) // 5 minutes
#define PTU_CATEGORY_PRU_MAX_SUPPORT (WPTC_CLASS_6) // maximum value of Category for PRU that is supported by PTU (5)

/// Used for local fault error that can be reported to PTU from Host App
enum {
    APP_PTU_LOCAL_ERROR_OVER_VOLTAGE = 0,
    APP_PTU_LOCAL_ERROR_OVER_TEMP,
    APP_PTU_LOCAL_ERROR_OVER_CURRENT,
    APP_PTU_LOCAL_ERROR_UNKNOWN
};

//Error messages
enum {
    APP_PTU_NO_ERROR = 0,
    APP_PTU_ERR_WRITE_FAILED,
    APP_PTU_ERR_READ_FAILED,
    APP_PTU_ERR_DYN_READ_VALIDATION_RFU_FAILED, // Dynamic Parameter received from PRU was wrong (probably one of RFU fields used)
    APP_PTU_ERR_ALERT_OVERVOLTAGE,
    APP_PTU_ERR_ALERT_OVERCURRENT,
    APP_PTU_ERR_ALERT_OVERTEMPERATURE,
    APP_PTU_ERR_ALERT_SELF_PROTECT,
    APP_PTU_ERR_ALERT_CHARGE_COMPLETE,
    APP_PTU_ERR_ALERT_WIRED_CHARGER_DETECT,
    APP_PTU_ERR_ALERT_CHARGE_PORT_ACTIVATED, // no error condition, only notification that PRU allows charge
    APP_PTU_ERR_ALERT_DYN_POWER_ADJUST_OK, // no error condition, limiting power operation eneded with success on PRU side
    APP_PTU_ERR_ALERT_MODE_TRANSITION_0s,
    APP_PTU_ERR_ALERT_MODE_TRANSITION_2s,
    APP_PTU_ERR_ALERT_MODE_TRANSITION_3s,
    APP_PTU_ERR_ALERT_MODE_TRANSITION_6s,
    APP_PTU_ERR_ALERT_RECONNECT_TIMER_EXPIRED, //used for mode transition and basic reconnect
    APP_PTU_ERR_REGISTRATION_TIMER_RAISED,
    APP_PTU_ERR_WPTS_PROFILE_NOT_INITIALIZED,
    APP_PTU_ERR_LINK_LOSS,
    APP_PTU_ERR_PRU_STAT_VALIDATION_RFU_FAILED, // PRU static validation failed - one of RFU fields used
    APP_PTU_ERR_PRU_STAT_VALIDATION_CAT_FAILED, // PRU static validation failed - category of PRU is too high
    APP_PTU_ERR_TIME_SET_CC_CHECK_FAILED, // time set cross connection check failed
    APP_PTU_ERR_PTU_STATIC_WRONG_NUM_DEVICES_VAL, // not enough devices as number of devices set (there is always min value for each PTU class)
    APP_PTU_ERR_PTU_STATIC_WRONG_PTU_CLASS, // when PTU class is set to wrong value
    APP_PTU_ERR_PTU_STATIC_NUM_DEV_EXCEED_CONN_NUM, // when number of devices exceeded number of possible connections on PTU side
    APP_PTU_ERR_PTU_STATIC_NOT_ALLOWED_CONN_DEV, // not allowed to set PTU static when there is some device connected already
    APP_PTU_ERR_PRU_REMOVED_FROM_CHARGE_AREA, // when PTU class is set to wrong value
    APP_PTU_ERR_PTU_NOT_ENOUGH_POWER_FOR_NEW_PRU, // when there is not enough energy to charge newly added PRU
    APP_PTU_ERR_PRU_CATEGORY_TO_HIGH, // when there is not enough energy to charge newly added PRU
    APP_PTU_ERR_PRU_ADJ_PWR_RESP_BIT_WRONGLY_SET, // adjust power response bit should be cleared by PRU in dynamic param after successfull read
    APP_PTU_ERR_3RD_CONSECUTIVE_REG_ERROR, // 3rd consecutive registration error occured, going into latching fault
    APP_PTU_ERR_POWER_SHARING_ADJUST_TIMEOUT, // when power sharing fails couse of timeout
};

enum {
    APP_PTU_ADV_IMPEDANCE_SHIFT_NEVER = 0,
    APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_1,
    APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_2,
    APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_3,
    APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_4,
    APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_5,
    APP_PTU_ADV_IMPEDANCE_SHIFT_CAT_RFU
};

// Wireless power transfer service connected device
struct wpts_pub_data
{
    struct pru_dynamic_value pru_dynamic_param;
    struct pru_static_value pru_static_param;
    struct bd_addr dev_addr;
    float curr_power_draw; // value in Watts
    uint16_t ble_conn_handle;
    uint8_t ptu_dev_state;
    /// Parsed values from advertising packet payload, described in A4WP Spec 6.5.1
    uint8_t imp_shift_category_adv;// PRU Category Impedance shift, taken from PRU advertising packet
    uint8_t pru_pwr_adv_val; /// Value of PRU PWR taken from PRU advertising packet  NOTE: to get real value, (-20dBm) should be added to this. 0xF8 val means 'unknown'
    uint8_t pru_gain_adv_val; /// Value of PRU Gain taken from PRU advertising packet  NOTE: to get real value, (-5dBi) should be added to this. 0x07 val means 'unknown'
    bool time_set_supported_adv;// true if time set supported, taken from PRU advertising packet
    bool reboot_with_no_reset_adv; // true if reboot with no reset supported, taken from PRU advertising packet
    bool ovp_status_supported; // true if OVP status is supported, taken from PRU advertising packet
};


// data used only by PTU FH app, not needed for external app
typedef struct
{
    struct wpts_pub_data pub_data;
    bool used;// true if device struct is currently used
    uint8_t pru_alert;
    bool disconnect_init_by_ptu;
    bool wptc_trans_in_prog;

    uint8_t shared_mode_val;// used to indicate which shared mode is currently active for PRU
    uint8_t ble_conn_idx;
    uint8_t prev_shared_mode_val;
    uint8_t reject_reason; // reason if disconnection has occurred
    bool ntf_ind_set_fail_before;// true if enabling both ind+ntf failed
    bool mode_transition_or_reconnect_ongoing;
    uint8_t last_char;
    ke_msg_id_t dyn_timer_id;
    bool dev_data_changed;// used for marking that device data has changed and ntf has to be sent to GTL
    struct bd_addr reconnect_addr; // used for mode transition feature
} wpts_dev;


struct ptu_rssi_weak_cache
{
    struct bd_addr dev_addr;
    bool valid_timeout;
    uint8_t count;
};

struct ptu_scan_data
{
    struct bd_addr scan_dev_addr;
    uint16_t wpt_service_begin_hdl;
    bool scan_dev_found;
    bool conn_in_progress;
    uint8_t pru_rssi;
    uint8_t adv_flags;
};

/// PTU App environment structure
struct app_ptu_env_tag
{
    struct ptu_scan_data scan_data;
    bool load_detected;
    bool was_state_latching;
    bool ptxin_variation_detect;
    struct ptu_static_value ptu_stat_val;
    struct ptu_rssi_weak_cache rssi_weak_cache;
    uint8_t time_set_default_val; /// used for PRU Control setting

    uint8_t self_clear_counter; // counter for clearing latching fault error by PTU
    uint8_t reg_counter;//counter for registration errors in a row
    wpts_dev wpts_devices[PTU_IMPL_MAX_SUPPORTED_DEV];
    uint8_t conn_count;// count of active connections
};
/*
 * DEFINES
 ****************************************************************************************
 */

/// Application environment
extern struct app_ptu_env_tag app_ptu_env;

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
void app_ptu_init(void);

void app_ptu_configuration_finished(void);

bool app_ptu_check_connecting_possible(struct bd_addr const *peer_addr, uint8_t pru_adv_flags);

void app_ptu_enable(uint16_t conhdl, uint8_t connidx, uint16_t wpt_service_start_hdl,
                    uint8_t pru_rssi, uint8_t wpt_adv,
                    struct gapc_connection_req_ind const *param);

void app_ptu_state_handler(wpts_dev *wdev);

void app_ptu_disconnect_complete(struct gapc_disconnect_ind const *disc_ind_param);

void app_ptu_gapc_disc_cmplt(uint8_t conn_idx, uint8_t status);

/**
 * sets state for APP TASK and informs TASK GTL about state change
 */
void app_ptu_state_set_with_notify(ke_state_t const state_id);

//called from app_ptu_task timer from app_ptu_task code
void app_ptu_read_pru_dynamic_param(wpts_dev *wdev);

void app_ptu_internal_clear_wdev_data(wpts_dev *wdev);

void app_ptu_proceed_to_latching_fault(void);

//EXTERNAL APP INTERFACE
void app_ptu_gtl_send_update_of_wdev_tab(void);
void app_ptu_send_ext_error_ntf(uint8_t err_ptu, uint16_t connhdl);
void app_ptu_gtl_notify_discovery_event(uint8_t discovery_event);

/// interaction with external process that controls charging
void app_ptu_load_detected_handle(void);

void app_ptu_latching_fault_cleared_handle(void);

void app_ptu_local_fault_detected_handle(uint8_t local_fault_err);

void app_ptu_local_fault_cleared_handle(void);

void app_ptu_power_share_adjustment_complete(uint8_t status);

bool app_ptu_task_handle_power_sharing(wpts_dev *added_wdev, bool adjust_low);

bool app_ptu_is_addr_in_weak_rssi_cache(struct bd_addr *pru_addr);

void app_ptu_reset_rssi_weak_cache(struct bd_addr * addr);
/// @} APP

#endif // APP_PTU_H_
