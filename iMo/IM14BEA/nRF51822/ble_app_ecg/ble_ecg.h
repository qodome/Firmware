/** @file
 *
 * @defgroup ble_sdk_srv_ecg ECG Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief ECG Service module.
 *
 * @details This module implements the ECG Service.
 *
 *          If an event handler is supplied by the application, the ECG
 *          Service will generate ECG Service events to the application.
 *
 * @note The application must propagate BLE stack events to the ECG Service
 *       module by calling ble_ecg_on_ble_evt() from the from the @ref ble_stack_handler function.
 *
 */

#ifndef BLE_ECG_H__
#define BLE_ECG_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_date_time.h"

#define BLE_UUID_ECG_SERVICE    0x0606
#define BLE_UUID_ECG_CHAR       0x0707

/**@brief ECG Service event type. */
typedef enum
{
    BLE_ECG_EVT_NOTIFY_ENABLED,                                         /**< ECG value notify enabled event. */
    BLE_ECG_EVT_NOTIFY_DISABLED,                                        /**< ECG value notify disabled event. */
} ble_ecg_evt_type_t;

/**@brief ECG Service event. */
typedef struct
{
    ble_ecg_evt_type_t evt_type;                                            /**< Type of event. */
} ble_ecg_evt_t;

// Forward declaration of the ble_ecg_t type. 
typedef struct ble_ecg_s ble_ecg_t;

/**@brief ECG Service event handler type. */
typedef void (*ble_ecg_evt_handler_t) (ble_ecg_t * p_ecg, ble_ecg_evt_t * p_evt);

/**@brief ECG Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
    ble_ecg_evt_handler_t        evt_handler;                          /**< Event handler to be called for handling events in the ECG Service. */
    ble_srv_cccd_security_mode_t ecg_attr_md;                          /**< Initial security level for ECG value attribute */
} ble_ecg_init_t;

/**@brief ECG Service structure. This contains various status information for
 *        the service. */
typedef struct ble_ecg_s
{
    ble_ecg_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the ECG Service. */
    uint16_t                     service_handle;                            /**< Handle of ECG Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     ecg_handles;                              /**< Handles related to the ECG value characteristic. */
    uint16_t                     conn_handle;                              /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
} ble_ecg_t;

/**@brief ECG Service result data structure. This contains measurement result */
typedef struct ble_ecg_v_s
{
    uint16_t  v[10];                                 /**< ECG value */
} ble_ecg_v_t;

/**@brief Function for initializing the ECG Service.
 *
 * @param[out]  p_ecg       ECG Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_ecg_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_ecg_init(ble_ecg_t * p_ecg, const ble_ecg_init_t * p_ecg_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the ECG Service.
 *
 * @param[in]   p_ecg      ECG Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_ecg_on_ble_evt(ble_ecg_t * p_ecg, ble_evt_t * p_ble_evt);

/**@brief Function for sending ECG value if notify has been enabled.
 *
 * @details The application calls this function after having performed a ECG
 *          sample. If notify has been enabled, the result data is encoded and
 *          sent to the client.
 *
 * @param[in]   p_ecg       ECG Service structure.
 * @param[in]   p_ecg_v     Pointer to new ECG value.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ecg_v_send(ble_ecg_t * p_ecg, ble_ecg_v_t * p_ecg_v);

/**@brief Function for checking if notify of ECG is currently enabled.
 *
 * @param[in]   p_ecg                  ECG Service structure.
 * @param[out]  p_notify_enabled       TRUE if notify is enabled, FALSE otherwise.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ecg_is_notify_enabled(ble_ecg_t * p_ecg, bool * p_notify_enabled);

#endif // BLE_ECG_H__
