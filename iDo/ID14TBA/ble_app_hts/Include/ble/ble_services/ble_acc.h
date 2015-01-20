/*
 * acc.h
 *
 *  Created on: 2014-11-3
 *      Author: Administrator
 */

#ifndef ACC_H_
#define ACC_H_

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_ACC_SERVICE    	0x1110
#define BLE_UUID_ACC_REG_CHAR   	0x1111
#define BLE_UUID_ACC_DATA_CHAR		0x1212

/**@brief Accelerometer Service event type. */
typedef enum
{
    BLE_ACC_EVT_NOTIFICATION_ENABLED,
    BLE_ACC_EVT_NOTIFICATION_DISABLED,
} ble_acc_evt_type_t;

typedef struct
{
	ble_acc_evt_type_t evt_type;                                            /**< Type of event. */
} ble_acc_evt_t;

// Forward declaration of the ble_acc_t type.
typedef struct ble_acc_s ble_acc_t;

/**@brief ACC Service event handler type. */
typedef void (*ble_acc_evt_handler_t) (ble_acc_t * p_acc, ble_evt_t * p_ble_evt);

/**@brief ACC Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
	ble_acc_evt_handler_t       evt_handler;
} ble_acc_init_t;

/**@brief ACC Service structure. This contains various status information for
 *        the service. */
typedef struct ble_acc_s
{
    ble_acc_evt_handler_t    evt_handler;                               /**< Event handler to be called for handling events in the ACC Service. */
    uint16_t                     service_handle;                            /**< Handle of ACC Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     acc_handles;                              /**< Handles related to the ACC value characteristic. */
    ble_gatts_char_handles_t     acc_data_handle;
    uint16_t                     conn_handle;                              /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
} ble_acc_t;

/**@brief ACC Service result data structure. This contains measurement result */
typedef struct ble_acc_v_s
{
	uint8_t	addr;									/* ACC address */
    uint8_t value;                                 	/**< ACC value */
} __attribute__((packed)) ble_acc_v_t;

/**@brief Function for initializing the ACC Service.
 *
 * @param[out]  p_acc       ACC Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_acc_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_acc_init(ble_acc_t * p_acc);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the ACC Service.
 *
 * @param[in]   p_acc      ACC Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_acc_on_ble_evt(ble_acc_t * p_acc, ble_evt_t * p_ble_evt);

#endif /* ACC_H_ */
