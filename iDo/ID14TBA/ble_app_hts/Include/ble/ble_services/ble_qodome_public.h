/*
 * Qodome public BLE interface
 */
#ifndef __QODOME_PUBLIC__
#define __QODOME_PUBLIC__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define QODOME_PUBLIC_SERVICE               0xBEEF
#define QODOME_PUBLIC_SET_NAME              0xBE01

// Forward declaration of the ble_qodome_t type.
typedef struct ble_qodome_s ble_qodome_t;

/**@brief Qodome Service event handler type. */
typedef void (*ble_qodome_evt_handler_t) (ble_qodome_t * p_qodome, ble_evt_t * p_ble_evt);

/**@brief Qodome Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
	ble_qodome_evt_handler_t       evt_handler;
} ble_qodome_init_t;

/**@brief Qodome Service structure. This contains various status information for
 *        the service. */
typedef struct ble_qodome_s
{
    ble_qodome_evt_handler_t    evt_handler;                               /**< Event handler to be called for handling events in the Qodome Service. */
    uint16_t                     service_handle;                            /**< Handle of Qodome Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     qodome_handles;                              /**< Handles related to the Qodome value characteristic. */
    uint16_t                     conn_handle;                              /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
} ble_qodome_t;

/**@brief Qodome Service result data structure. This contains measurement result */
typedef struct ble_qodome_v_s
{
	uint32_t	addr;									/* Qodome address */
    uint8_t  	v[16];                                 	/**< Qodome value */
} __attribute__((packed)) ble_qodome_v_t;

/**@brief Function for initializing the Qodome Service.
 *
 * @param[out]  p_qodome       Qodome Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_qodome_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_qodome_init(void);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Qodome Service.
 *
 * @param[in]   p_qodome      Qodome Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_qodome_on_ble_evt(ble_qodome_t * p_qodome, ble_evt_t * p_ble_evt);

ble_qodome_t get_ble_qodome(void);
uint8_t qodome_flag_get(void);

#endif /* Qodome_H_ */
