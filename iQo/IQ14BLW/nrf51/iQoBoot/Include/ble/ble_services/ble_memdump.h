/*
 * memdump.h
 *
 *  Created on: 2014-11-3
 *      Author: Administrator
 */

#ifndef MEMDUMP_H_
#define MEMDUMP_H_

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_MEMDUMP_SERVICE    0x5555
#define BLE_UUID_MEMDUMP_CHAR       0x6666

// Forward declaration of the ble_memdump_t type.
typedef struct ble_memdump_s ble_memdump_t;

/**@brief MEMDUMP Service event handler type. */
typedef void (*ble_memdump_evt_handler_t) (ble_memdump_t * p_memdump, ble_evt_t * p_ble_evt);

/**@brief MEMDUMP Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
	ble_memdump_evt_handler_t       evt_handler;
} ble_memdump_init_t;

/**@brief MEMDUMP Service structure. This contains various status information for
 *        the service. */
typedef struct ble_memdump_s
{
    ble_memdump_evt_handler_t    evt_handler;                               /**< Event handler to be called for handling events in the MEMDUMP Service. */
    uint16_t                     service_handle;                            /**< Handle of MEMDUMP Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     memdump_handles;                              /**< Handles related to the MEMDUMP value characteristic. */
    uint16_t                     conn_handle;                              /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
} ble_memdump_t;

/**@brief MEMDUMP Service result data structure. This contains measurement result */
typedef struct ble_memdump_v_s
{
	uint32_t	addr;									/* MEMDUMP address */
    uint8_t  	v[16];                                 	/**< MEMDUMP value */
} __attribute__((packed)) ble_memdump_v_t;

/**@brief Function for initializing the MEMDUMP Service.
 *
 * @param[out]  p_memdump       MEMDUMP Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_memdump_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_memdump_init(void);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the MEMDUMP Service.
 *
 * @param[in]   p_memdump      MEMDUMP Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_memdump_on_ble_evt(ble_memdump_t * p_memdump, ble_evt_t * p_ble_evt);

ble_memdump_t get_ble_memdump(void);
uint8_t memdump_flag_get(void);

#endif /* MEMDUMP_H_ */
