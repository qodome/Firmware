/*
 * led.h
 *
 *  Created on: 2014-11-3
 *      Author: Administrator
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_IQO_LED_SERVICE  	0x2014
#define BLE_UUID_IQO_LED_CHAR  		0x1212

// Forward declaration of the ble_led_t type.
typedef struct ble_led_s ble_led_t;

/**@brief LED Service event handler type. */
typedef void (*ble_led_evt_handler_t) (ble_led_t * p_led, ble_evt_t * p_ble_evt);

/**@brief LED Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
    ble_srv_security_mode_t 	 	led_attr_md;                          /**< Initial security level for LED value attribute */
} ble_led_init_t;

/**@brief LED Service structure. This contains various status information for
 *        the service. */
typedef struct ble_led_s
{
    ble_led_evt_handler_t    evt_handler;                               /**< Event handler to be called for handling events in the LED Service. */
    uint16_t                     service_handle;                            /**< Handle of LED Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     led_handles;                              /**< Handles related to the LED value characteristic. */
    uint16_t                     conn_handle;                              /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
} ble_led_t;

/**@brief LED Service result data structure. This contains measurement result */
typedef struct ble_led_v_s
{
	uint8_t	led_pwm[4];									/* LED address */
} __attribute__((packed)) ble_led_v_t;

/**@brief Function for initializing the LED Service.
 *
 * @param[out]  p_led       LED Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_led_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_led_init(ble_led_t * p_led);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the LED Service.
 *
 * @param[in]   p_led      LED Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_led_on_ble_evt(ble_led_t * p_led, ble_evt_t * p_ble_evt);

#endif /* LED_H_ */
