/*
 * sensor.h
 *
 *  Created on: 2014-11-3
 *      Author: Administrator
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/**@brief Sensor Service event type. */
typedef enum
{
    BLE_SENSOR_EVT_NOTIFICATION_ENABLED,                                         /**< Health Thermometer value notification enabled event. */
    BLE_SENSOR_EVT_NOTIFICATION_DISABLED,                                        /**< Health Thermometer value notification disabled event. */
} ble_sensor_evt_type_t;

/**@brief Sensor Service event. */
typedef struct
{
	ble_sensor_evt_type_t evt_type;                                            /**< Type of event. */
} ble_sensor_evt_t;

// Forward declaration of the ble_sensor_t type.
typedef struct ble_sensor_s ble_sensor_t;

/**@brief SENSOR Service event handler type. */
typedef void (*ble_sensor_evt_handler_t) (ble_sensor_t * p_sensor, ble_sensor_evt_t * p_ble_evt);

/**@brief SENSOR Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
	ble_sensor_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the Health Thermometer Service. */
} ble_sensor_init_t;

/**@brief SENSOR Service structure. This contains various status information for
 *        the service. */
typedef struct ble_sensor_s
{
    ble_sensor_evt_handler_t    evt_handler;                               /**< Event handler to be calsensor for handling events in the SENSOR Service. */
    uint16_t                     service_handle;                            /**< Handle of SENSOR Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     sensor_handle;                              /**< Handles related to the SENSOR value characteristic. */
    uint16_t                     conn_handle;                              /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
} ble_sensor_t;

/**@brief SENSOR Service result data structure. This contains measurement result */
typedef struct ble_sensor_v_s
{
	uint8_t	sensor_pwm[4];									/* SENSOR address */
} __attribute__((packed)) ble_sensor_v_t;

/**@brief Function for initializing the SENSOR Service.
 *
 * @param[out]  p_sensor       SENSOR Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_sensor_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_sensor_init(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the SENSOR Service.
 *
 * @param[in]   p_sensor      SENSOR Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_sensor_on_ble_evt(ble_sensor_t * p_sensor, ble_evt_t * p_ble_evt);

uint32_t ble_sensor_send_isl(ble_sensor_t * p_sensor, uint8_t *pBuf);

#endif /* SENSOR_H_ */
