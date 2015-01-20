/*
 * date_time.h
 *
 *  Created on: 2014-10-31
 *      Author: Administrator
 */

#ifndef DATE_TIME_H_
#define DATE_TIME_H_


#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_date_time.h"

/**@brief Health Thermometer Service event type. */
typedef enum
{
  TEST1
} ble_time_evt_type_t;

/**@brief Health Thermometer Service event. */
typedef struct
{
    ble_time_evt_type_t evt_type;                                            /**< Type of event. */
} ble_time_evt_t;

// Forward declaration of the ble_time_t type.
typedef struct ble_time_s ble_time_t;

/**@brief Health Thermometer Service event handler type. */
typedef void (*ble_time_evt_handler_t) (ble_time_t * p_time, ble_time_evt_t * p_evt);

/**@brief FLOAT format (IEEE-11073 32-bit FLOAT, defined as a 32-bit value with a 24-bit mantissa
 *        and an 8-bit exponent. */


/**@brief Health Thermometer Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
    ble_time_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the Health Thermometer Service. */

    ble_srv_security_mode_t      time_type_attr_md;                     /**< Initial security level for health thermometer tempearture type attribute */
 } ble_time_init_t;

/**@brief Health Thermometer Service structure. This contains various status information for
 *        the service. */
typedef struct ble_time_s
{
    ble_time_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the Health Thermometer Service. */
    uint16_t                     service_handle;                            /**< Handle of Health Thermometer Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     time_handles;                              /**< Handles related to the Health Thermometer Measurement characteristic. */
    uint16_t                     conn_handle;                               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */

} ble_time_t;

/**@brief Health Thermometer Service measurement structure. This contains a Health Thermometer
 *        measurement. */
typedef struct ble_time_value_s
{
     uint32_t  value;
} ble_time_value_t;

/**@brief Function for initializing the Health Thermometer Service.
 *
 * @param[out]  p_time       Health Thermometer Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_time_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_time_init(ble_time_t * p_time, const ble_time_init_t * p_time_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Health Thermometer Service.
 *
 * @param[in]   p_time      Health Thermometer Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_time_on_ble_evt(ble_time_t * p_time, ble_evt_t * p_ble_evt);

void date_time_init(void);

uint32_t date_time_get_relative(void);
uint32_t date_time_get_wall(void);
void date_time_set_wall(uint32_t t);

uint8_t date_time_initialized(void);

#endif /* DATE_TIME_H_ */
