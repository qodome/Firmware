/*
 * ble_register_rw.h
 *
 *  Created on: 2014-10-28
 *      Author: Administrator
 */


#ifndef BLE_REGISTER_RW_H_
#define BLE_REGISTER_RW_H_



#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_date_time.h"


/**@brief Health Thermometer Service event type. */
typedef enum
{
  TEST
} ble_reg_evt_type_t;

/**@brief Health Thermometer Service event. */
typedef struct
{
    ble_reg_evt_type_t evt_type;                                            /**< Type of event. */
} ble_reg_evt_t;

// Forward declaration of the ble_reg_t type.
typedef struct ble_reg_s ble_reg_t;

/**@brief Health Thermometer Service event handler type. */
typedef void (*ble_reg_evt_handler_t) (ble_reg_t * p_reg, ble_reg_evt_t * p_evt);

/**@brief FLOAT format (IEEE-11073 32-bit FLOAT, defined as a 32-bit value with a 24-bit mantissa
 *        and an 8-bit exponent. */


/**@brief Health Thermometer Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
    ble_reg_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the Health Thermometer Service. */

    ble_srv_security_mode_t      reg_type_attr_md;                     /**< Initial security level for health thermometer tempearture type attribute */
 } ble_reg_init_t;

/**@brief Health Thermometer Service structure. This contains various status information for
 *        the service. */
typedef struct ble_reg_s
{
    ble_reg_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the Health Thermometer Service. */
    uint16_t                     service_handle;                            /**< Handle of Health Thermometer Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     reg_handles;                              /**< Handles related to the Health Thermometer Measurement characteristic. */
    uint16_t                     conn_handle;                               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */

} ble_reg_t;

/**@brief Health Thermometer Service measurement structure. This contains a Health Thermometer
 *        measurement. */
typedef struct ble_reg_value_s
{
    uint16_t addr;
    uint16_t value;
} ble_reg_value_t;

/**@brief Function for initializing the Health Thermometer Service.
 *
 * @param[out]  p_reg       Health Thermometer Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_reg_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_reg_init(ble_reg_t * p_reg, const ble_reg_init_t * p_reg_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Health Thermometer Service.
 *
 * @param[in]   p_reg      Health Thermometer Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_reg_on_ble_evt(ble_reg_t * p_reg, ble_evt_t * p_ble_evt);

extern uint16_t spi_read (uint8_t addr);

extern void spi_write (uint8_t addr, uint16_t cmd);


#endif /* BLE_REGISTER_RW_H_ */

/** @} */


