/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

/** @file
 *
 * @defgroup ble_sdk_srv_hts Health Thermometer Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Health Thermometer Service module.
 *
 * @details This module implements the Health Thermometer Service.
 *
 *          If an event handler is supplied by the application, the Health Thermometer 
 *          Service will generate Health Thermometer Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Health Thermometer Service
 *       module by calling ble_hts_on_ble_evt() from the from the @ref ble_stack_handler function.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_HTS_H__
#define BLE_HTS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_date_time.h"


// Temperature Type measurement locations
#define BLE_HTS_TEMP_TYPE_ARMPIT      1
#define BLE_HTS_TEMP_TYPE_BODY        2
#define BLE_HTS_TEMP_TYPE_EAR         3
#define BLE_HTS_TEMP_TYPE_FINGER      4
#define BLE_HTS_TEMP_TYPE_GI_TRACT    5
#define BLE_HTS_TEMP_TYPE_MOUTH       6
#define BLE_HTS_TEMP_TYPE_RECTUM      7
#define BLE_HTS_TEMP_TYPE_TOE         8
#define BLE_HTS_TEMP_TYPE_EAR_DRUM    9

/**@brief Health Thermometer Service event type. */
typedef enum
{
    BLE_HTS_EVT_INDICATION_ENABLED,                                         /**< Health Thermometer value indication enabled event. */
    BLE_HTS_EVT_INDICATION_DISABLED,                                        /**< Health Thermometer value indication disabled event. */
    BLE_HTS_EVT_INDICATION_CONFIRMED,                                        /**< Confirmation of a temperature measurement indication has been received. */
    BLE_HTS_EVT_NOTIFICATION_ENABLED,                                         /**< Health Thermometer value notification enabled event. */
    BLE_HTS_EVT_NOTIFICATION_DISABLED,                                        /**< Health Thermometer value notification disabled event. */
} ble_hts_evt_type_t;

/**@brief Health Thermometer Service event. */
typedef struct
{
    ble_hts_evt_type_t evt_type;                                            /**< Type of event. */
} ble_hts_evt_t;

// Forward declaration of the ble_hts_t type. 
typedef struct ble_hts_s ble_hts_t;

/**@brief Health Thermometer Service event handler type. */
typedef void (*ble_hts_evt_handler_t) (ble_hts_t * p_hts, ble_hts_evt_t * p_evt);

/**@brief FLOAT format (IEEE-11073 32-bit FLOAT, defined as a 32-bit value with a 24-bit mantissa
 *        and an 8-bit exponent. */
typedef struct
{
	int8_t  exponent;                                                         /**< Base 10 exponent */
	int32_t mantissa;                                                         /**< Mantissa, should be using only 24 bits */
} ieee_float32_t;

/**@brief Health Thermometer Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
    ble_hts_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the Health Thermometer Service. */
} ble_hts_init_t;

/**@brief Health Thermometer Service structure. This contains various status information for
 *        the service. */
typedef struct ble_hts_s
{
    ble_hts_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the Health Thermometer Service. */
    uint16_t                     conn_handle;                               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint16_t                     service_handle;                            /**< Handle of Health Thermometer Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     meas_handle;                              /**< Handles related to the Health Thermometer Measurement characteristic. */
    ble_gatts_char_handles_t     intermediate_handle;                      /**< Handles related to the Health Thermometer Measurement characteristic. */
    ble_gatts_char_handles_t	 meas_interval_handle;
    ble_gatts_char_handles_t	 interval_range_handle;
} ble_hts_t;

/**@brief Health Thermometer Service measurement structure. This contains a Health Thermometer
 *        measurement. */
typedef struct ble_hts_meas_s
{
    bool                         temp_in_fahr_units;                        /**< True if Temperature is in Fahrenheit units, Celcius otherwise. */
    bool                         time_stamp_present;                        /**< True if Time Stamp is present. */
    bool                         temp_type_present;                         /**< True if Temperature Type is present. */
    ieee_float32_t               temp_in_celcius;                           /**< Temperature Measurement Value (Celcius). */
    ieee_float32_t               temp_in_fahr;                              /**< Temperature Measurement Value (Fahrenheit). */
    ble_date_time_t              time_stamp;                                /**< Time Stamp. */
    uint8_t                      temp_type;                                 /**< Temperature Type. */
} ble_hts_meas_t;

/**@brief Function for initializing the Health Thermometer Service.
 *
 * @param[out]  p_hts       Health Thermometer Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_hts_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_hts_init(ble_hts_t * p_hts, const ble_hts_init_t * p_hts_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Health Thermometer Service.
 *
 * @param[in]   p_hts      Health Thermometer Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_hts_on_ble_evt(ble_hts_t * p_hts, ble_evt_t * p_ble_evt);

// Send out Intermediate Temperature (notify)
uint32_t ble_hts_send_it(ble_hts_t * p_hts, ble_hts_meas_t * p_hts_meas);

// Send out Temperature Measurement (indication)
uint32_t ble_hts_send_tm(ble_hts_t * p_hts, ble_hts_meas_t * p_hts_meas);

// Fill in HTS report data
uint8_t hts_measurement_encode(ble_hts_meas_t *p_hts_meas, uint8_t *p_encoded_buffer);

#endif // BLE_HTS_H__

/** @} */
