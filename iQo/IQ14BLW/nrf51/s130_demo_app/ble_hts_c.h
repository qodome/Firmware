#ifndef BLE_HTS_C_H__
#define BLE_HTS_C_H__

#include <stdint.h>
#include "ble.h"

/* HTS Client event type. */
typedef enum
{
    BLE_HTS_C_EVT_DISCOVERY_COMPLETE = 1,   /* Event indicating that the Health Therometer Service has been discovered at the peer. */
    BLE_HTS_C_EVT_TM_INDICATION             /* Event indicating that an indication of the Temperature Measurement characteristic has been received from the peer. */
} ble_hts_c_evt_type_t;

/* Structure containing the temperature measurement received from the peer. */
typedef struct
{
    uint32_t tm_value;  /* Temperature Value. */
} ble_temp_t;

/* Health Therometer Event structure. */
typedef struct
{
    ble_hts_c_evt_type_t evt_type;  /**< Type of the event. */
    union
    {
        ble_temp_t temp;  /* Temperature measurement received. This will be filled if the evt_type is @ref BLE_HTS_C_EVT_TM_INDICATION */
    } params;
} ble_hts_c_evt_t;

// Forward declaration of the ble_bas_t type.
typedef struct ble_hts_c_s ble_hts_c_t;

typedef void (* ble_hts_c_evt_handler_t) (ble_hts_c_t * p_ble_hts_c, ble_hts_c_evt_t * p_evt);

/* Temperature Measurement Client structure */
typedef struct ble_hts_c_s
{
    uint16_t                conn_handle;      /**< Connection handle as provided by the SoftDevice. */
    uint16_t                hts_cccd_handle;  /**< Handle of the CCCD of the Health Therometer Measurement characteristic. */
    uint16_t                tm_handle;        /**< Handle of the Temperature Measurement characteristic as provided by the SoftDevice. */
    ble_hts_c_evt_handler_t evt_handler;      /**< Application event handler to be called when there is an event related to the heart rate service. */
} ble_hts_c_t;

/* HTS initialization structure. */
typedef struct
{
    ble_hts_c_evt_handler_t evt_handler;  /**< Event handler to be called by the HTS Client module whenever there is an event related to the HTS */
} ble_hts_c_init_t;

uint32_t ble_hts_c_init(uint16_t conn_handle, ble_hts_c_t * p_ble_hts_c, ble_hts_c_init_t * p_ble_hts_c_init);

void ble_hts_c_on_ble_evt(ble_hts_c_t * p_ble_hts_c, const ble_evt_t * p_ble_evt);

uint32_t ble_hts_c_tm_idct_enable(ble_hts_c_t * p_ble_hts_c);

#endif // BLE_HTS_C_H__
