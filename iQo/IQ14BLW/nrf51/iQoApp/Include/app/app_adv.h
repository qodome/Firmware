/*
 * Application advertise parameters
 */
#ifndef __APP_ADV__
#define __APP_ADV__

#include "app_util.h"

#define APP_ADV_INTERVAL                     MSEC_TO_UNITS(1285, UNIT_0_625_MS)       /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           0                                        /**< The advertising timeout in units of seconds. */

#endif
