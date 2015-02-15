/*
 * UTCtime_convert.h
 *
 *  Created on: 2014-10-27
 *      Author: Administrator
 */

#ifndef UTCTIME_CONVERT_H_
#define UTCTIME_CONVERT_H_

#include "ble_date_time.h"

void osal_ConvertUTCTime(ble_date_time_t *tm, uint32_t secTime);

uint32_t osal_ConvertUTCSecs(ble_date_time_t *tm);

#endif /* UTCTIME_CONVERT_H_ */
