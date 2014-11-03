/*
 * Statistics profile 
 */
#ifdef DEBUG_STATS

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "statistics.h"
#include "SoftwareSPI.h"
#include "recorder.h"
#include "oad.h"
#include "oad_target.h"
#include "st_util.h"
#include "temperature.h"
#include "recorder.h"
#include "OSAL_PwrMgr.h"
#include "hal_sleep.h"

#pragma location="XDATA_RESERVE"
__no_init struct stats_t s;
#pragma required=s

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void Stats_Init()
{ 
    osal_memset((void *)&s, 0, sizeof(struct stats_t));  
}

#endif
/*********************************************************************
 *********************************************************************/
