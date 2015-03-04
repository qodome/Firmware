/**
 ****************************************************************************************
 *
 * @file gap_task.h
 *
 * @brief Header file - GAPTASK.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: 24801 $
 *
 ****************************************************************************************
 */
#ifndef GAP_TASK_H_
#define GAP_TASK_H_
/**
 ****************************************************************************************
 * @addtogroup GAPTASK Task
 * @ingroup GAP
 * @brief Handles ALL messages to/from GAP block.
 *
 * The GAPTASK is responsible for managing the messages coming from
 * the link layer, application layer and host-level layers. The task
 * handling includes device discovery, bonding, connection establishment,
 * link termination and name discovery.
 *
 * Messages can originate from @ref L2C "L2C", @ref ATT "ATT", @ref SMP "SMP",
 * @ref LLM "LLM" and @ref LLC "LLC"
 *
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

//#include <stdbool.h>
#include "gapc_task.h"
#include "gapm_task.h"

#define TASK_APP    TASK_GTL
/// @} GAPTASK

#endif // GAP_TASK_H_
