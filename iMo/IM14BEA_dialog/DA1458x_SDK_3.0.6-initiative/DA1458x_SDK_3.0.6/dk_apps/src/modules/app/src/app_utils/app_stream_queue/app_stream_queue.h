/**
 ****************************************************************************************
 *
 * @file app_stream_queue.h
 *
 * @brief 
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef APP_STREAM_QUEUE_H_
#define APP_STREAM_QUEUE_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief AudioStreamer Application entry point.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwble_config.h"

#if (STREAMDATA_QUEUE)

#include <stdint.h>          // standard integer definition
#include <co_bt.h>



/*
 * GLOBAL VARIABLE DECLARATION
 ****************************************************************************************
 */
struct metrics 
{
  volatile uint16_t timeperiod;
  volatile uint16_t packets_tx;
  volatile uint16_t bytes_tx;

  volatile uint16_t packets_rx;
  volatile uint16_t bytes_rx;
  
  volatile uint16_t packets_errors;

    //temp variables of the metrics
  volatile uint16_t last_available;
  volatile uint16_t metrics_cnt;
  volatile uint8_t tosend;
};

/**
 ****************************************************************************************
 *
 * AudioStreamer Application Functions
 *
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initialize the fifo used by the stream mechanism. 
 ****************************************************************************************
 */
void stream_fifo_init (void);

/**
 ****************************************************************************************
 * @brief Add a new packet in the stream fifo to be sent using notifications, or write no response messages. 
 ****************************************************************************************
 */
int stream_fifo_add (void* datapt, uint8 len, int handle, int type, void (*p_callback) (void* , int));

/**
 ****************************************************************************************
 * @brief Check if a specific FIFO item is used or free. 
 ****************************************************************************************
 */
int stream_fifo_check (uint8 idx);

/**
 ****************************************************************************************
 * @brief The stream handler executed at the beginning of a connection event. 
 ****************************************************************************************
 */
int stream_queue_more_data(void);

/**
 ****************************************************************************************
 * @brief Setup the target number of packets send per interval
 ****************************************************************************************
 */
uint16_t stream_setup_int_tomax (uint16_t connection_int);

/**
 ****************************************************************************************
 * @brief Update the number of packets and bytes received 
 ****************************************************************************************
 */
void add_packets_rx ( int16_t data_bytes);

#ifdef METRICS
/**
 ****************************************************************************************
 * @brief Initialiaze all state variables for the statistics
 *
 * @return none
 ****************************************************************************************
 */
void init_metrics (void);
#endif // METRICS

#endif //BLE_STREAMDATA_DEVICE

/// @} APP

#endif // APP_H_
