 /**
 ****************************************************************************************
 *
 * @file app_stream_queue.c
 *
 * @brief Stream Queue Mechanism.
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"               // SW configuration
#if (BLE_APP_PRESENT)
#if (STREAMDATA_QUEUE)
#include "app_stream_queue.h"

#include <string.h>                  // string manipulation and functions

#include "app.h"                     // application definitions
#include "app_task.h"                // application task definitions
#include "l2cc_task.h"
#include "co_bt.h"

#include "arch.h"                      // platform definitions
#include "datasheet.h"
#include "gpio.h"

#if PLF_DISPLAY
#include "display.h"
#endif //PLF_DISPLAY


#include "l2cm.h"

#define STREAMDATAD_PACKET_SIZE (20)

#define MAX_TX_BUFS (18)
#define METRICS_UPD 400 
#define FIFO_SIZE (25)




/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

void add_err_packet(void);
void init_metrics (void);
void calc_error_tx (void);
void calc_update_avail (int16_t available);
void calc_update (void);
inline void add_packets_tx ( int16_t data_bytes);
void stream_fifo_init (void);
int stream_fifo_check (uint8 idx);

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
*/
struct connection_params_t
{
  uint16_t pkt_conint;          //number of packets per connection interval
  uint16_t connection_int;      //
};

struct stream_pkt
{
  void *datapt;
  int handle;
  void (*p_callback) (void* , int);
  uint16 type;
  uint8 len;
  uint8 used;
};
struct stream_pkt pkt_fifo[FIFO_SIZE];


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
*/

uint8 fifo_read=0;
uint8 fifo_write=0;

// maximum packets per connection interval (max_pkt_conint[0] refers to 7.5 ms)
const uint16_t max_pkt_conint[]={7,8,9,10,10,12,13,13,15,15,16,17};
// maximum rate per connection interval (max_rate_conint[0] refers to 7.5 ms)
const uint16_t max_rate_conint[]={149,146,144,142,128,139,138,128,137,128,128};

int8 asynch_flag=0;
  //TODO read the connection interval and set the min_val
int str_count=8;          //needs +1 ??
  //TODO read the connection interval and set the min_val
int min_val=1;
struct connection_params_t stream_params={7,6}; //DEFAULT settings 7 packets for connection interval 7.5ms

/*
 * 
 ****************************************************************************************
*/
extern int transmitting_data;
extern char cpt_event;


/**
 ****************************************************************************************
 * @brief Setup the target number of packets send per interval
 *
 * @param[in] connection_int   Connection interval settings in multiple of 1.25ms.
 *
 * @return The target number of packets.
 ****************************************************************************************
 */

uint16_t stream_setup_int_tomax (uint16_t connection_int)
{
    int16_t maximum_pkts=17;
    if ((connection_int<18)&&(connection_int>5))
    {
        maximum_pkts=max_pkt_conint[(connection_int-6)];
    }
    stream_params.connection_int=connection_int;
    stream_params.pkt_conint=maximum_pkts;
    str_count=stream_params.pkt_conint+1;
    
    return (stream_params.pkt_conint);  //returns the value of pkt sent per connection interval
    
}



#ifdef METRICS

struct metrics global_metrics;
struct metrics global_metrics_tosend;

/**
 ****************************************************************************************
 * @brief Initialiaze all state variables for the statistics
 *
 * @return none
 ****************************************************************************************
 */
 
void init_metrics (void)
{
    memset (&global_metrics,0,sizeof(struct metrics));
    global_metrics.last_available=l2cm_get_nb_buffer_available();
}

/**
 ****************************************************************************************
 * @brief Update the state variable that tracks the number of available buffers in the queue
 *
 * @param[in] available     Available buffers in queue.
 *
 * @return none
 ****************************************************************************************
 */

inline void calc_update_avail (int16_t available)
{
  global_metrics.last_available=available;
}

/**
 ****************************************************************************************
 * @brief Increase the counter of errors by one
 *
 * @return none
 ****************************************************************************************
 */

void add_err_packet(void)
{
    global_metrics.packets_errors++;
    
};

/**
 ****************************************************************************************
 * @brief Update the number of packets and bytes received 
 *
 * @param[in] data_bytes Number of data bytes received 
 *
 * @return none
 ****************************************************************************************
 */

void add_packets_rx ( int16_t data_bytes)
{
  global_metrics.packets_rx++;
  global_metrics.bytes_rx+=data_bytes;

}

/**
 ****************************************************************************************
 * @brief Update the number of packets and bytes transmitted 
 *
 * @param[in] data_bytes Number of data bytes transmitted 
 *
 * @return none
 ****************************************************************************************
 */

void add_packets_tx ( int16_t data_bytes)
{
  global_metrics.packets_tx++;
  global_metrics.bytes_tx+=data_bytes;

}

/**
 ****************************************************************************************
 * @brief Check and periodically produce a report with all the statistics. 
 *
 * @return none
 ****************************************************************************************
 */

void calc_update (void)
{
    
    uint16_t inbuffer;
    
  global_metrics.metrics_cnt++;
  if ((global_metrics.metrics_cnt*stream_params.connection_int)>=(METRICS_UPD))
  {
    
    memcpy(&global_metrics_tosend,&global_metrics,sizeof(struct metrics));
    memset(&global_metrics,0,sizeof(struct metrics));
    global_metrics.last_available=global_metrics_tosend.last_available;
    //TODO change to measure better
    int16_t still_in_buffer=MAX_TX_BUFS-global_metrics.last_available;
    global_metrics_tosend.timeperiod = global_metrics_tosend.metrics_cnt*stream_params.connection_int;
    inbuffer = 18 - global_metrics.last_available;
    global_metrics_tosend.packets_tx-=inbuffer;
    global_metrics_tosend.bytes_tx-=((inbuffer)*20);
    global_metrics.packets_tx=inbuffer;
    global_metrics.bytes_tx=((inbuffer)*20);
    global_metrics_tosend.tosend=1;
  }
}

#endif




/**
 ****************************************************************************************
 * @brief Initialize the fifo used by the stream mechanism. 
 *
 * @return none
 ****************************************************************************************
 */

void stream_fifo_init ()
{
  memset (pkt_fifo, 0, sizeof (struct stream_pkt[FIFO_SIZE]));
  fifo_read=fifo_write=0;
  

}

//L2C_CODE_ATT_WR_CMD
//L2C_CODE_ATT_HDL_VAL_NTF


/**
 ****************************************************************************************
 * @brief Add a new packet in the stream fifo to be sent using notifications, or write no response messages. 
 *
 * @param[in] datapt    A pointer to where the data are kept
 *
 * @param[in] len       Number of data to be sent
 *
 * @param[in] handle    The profile handle of the parameter sent  
 *
 * @param[in] type      Type of message. L2C_CODE_ATT_HDL_VAL_NTF for notifications and L2C_CODE_ATT_WR_CMD for write no response messages
 *
 * @param[in] p_callback A callback to be called when the data are copied, in order for the caller to free the buffer.
 *
 * @return -1 if the FIFO is full or else the index within the FIFO array where the message has been stored
 ****************************************************************************************
 */

int stream_fifo_add (void* datapt, uint8 len, int handle, int type, void (*p_callback) (void* , int))
{
  uint8 ret_val;
  if (pkt_fifo[fifo_write].used!=0)
    return -1;  //buffer is full
  
  pkt_fifo[fifo_write].used=1;
  pkt_fifo[fifo_write].datapt=datapt;
  pkt_fifo[fifo_write].len=len;
  pkt_fifo[fifo_write].handle=handle;
  pkt_fifo[fifo_write].type=type;
  pkt_fifo[fifo_write].p_callback=p_callback;
  ret_val=fifo_write;
  fifo_write++;
  if (fifo_write>=FIFO_SIZE) fifo_write=0;
  return (ret_val);
}

/**
 ****************************************************************************************
 * @brief Check if a specific FIFO item is used or free. 
 *
 * @param[in] idx    The index of the FIFO item
 *
 * @return -1 if index is out of range, else 0 if the item is free 1 if it is used.
 ****************************************************************************************
 */
int stream_fifo_check (uint8 idx)
{
  if (idx>=FIFO_SIZE)
    return -1;
  else
    return (pkt_fifo[idx].used);
}



/**
 ****************************************************************************************
 * @brief Prepare a message for delivery to L2CAP. 
 *
 * @param[in] strpkt    Pointer to the FIFO item describing the message.
 *
 * @return none.
 ****************************************************************************************
 */
void send_pkt_to_l2cc ( struct stream_pkt *strpkt)
{
   
    struct l2cc_pdu_send_req *pkt = KE_MSG_ALLOC_DYN(L2CC_PDU_SEND_REQ,
                                                 KE_BUILD_ID(TASK_L2CC, app_env.conidx), TASK_APP,
                                                 l2cc_pdu_send_req,
                                                 STREAMDATAD_PACKET_SIZE);
    
    if (pkt==NULL) return;
    
    
    // Set attribute channel ID
    pkt->pdu.chan_id   = L2C_CID_ATTRIBUTE;
    // Set packet opcode.
    pkt->pdu.data.code = strpkt->type;
    if (pkt->pdu.data.code == L2C_CODE_ATT_HDL_VAL_NTF)
    {    
        pkt->pdu.data.hdl_val_ntf.handle  = strpkt->handle;
        pkt->pdu.data.hdl_val_ntf.value_len = strpkt->len;
        /* copy the content to value */
        memcpy(&(pkt->pdu.data.hdl_val_ntf.value[0]), strpkt->datapt, pkt->pdu.data.hdl_val_ntf.value_len);
    }
    else
    {
        pkt->pdu.data.wr_cmd.handle  = strpkt->handle;
        pkt->pdu.data.wr_cmd.value_len = strpkt->len;
        /* copy the content to value */
        memcpy(&(pkt->pdu.data.wr_cmd.value[0]), strpkt->datapt, pkt->pdu.data.wr_cmd.value_len);
    }
    ke_msg_send(pkt);
     
#ifdef METRICS
        add_packets_tx(strpkt->len);
#endif
     
#ifdef RATE_MEASURE    
      packets_sent++;
#endif
     
      strpkt->used=0;
      if (strpkt->p_callback!=NULL)
        strpkt->p_callback(strpkt->datapt,strpkt->handle);
}


/**
 ****************************************************************************************
 * @brief Fill the transmit queue up to point. 
 *
 * @param[in] min_avail    How many places should be kept free in the queue.
 *
 * @return the number of packets actually placed in the queue.
 ****************************************************************************************
 */

static inline int stream_queue_data_until (uint8 min_avail)
{
    int max_count=0;
	  int available;
    int retval=0;
  
    if (pkt_fifo[fifo_read].used==0) 
      return retval; //nothing to send quick check in order not to spend time
  
    #if 0
    //I may need to fill UP all places for large connection intervals.
    if (min_avail==0) //never place the device into busy state
      return retval; 
    #endif
    
    available=l2cm_get_nb_buffer_available();
  	if (available<=1) 
      return retval;       //never place the device into busy state
	  
    max_count=available-min_avail;  //maximum number of packets to add 
    
    while ((pkt_fifo[fifo_read].used!=0)&&(max_count>0))
    {
      send_pkt_to_l2cc (&pkt_fifo[fifo_read]);
      fifo_read++;
      retval++;
      max_count--;
      if (fifo_read>=FIFO_SIZE) 
          fifo_read=0;
    }
    return (retval);
    
}

/**
 ****************************************************************************************
 * @brief The stream handler executed at the beginning of a connection event. 
 *
 * @return the number of packets actually placed in the queue.
 ****************************************************************************************
 */

int stream_queue_more_data_during_tx (void)
{
  
  int retval=0;
  int until=min_val;
  int available, already_in;
  available=l2cm_get_nb_buffer_available();
  already_in=MAX_TX_BUFS-available;
  
  
  if (asynch_flag)           //previous connection event ended with asynch, current frees the buffers
    until=available-(str_count-1);
  if (until<1) until=1;

  if (already_in>str_count)  //send more than we want to, there will not be enough time 
  {  asynch_flag=1;           //to add more on the complete event we will add and flag 
//     set_pxact_gpio();
    }       //to complete event NOT to try to add more to get synched again
   else
    asynch_flag=0;

  retval=stream_queue_data_until (until);
#ifdef METRICS
   calc_update_avail (available-retval);
#endif
  return (retval);
	//set_pxact_gpio();
}


/**
 ****************************************************************************************
 * @brief The stream handler executed at the end of a connection event. 
 *
 * @return the number of packets actually placed in the queue.
 ****************************************************************************************
 */
int stream_queue_more_data_end_of_event (void)
{
    int retval=0;
	  
    int available, already_in, until;
    available=l2cm_get_nb_buffer_available();
    already_in=MAX_TX_BUFS-available;

    if (already_in>=str_count)
    {
     // set_pxact_gpio();
#ifdef METRICS
      calc_update_avail (available);
#endif
      return retval; //no need to send anything
    }
  
    if ((available<17)&&(asynch_flag==1))
    {
      //set_pxact_gpio();
#ifdef METRICS
      calc_update_avail (available);
#endif
      return retval;  //try to get synched again
    }
   
    until=MAX_TX_BUFS-str_count;
    if ((until==0)&&(str_count!=18)) 
      until=1;
    
    retval=stream_queue_data_until (until);
#ifdef METRICS
    calc_update_avail (available-retval);
#endif 
    
    return (retval);
}

/**
 ****************************************************************************************
 * @brief The stream handler routine. Constantly called from the main loop. 
 *
 * @return the number of packets actually placed in the queue during this call.
 ****************************************************************************************
 */
int stream_queue_more_data(void)
{
  int retval=0;

    if (cpt_event==1)
    { 
        //calc_error_tx();
        retval=stream_queue_more_data_end_of_event();
        transmitting_data=0;
        cpt_event=2;
#ifdef METRICS
        calc_update();
#endif
      
      
#ifdef RATE_MEASURE    
      connection_events++
#endif
    }
    else
    if ((transmitting_data==1)&&(cpt_event==2))
     {
         //set_pxact_gpio();
				 retval=stream_queue_more_data_during_tx();
         transmitting_data=0;
        if (cpt_event==2) 
        {
          //if transmission stopped from remote the cpt_event may be already there
          cpt_event=0;
        }
      }
    return (retval);
}

#endif //(STREAMDATA_QUEUE)

#endif // (BLE_APP_PRESENT)

/// @} APP
