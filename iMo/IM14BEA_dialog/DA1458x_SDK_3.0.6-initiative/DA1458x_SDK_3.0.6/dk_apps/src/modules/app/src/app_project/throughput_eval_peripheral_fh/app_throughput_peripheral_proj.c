/**
****************************************************************************************
*
* @file app_throughput_peripheral_proj.c
*
* @brief Throughput evaluation peripheral project source code .
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
#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_sec.h"
#include "app_throughput_peripheral_proj.h"

#include "co_math.h"                 // Common Maths Definition
#include "gapc.h"
#if (NVDS_SUPPORT)
#include "nvds.h"                    // NVDS Definitions
#endif //(NVDS_SUPPORT)

#include "l2cc_task.h"
#include "pwm.h"

#if (STREAMDATA_QUEUE)
#include "app_stream_queue.h"
#endif
#include "spi.h"
#include "wkupct_quadec.h"



uint8_t test_state __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY
// uint8_t ecg_trigger_flag __attribute__((section("retention_mem_area0"), zero_init));
uint8_t ecg_data[1000] __attribute__((section("retention_mem_area0"), zero_init)); //23LC1024 sram 128*8=1024bit
uint16_t ecg_data_idx __attribute__((section("retention_mem_area0"), zero_init));
uint8_t ecg_start_flag __attribute__((section("retention_mem_area0"), zero_init));
uint8_t ecg_compressed[500];
uint32_t compressed_size = 0;
// uint32_t ecg_data_length = 200;
uint8_t read_ecg_data[500];
// uint32_t decoding_size = 0;
// uint8_t decoding_data[500] = {0};
// uint32_t total_size = 0;
#define MAX_TX_BUFS (32)
#define STREAMDATAD_PACKET_SIZE (20)
uint8 test_pkt [MAX_TX_BUFS][STREAMDATAD_PACKET_SIZE];
uint32_t band_id = 0x000000;
uint32_t address_read = 0x000000;
bool flag_block = 0;// when block = 0  ,WRITE	SRAM address 0x000000 ~0x010000
								//  when block = 1  ,WRITE SRAM address 0x010000 ~0x020000

bool flag_start_read = 0;
// acc varible
uint8_t acc_addr = 0;
uint8_t acc_value = 0;
uint8_t acc_data[20] = {0};
uint16_t acc_cnt = 0;
uint8_t acc_data_enabled = 0;
uint8_t acc_sample_cnt = 0;
uint8_t acc_started = 0;

// test
uint8_t test_acc = 0;
uint8_t test_acc_id = 0;
uint16_t acc_fifo_sample_cnt;


#ifdef METRICS
extern struct metrics global_metrics_tosend;
#endif

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

 /**
 ****************************************************************************************
 * @brief app_api function. Project's actions in app_init
 *
 * @return void.
 ****************************************************************************************
*/

void app_init_func(void)
{	

/**************************************************
Call initializiation function for supported profiles
i.e.    
#if (BLE_DIS_SERVER)    
	app_dis_init();
#endif
****************************************************/    

}

/**
 ****************************************************************************************
 * @brief app_api function. Project's actions in app_sec_init during system initialization
 *
 * @return void.
 ****************************************************************************************
*/

void app_sec_init_func(void)
{

/**************************************************
Initialize security environment. 
i.e.    
//Setup required auth mode.    
#if (BLE_APP_SEC)
	app_sec_env.auth = (GAP_AUTH_REQ_NO_MITM_BOND);
#endif
****************************************************/    
    
}


/**
 ****************************************************************************************
 * @brief app_api function. Project's actions in app_connect (device connection)
 *
 * @param[in] taskid     App task's id.
 * @param[in] param      Received gapc_connection_req_ind.
 *
 * @return bool     true for connection request acception / false rejection 
 ****************************************************************************************
*/

void app_connection_func(struct gapc_connection_req_ind const *param)
{
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
    
        ke_timer_clear(APP_CONN_TIMER, TASK_APP);
        
        ke_state_set(TASK_APP, APP_CONNECTED);
        
#if BLE_INTEGRATED_HOST_GTL
        struct app_ext_connect_ind *cmd = KE_MSG_ALLOC(APP_EXT_CONNECT_CMP_IND,
                                               TASK_GTL, TASK_APP, app_ext_connect_ind);
	
        cmd->state = true;
        cmd->conn_handle = param->conhdl;

        cmd->addr_type = param->peer_addr_type;
        memcpy(cmd->addr, param->peer_addr.addr, BD_ADDR_LEN);
    
        // Send the message
        ke_msg_send(cmd);
#endif        

#if (STREAMDATA_QUEUE)        
        stream_setup_int_tomax(param->con_interval);
#endif
        
#if BLE_STREAMDATA_DEVICE        
        app_streamdatad_enable();
#endif
        
        // Retrieve the connection info from the parameters
        //app_env.conhdl = param->conhdl;

        //app_env.peer_addr_type = param->peer_addr_type;
        //memcpy(app_env.peer_addr.addr, param->peer_addr.addr, BD_ADDR_LEN);

# if (BLE_APP_SEC)
            
        // send connection confirmation
        app_connect_confirm(app_sec_env.auth);
        
# else // (BLE_APP_SEC)
        // send connection confirmation
        app_connect_confirm(GAP_AUTH_REQ_NO_MITM_NO_BOND);            
# endif // (BLE_APP_SEC)
    }
    else
    {
        // No connection has been establish, restart advertising
        app_adv_start();
    }    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Project advertising function. Setup advertise string.
 *
 *
 *
 * @return void.
 ****************************************************************************************
*/
void app_adv_func(struct gapm_start_advertise_cmd *cmd)
{

//  Start advertising. Fill GAPM_START_ADVERTISE_CMD message

     //  Device Name Length
    uint8_t device_name_length;
    int8_t device_name_avail_space;
    uint8_t device_name_temp_buf[64];

    cmd->op.code     = GAPM_ADV_UNDIRECT;
    cmd->op.addr_src = GAPM_PUBLIC_ADDR;
    cmd->intv_min    = 150;//APP_ADV_INT_MIN;
    cmd->intv_max    = 150;//APP_ADV_INT_MAX;
    cmd->channel_map = APP_ADV_CHMAP;

    cmd->info.host.mode = GAP_GEN_DISCOVERABLE;

    /*-----------------------------------------------------------------------------------
     * Set the Advertising Data and the Scan Response Data
     *---------------------------------------------------------------------------------*/
    cmd->info.host.adv_data_len       = APP_ADV_DATA_MAX_SIZE;
    cmd->info.host.scan_rsp_data_len  = APP_SCAN_RESP_DATA_MAX_SIZE;
    
    // Advertising Data
    #if (NVDS_SUPPORT)
    if(nvds_get(NVDS_TAG_APP_BLE_ADV_DATA, &cmd->info.host.adv_data_len,
                &cmd->info.host.adv_data[0]) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        cmd->info.host.adv_data_len = APP_ADV_DATA_LEN;
        memcpy(&cmd->info.host.adv_data[0], APP_ADV_DATA, cmd->info.host.adv_data_len);
    }

    // Scan Response Data
    #if (NVDS_SUPPORT)
    if(nvds_get(NVDS_TAG_APP_BLE_SCAN_RESP_DATA, &cmd->info.host.scan_rsp_data_len,
                &cmd->info.host.scan_rsp_data[0]) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        cmd->info.host.scan_rsp_data_len = APP_SCNRSP_DATA_LENGTH;
        memcpy(&cmd->info.host.scan_rsp_data[0], APP_SCNRSP_DATA, cmd->info.host.scan_rsp_data_len);
    }

    // Get remaining space in the Advertising Data - 2 bytes are used for name length/flag
    device_name_avail_space = APP_ADV_DATA_MAX_SIZE - cmd->info.host.adv_data_len - 2;

    // Check if data can be added to the Advertising data
    if (device_name_avail_space > 0)
    {
        // Get the Device Name to add in the Advertising Data (Default one or NVDS one)
        #if (NVDS_SUPPORT)
        device_name_length = NVDS_LEN_DEVICE_NAME;
        if (nvds_get(NVDS_TAG_DEVICE_NAME, &device_name_length, &device_name_temp_buf[0]) != NVDS_OK)
        #endif //(NVDS_SUPPORT)
        {
            // Get default Device Name (No name if not enough space)
            device_name_length = strlen(APP_DEVICE_NAME);
            memcpy(&device_name_temp_buf[0], APP_DEVICE_NAME, device_name_length);
        }

        if(device_name_length > 0)
        {
            // Check available space
            device_name_length = co_min(device_name_length, device_name_avail_space);

            // Fill Length
            cmd->info.host.adv_data[cmd->info.host.adv_data_len]     = device_name_length + 1;
            // Fill Device Name Flag
            cmd->info.host.adv_data[cmd->info.host.adv_data_len + 1] = '\x09';
            // Copy device name
            memcpy(&cmd->info.host.adv_data[cmd->info.host.adv_data_len + 2], device_name_temp_buf, device_name_length);

            // Update Advertising Data Length
            cmd->info.host.adv_data_len += (device_name_length + 2);
        }
    }
    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Project's actions in app_disconnect
 *
 * @param[in] taskid     App task's id.
 * @param[in] param      Received gapc_disconnect_ind msg.
 *
 * @return void.            
 ****************************************************************************************
*/

void app_disconnect_func(ke_task_id_t task_id, struct gapc_disconnect_ind const *param)
{
    
    uint8_t state = ke_state_get(task_id);
    
#if BLE_BATT_SERVER
 app_batt_poll_stop();
#endif // BLE_BATT_SERVER
    
    if ((state == APP_SECURITY) || (state == APP_CONNECTED)  || (state == APP_PARAM_UPD))
    {
        
/**************************************************
            Handle disconnection event
***************************************************/    

        ke_state_set(TASK_APP, APP_CONNECTABLE);
#if (STREAMDATA_QUEUE)        
        test_state = APP_STREAM_STOP;
#endif
        
#if BLE_INTEGRATED_HOST_GTL        
        
        struct app_ext_connect_ind *cmd = KE_MSG_ALLOC(APP_EXT_DISCONNECT_CMP_IND,
                                               TASK_GTL, TASK_APP, app_ext_connect_ind);
	
        cmd->conn_handle = param->conhdl;
    
        // Send the message
        ke_msg_send(cmd);
        
#endif

        app_adv_start();
                        
    }
    else
    {
        // We are not in a Connected State
        ASSERT_ERR(0);
    }
      
}    
						


/**
 ****************************************************************************************
 * @brief app_api function. Project's actions for profiles' database initialization
 *
 * @return void.
 ****************************************************************************************
*/

volatile uint8_t dbg; 
bool app_db_init_func(void)
{

/**************************************************
Initialize next supported profile's Database.    
Check if all supported profiles' Databases are initialized and return status.
***************************************************/    

    
    // Indicate if more services need to be added in the database
    bool end_db_create = false;
    
    dbg = APP_PRF_LIST_STOP;
    
    // Check if another should be added in the database
    if (app_env.next_prf_init < APP_PRF_LIST_STOP)
    {
        switch (app_env.next_prf_init)
        {
            #if (BLE_STREAMDATA_DEVICE)
            case (APP_STREAM_TASK):
            {
                // Add proxr Service in the DB
                app_stream_create_db();
            } break;
          #endif
            default:
            {
                ASSERT_ERR(0);
            } break;

        }

        // Select following service to add
        app_env.next_prf_init++;
    }
    else
    {
        end_db_create = true;
    }

    return end_db_create;
}

/**
 ****************************************************************************************
 * @brief app_api function. Sends request to update connection's paramaters.
 *
 * @return void.
 ****************************************************************************************
*/

void app_param_update_func(void)
{

/**************************************************
Create and send connection param update message.
***************************************************/    

    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Project configures GAPM. Called upon reset completion
 *
 * @param[in] task_id    id of the kernel task calling this function
 * @param[in] cmd        parameters to pass to the stack
 *
 * @return void.
 ****************************************************************************************
*/

void app_configuration_func(ke_task_id_t const task_id, struct gapm_set_dev_config_cmd *cmd)
{
    
/**************************************************
Create GAPM_SET_DEV_CONFIG message to configure BLE stack
***************************************************/    
    
    
    // set device configuration
    cmd->operation = GAPM_SET_DEV_CONFIG;
    // Device Role
    cmd->role = GAP_PERIPHERAL_SLV;
	
    // Device IRK
    // cmd->irk = ;

    // Device Appearance
    cmd->appearance = 0x0000;

    // Device Appearance write permission requirements for peer device
    cmd->appearance_write_perm = GAPM_WRITE_DISABLE;
    // Device Name write permission requirements for peer device
    cmd->name_write_perm = GAPM_WRITE_DISABLE;
    
    // Maximum trasnimt unit size
    cmd->max_mtu = 23;
		
    /*
    // Peripheral only: *****************************************************************
    // Slave preferred Minimum of connection interval
    cmd->con_intv_min = 8;         // 10ms (8*1.25ms)
    // Slave preferred Maximum of connection interval
    cmd->con_intv_max = 16;        // 20ms (16*1.25ms)
    // Slave preferred Connection latency
    cmd->con_latency  = 0;
    // Slave preferred Link supervision timeout
    cmd->superv_to    = 100;

    // Privacy settings bit field
    cmd->flags = 0;
    */
		
    return;   
}

/**
 ****************************************************************************************
 * @brief app_api function. Called upon device's configuration completion
 *
 * @return void.
 ****************************************************************************************
*/

void app_set_dev_config_complete_func(void)
{
    ke_state_set(TASK_APP, APP_DB_INIT);
    
    if (app_db_init())
    {
        // No service to add in the DB -> Start Advertising
        app_adv_start();
    }
    
#if BLE_INTEGRATED_HOST_GTL    
	struct app_ready_ind *ind = KE_MSG_ALLOC(APP_READY_IND, TASK_GTL, TASK_APP, app_ready_ind);
   
	ke_msg_send(ind);    
#endif //BLE_INTEGRATED_HOST_GTL
	
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Called upon connection param's update rejection
 *
 * @param[in] status        Error code
 *
 * @return void.
 ****************************************************************************************
*/

void app_update_params_rejected_func(uint8_t status)
{
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Called upon connection param's update completion
 *
 * @return void.
 ****************************************************************************************
*/

void app_update_params_complete_func(void)
{

/**************************************************
Handle connection parameters update event 
***************************************************/    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles undirect advertising completion.
 *
 * @param[in] status        Command complete message status
 *
 * @return void.
 ****************************************************************************************
*/

void app_adv_undirect_complete(uint8_t status)
{
    
/**************************************************
Handle undirected advirtising complete event
***************************************************/    
    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles direct advertising completion.
 *
 * @param[in] status        Command complete message status
 *
 * @return void.
 ****************************************************************************************
*/

void app_adv_direct_complete(uint8_t status)
{
    
/**************************************************
Handle directed advirtising complete event
***************************************************/    
    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles Database creation. Start application.
 *
 * @return void.
 ****************************************************************************************
*/

void app_db_init_complete_func(void)
{
    
    app_adv_start();
    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles scan procedure completion.
 *
 * @return void.
 ****************************************************************************************
*/

void app_scanning_completed_func(void)
{
    
}

/**
 ****************************************************************************************
 * @brief app_api function. Fill gapm_start_connection_cmd msg params.
 *
 * @param[in] msg        gapm_start_connection_cmd message
 *
 * @return void.
 ****************************************************************************************
*/

void app_connect_func(struct gapm_start_connection_cmd *msg)                           
{
    
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles reception of gapm_adv_report_ind msg.
 *
 * @param[in] param     gapm_adv_report_ind message
 *
 * @return void.
 ****************************************************************************************
*/

void app_adv_report_ind_func(struct gapm_adv_report_ind *param)
{
    
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles connection request failure.
 *
 * @return void.
 ****************************************************************************************
*/

void app_connect_failed_func(void)
{
    
}

#if (BLE_APP_SEC)
void app_send_pairing_rsp_func(struct gapc_bond_req_ind *param)
{

/**************************************************
Handle pairring request message. send a peiring response 
i.e.    
    
    struct gapc_bond_cfm* cfm = KE_MSG_ALLOC(GAPC_BOND_CFM, TASK_GAPC, TASK_APP, gapc_bond_cfm);

    cfm->request = GAPC_PAIRING_RSP;
    cfm->accept = true;

    // OOB information
    cfm->data.pairing_feat.oob            = GAP_OOB_AUTH_DATA_NOT_PRESENT;
    // Encryption key size
    cfm->data.pairing_feat.key_size       = KEY_LEN;
    // IO capabilities
    cfm->data.pairing_feat.iocap          = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
    // Authentication requirements
    cfm->data.pairing_feat.auth           = GAP_AUTH_REQ_NO_MITM_BOND;
    //Security requirements
    cfm->data.pairing_feat.sec_req        = GAP_NO_SEC;
    //Initiator key distribution
    //GZ cfm->data.pairing_feat.ikey_dist      = GAP_KDIST_NONE;
    cfm->data.pairing_feat.ikey_dist      = GAP_KDIST_SIGNKEY;
    //Responder key distribution
    cfm->data.pairing_feat.rkey_dist      = GAP_KDIST_ENCKEY;
    
    ke_msg_send(cfm);
    
***************************************************/
}

void app_send_tk_exch_func(struct gapc_bond_req_ind *param)
{
    
/**************************************************
Send Temporary key 
i.e.        
    struct gapc_bond_cfm* cfm = KE_MSG_ALLOC(GAPC_BOND_CFM, TASK_GAPC, TASK_APP, gapc_bond_cfm);
    uint32_t pin_code = app_sec_gen_tk();
    cfm->request = GAPC_TK_EXCH;
    cfm->accept = true;
    
    memset(cfm->data.tk.key, 0, KEY_LEN);
    
    cfm->data.tk.key[3] = (uint8_t)((pin_code & 0xFF000000) >> 24);
    cfm->data.tk.key[2] = (uint8_t)((pin_code & 0x00FF0000) >> 16);
    cfm->data.tk.key[1] = (uint8_t)((pin_code & 0x0000FF00) >>  8);
    cfm->data.tk.key[0] = (uint8_t)((pin_code & 0x000000FF) >>  0);
    
    ke_msg_send(cfm);
***************************************************/    
}

void app_send_irk_exch_func(struct gapc_bond_req_ind *param)
{
    
/**************************************************
Send IRK
***************************************************/        
    return;
}

void app_send_csrk_exch_func(struct gapc_bond_req_ind *param)
{
/**************************************************
Send CSRK
i.e.
    
    struct gapc_bond_cfm* cfm = KE_MSG_ALLOC(GAPC_BOND_CFM, TASK_GAPC, TASK_APP, gapc_bond_cfm);

    cfm->request = GAPC_CSRK_EXCH;

    cfm->accept = true;

    memset((void *) cfm->data.csrk.key, 0, KEY_LEN);
    memcpy((void *) cfm->data.csrk.key, (void *)"\xAB\xAB\x45\x55\x23\x01", 6);

    ke_msg_send(cfm);
***************************************************/        
}

void app_send_ltk_exch_func(struct gapc_bond_req_ind *param)
{
/**************************************************
Send Long term key 
i.e.
    
    struct gapc_bond_cfm* cfm = KE_MSG_ALLOC(GAPC_BOND_CFM, TASK_GAPC, TASK_APP, gapc_bond_cfm);

    // generate ltk
    app_sec_gen_ltk(param->data.key_size);

    cfm->request = GAPC_LTK_EXCH;

    cfm->accept = true;

    cfm->data.ltk.key_size = app_sec_env.key_size;
    cfm->data.ltk.ediv = app_sec_env.ediv;

    memcpy(&(cfm->data.ltk.randnb), &(app_sec_env.rand_nb) , RAND_NB_LEN);
    memcpy(&(cfm->data.ltk.ltk), &(app_sec_env.ltk) , KEY_LEN);

    ke_msg_send(cfm);
***************************************************/        
}

void app_paired_func(void)
{
/**************************************************
Handle Pairing/Bonding copletion event
**************************************************/    
    return;
}

bool app_validate_encrypt_req_func(struct gapc_encrypt_req_ind *param)
{
    
/**************************************************
Handle Pairing/Bonding copletion event
**************************************************/        
    
    return true;
}

void app_sec_encrypt_ind_func(void)
{
    
    return; 
}

void app_sec_encrypt_complete_func(void)
{
/**************************************************
Handle encryption completed event. 
****************************************************/    
    
    return; 
}

void app_mitm_passcode_entry_func(ke_task_id_t const src_id, ke_task_id_t const dest_id)
{
    
/**************************************************
Handle encryption completed event. 
****************************************************/        
    
    return;
}

#endif //BLE_APP_SEC

#if BLE_INTEGRATED_HOST_GTL

/**
 ****************************************************************************************
 * @brief Handles reception of uart test message from external processor application. Responds with APP_EXT_TEST_RSP
 * to confirm that channel is alive. 
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int app_uart_test_req_handler(ke_msg_id_t const msgid,
                                           const struct app_uart_test_req *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    struct app_uart_test_rsp *cmd = KE_MSG_ALLOC(APP_EXT_TEST_RSP,
                                           src_id , TASK_APP,
                                           app_uart_test_rsp);

    cmd->value = param->value + 1;

    // Send the message
    ke_msg_send(cmd);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of disconnect command from external processor application. Disconnects from peripherals
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int app_ext_disconnect_cmd_handler(ke_msg_id_t const msgid,
                             const struct app_ext_disconnect_cmd *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
	
    uint8_t conidx;
    
    conidx = gapc_get_conidx(param->conn_handle);
    
    struct gapc_disconnect_cmd *cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                                              KE_BUILD_ID(TASK_GAPC, conidx), TASK_APP,
                                              gapc_disconnect_cmd);

    cmd->operation = GAPC_DISCONNECT;
    cmd->reason = CO_ERROR_REMOTE_USER_TERM_CON;

    // Send the message
    ke_msg_send(cmd);
	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of start streaming command from external processor application. Starts/stops transmition proccess
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int app_ext_transmit_cmd_handler(ke_msg_id_t const msgid,
                             const struct app_ext_transmit_cmd *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
  

    /// Start transmission
#if (STREAMDATA_QUEUE)        
    if (param->action == APP_STREAM_START)
    {
        if (test_state != APP_STREAM_START)
        {
//            set_pxact_gpio();
            test_pkt_init();
        }
        test_state = APP_STREAM_START;
    }
    else
        test_state = APP_STREAM_STOP;
#endif 
    
    return (KE_MSG_CONSUMED);
}

#endif //BLE_INTEGRATED_HOST_GTL

/**
 ****************************************************************************************
 * @brief Sends streaming counters to external processor application. 
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

void app_ext_stats_report_send(void)
{
    
    if (ke_state_get(TASK_APP) == APP_CONNECTED)
    {
#ifdef METRICS
        //set_pxact_gpio();
#if  BLE_INTEGRATED_HOST_GTL
        struct app_ext_stats_report *cmd = KE_MSG_ALLOC(APP_EXT_STATS_REPORT,
                                                   TASK_GTL, TASK_APP, app_ext_stats_report);

        //memset(cmd, 0, sizeof(struct app_ext_stats_report));

        cmd->type =  0;
        cmd->conn_handle =  app_env.conhdl;
        memcpy(&cmd->timeperiod, &global_metrics_tosend, sizeof(uint16_t) * 6);
                
        // Send the message
        ke_msg_send(cmd);
#endif
#endif        //METRICS
        
    }
}

/**
****************************************************************************************
* @brief Send an ECG message over the SPI
* @param[in] size:      size of data to send in bytes
* @param[in] *ecg_ptr:  pointer to the first byte to be sent

****************************************************************************************
*/

void write_ecg_data_to_sram(uint32_t size,uint32_t address, uint8_t *write_ecg_ptr)
{
		uint32_t i; 
	  uint32_t address_temp = 0;
	  uint32_t band_id_init = 0x010000;
		
/**
****************************************************************************************
 Initialize SPI
****************************************************************************************
*/	
		RESERVE_GPIO(SPI_EN, GPIO_PORT_2, GPIO_PIN_5, PID_SPI_EN); 
    RESERVE_GPIO(SPI_CLK, GPIO_PORT_2, GPIO_PIN_8, PID_SPI_CLK);
    RESERVE_GPIO(SPI_DO, GPIO_PORT_2, GPIO_PIN_9, PID_SPI_DO);
    RESERVE_GPIO(SPI_DI, GPIO_PORT_2, GPIO_PIN_6, PID_SPI_DI);
    RESERVE_GPIO(EN_HOLD, GPIO_PORT_2, GPIO_PIN_7, PID_HOLD);
	
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_5, OUTPUT, PID_SPI_EN, true );
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_8, OUTPUT, PID_SPI_CLK, false );
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_9, OUTPUT, PID_SPI_DO, false );	
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_6, INPUT, PID_SPI_DI, false );
	  GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_7, OUTPUT, PID_GPIO, true );
	
    SPI_Pad_t cs_pad_param;
    cs_pad_param.port = GPIO_PORT_2;
    cs_pad_param.pin = GPIO_PIN_5;
    
    spi_init(&cs_pad_param, SPI_MODE_8BIT, SPI_ROLE_MASTER, SPI_CLK_IDLE_POL_LOW, SPI_PHA_MODE_0, SPI_MINT_DISABLE, SPI_XTAL_DIV_14);
 		
		if((size + address) >= 0x020000) 
		{
				address = band_id_init * 0;
				address_temp = address;
				band_id = address + size;
				flag_block = 0;
			 // flag_start_read = 1;
		}
	
		else if(((size + address) <= 0x010000 ) && ((address) <  0x010000))
		{
				
				address_temp = address;
				band_id = address +size;
				flag_block = 0;
		}
		else if(((size + address) >= 0x010000 ) && ((address) <  0x010000))
		{
				address = band_id_init;
				address_temp = address;
				band_id = address +size;
				flag_block = 1;
			 // flag_start_read = 1;
		}
		
		else // if(((size + address) <= 0x020000 ) && ((address) >  0x010000))
		{
				address_temp = address;
			  band_id = address +size;
				flag_block = 1;
				
		}
		
		
	  for (i=0; i<size; i++)
		{			
						
				spi_cs_high();  // Close CS 
				spi_cs_low();   // Open CS
							
				spi_access(0x02); //Command for write programming
				spi_access((uint8_t)(address_temp>>16));
				spi_access((uint8_t)(address_temp>>8));
				spi_access((uint8_t)address_temp);
				spi_access(*write_ecg_ptr++);
				
				spi_cs_high();  // Close CS
			
				address_temp ++;
				// total_size++;
		}		
}

/**
****************************************************************************************
* @brief Receive an ECG message over the SPI
* @param[in] *ecg_ptr:  pointer to the position the received data will be stored
* @return Message type
****************************************************************************************
*/
void read_ecg_data_from_sram(uint32_t size, uint32_t address,uint8_t *read_ecg_ptr)
{
		uint32_t i = 0;
		uint32_t address_temp = 0;
    address_temp = address;
/**
****************************************************************************************
 Initialize SPI
****************************************************************************************
*/	
		
		RESERVE_GPIO(SPI_EN, GPIO_PORT_2, GPIO_PIN_5, PID_SPI_EN); 
    RESERVE_GPIO(SPI_CLK, GPIO_PORT_2, GPIO_PIN_8, PID_SPI_CLK);
    RESERVE_GPIO(SPI_DO, GPIO_PORT_2, GPIO_PIN_9, PID_SPI_DO);
    RESERVE_GPIO(SPI_DI, GPIO_PORT_2, GPIO_PIN_6, PID_SPI_DI);
    RESERVE_GPIO(EN_HOLD, GPIO_PORT_2, GPIO_PIN_7, PID_HOLD);
	
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_5, OUTPUT, PID_SPI_EN, true );
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_8, OUTPUT, PID_SPI_CLK, false );
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_9, OUTPUT, PID_SPI_DO, false );	
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_6, INPUT, PID_SPI_DI, false );
	  GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_7, OUTPUT, PID_GPIO, true );
	
    SPI_Pad_t cs_pad_param;
    cs_pad_param.port = GPIO_PORT_2;
    cs_pad_param.pin = GPIO_PIN_5;
    
    spi_init(&cs_pad_param, SPI_MODE_8BIT, SPI_ROLE_MASTER, SPI_CLK_IDLE_POL_LOW, SPI_PHA_MODE_0, SPI_MINT_DISABLE, SPI_XTAL_DIV_14);
		
	  if(((size + address) >= 0x010000 ) && ((address) <  0x010000) )
		{
				address_read = 0;
		}
		if(flag_block == 0)
		{
		    address_temp = 0x010000 + address_read;  
		}
		else
		{
		    address_temp = address_read + 0x000000;
		}	

		for(i=0; i<size ; i++)
		{
			  						
				spi_cs_high();
				spi_cs_low();            			            	// pull CS low    
			
				spi_access(0x03);             // Command for sequencial reading from memory		
				spi_access((uint8_t)(address_temp>>16));
				spi_access((uint8_t)(address_temp>>8));
				spi_access((uint8_t)address_temp);
				*read_ecg_ptr++ = spi_access(0x00);   // bare SPI transaction
		
				spi_cs_high();               			            // push CS high
			
				address_temp ++;			
		}
		address_read += size;
}
 

void app_ecg_trigger(void)
{    
    uint16_t idx = 0;
    uint32_t tmp = 0;
    uint8_t delay_idx = 0;
		uint32_t length = 200;
		// Initialize SPI
    RESERVE_GPIO(SPI_EN, GPIO_PORT_1, GPIO_PIN_3, PID_SPI_EN); 
    RESERVE_GPIO(SPI_CLK, GPIO_PORT_0, GPIO_PIN_4, PID_SPI_CLK);
    RESERVE_GPIO(SPI_DO, GPIO_PORT_0, GPIO_PIN_3, PID_SPI_DO);
    RESERVE_GPIO(SPI_DI, GPIO_PORT_0, GPIO_PIN_2, PID_SPI_DI);
        
    GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_3, OUTPUT, PID_SPI_EN, true );
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_4, OUTPUT, PID_SPI_CLK, false );
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_3, OUTPUT, PID_SPI_DO, false );	
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_2, INPUT, PID_SPI_DI, false );
    
    SPI_Pad_t cs_pad_param;
    cs_pad_param.port = GPIO_PORT_1;
    cs_pad_param.pin = GPIO_PIN_3;
    
    spi_init(&cs_pad_param, SPI_MODE_8BIT, SPI_ROLE_MASTER, SPI_CLK_IDLE_POL_LOW, SPI_PHA_MODE_1, SPI_MINT_DISABLE, SPI_XTAL_DIV_14);

    spi_cs_low();
    
    for (idx = 0; idx < length; idx++) {   
        for(delay_idx = 0; delay_idx < 20; delay_idx++) {
            __nop();
        }
        ecg_data[idx] = spi_access(tmp);
    }
    spi_cs_high(); 
    ecg_data_idx = 0;		
		spi_release();
    GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_3, OUTPUT, PID_GPIO, true );
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_4, OUTPUT, PID_GPIO, false );
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_3, OUTPUT, PID_GPIO, false );	
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_2, INPUT, PID_GPIO, false );
   //FIXME
		
			for(idx=0;idx<length;idx++)
		{
				ecg_data[idx] = idx;
		}
		
		write_ecg_data_to_sram(length,band_id, ecg_data); //band_id
		flag_start_read = 1;
}

void ecg_wakeup_cb(void)
{
   // ecg_trigger_flag = 1;
    
	if(GetBits16(SYS_STAT_REG, PER_IS_DOWN))   {
		periph_init();
	}
	  app_ecg_trigger(); 
		app_ecg_enable();
}


void app_ecg_enable(void)
{   
    RESERVE_GPIO(ECG, GPIO_PORT_2, GPIO_PIN_0, ECG_TRIGGER); 
		GPIO_SetPinFunction(GPIO_PORT_2, GPIO_PIN_0, INPUT, PID_GPIO);
    wkupct_register_callback(ecg_wakeup_cb);
   // if (GPIO_GetPinStatus( GPIO_PORT_2, GPIO_PIN_0 ))
    wkupct_enable_irq(1<<16, 0, 1, 0); // P2_0, polarity low , 1 event, debouncing time = 0
}

#if (STREAMDATA_QUEUE)

/**
 ****************************************************************************************
 * @brief Throughput test Function initialise application level buffers.
 *
 * @return void.
 ****************************************************************************************
*/

void test_pkt_init (void)
{
  int i;
  for (i=0;i<MAX_TX_BUFS;i++)
  {
    memset (&test_pkt[i] [0],0,2);
    memset (&test_pkt[i] [2],i,STREAMDATAD_PACKET_SIZE-2);
  }
  
}

/**
 ****************************************************************************************
 * @brief Callback function. Registered when packet is provided to stream queue. Called when packet is sent.
 *
 * @return void.
 ****************************************************************************************
*/

void test_callback (void* addr, int handle)
{
  volatile uint16 *val = (volatile uint16 *) addr;
  (*val)++;
}

 /**
 ****************************************************************************************
 * @brief Generate messages and provide to stream queue.
 *
 * @return void.
 ****************************************************************************************
*/
 
void test_pkt_gen (void)
{
//    uint32_t ecg_data_length = 200;  
  	static uint8_t pt=0;
    static int hnd=0;
	
	  if(check_packet_buffer_enable() > 0)
		{
			
					if(flag_start_read == 1)
					{
								flag_start_read = 0;
						
								// read_ecg_data_from_sram(ecg_data_length,address_read, read_ecg_data);//address_read
		
								pt = 0;
								while ((pt < 200) && stream_fifo_add (&ecg_data[pt], STREAMDATAD_PACKET_SIZE/2, STREAMDATAD_DIR_VAL_HANDLE(hnd), L2C_CODE_ATT_HDL_VAL_NTF, test_callback)>0)  //send as many as possible
								{
									pt += STREAMDATAD_PACKET_SIZE/2; 
									// hnd++; if (hnd>=STREAMDATAD_MAX) hnd=0;
								};
															 
					}
			 
	   }
}
#endif

uint8_t acc_spi_read(uint8_t addr)
{
    uint8_t acc_read_buffer;	
  	//Initialize SPI
		RESERVE_GPIO(SPI_EN, GPIO_PORT_1, GPIO_PIN_0, PID_SPI_EN); 
    RESERVE_GPIO(SPI_CLK, GPIO_PORT_0, GPIO_PIN_0, PID_SPI_CLK);
    RESERVE_GPIO(SPI_DO, GPIO_PORT_0, GPIO_PIN_6, PID_SPI_DO);
    RESERVE_GPIO(SPI_DI, GPIO_PORT_0, GPIO_PIN_5, PID_SPI_DI);

    GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_0, OUTPUT, PID_SPI_EN, true );
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_0, OUTPUT, PID_SPI_CLK, false );
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_6, OUTPUT, PID_SPI_DO, false );	
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_5, INPUT, PID_SPI_DI, false );
	 
    SPI_Pad_t cs_pad_param;
    cs_pad_param.port = GPIO_PORT_1;
    cs_pad_param.pin = GPIO_PIN_0;
    
    spi_init(&cs_pad_param, SPI_MODE_8BIT, SPI_ROLE_MASTER, SPI_CLK_IDLE_POL_LOW, SPI_PHA_MODE_0, SPI_MINT_DISABLE, SPI_XTAL_DIV_14); 
  						
		spi_cs_high();
		spi_cs_low();            			            	// pull CS low    
	
		spi_access(0x0B);             // Command for sequencial reading from memory		
		spi_access(addr);
	  acc_read_buffer = spi_access(0x00);   // bare SPI transaction

		spi_cs_high();               			            // push CS high
		
		return acc_read_buffer;
}

void acc_spi_write(uint8_t addr,uint8_t cmd)
{
    uint32_t idx;  
	//Initialize SPI
		RESERVE_GPIO(SPI_EN, GPIO_PORT_1, GPIO_PIN_0, PID_SPI_EN); 
    RESERVE_GPIO(SPI_CLK, GPIO_PORT_0, GPIO_PIN_0, PID_SPI_CLK);
    RESERVE_GPIO(SPI_DO, GPIO_PORT_0, GPIO_PIN_6, PID_SPI_DO);
    RESERVE_GPIO(SPI_DI, GPIO_PORT_0, GPIO_PIN_5, PID_SPI_DI);

    GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_0, OUTPUT, PID_SPI_EN, true );
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_0, OUTPUT, PID_SPI_CLK, false );
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_6, OUTPUT, PID_SPI_DO, false );	
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_5, INPUT, PID_SPI_DI, false );
	 
    SPI_Pad_t cs_pad_param;
    cs_pad_param.port = GPIO_PORT_1;
    cs_pad_param.pin = GPIO_PIN_0;
    
    spi_init(&cs_pad_param, SPI_MODE_8BIT, SPI_ROLE_MASTER, SPI_CLK_IDLE_POL_LOW, SPI_PHA_MODE_0, SPI_MINT_DISABLE, SPI_XTAL_DIV_14); 
  						
		spi_cs_high();
		spi_cs_low();            			            	// pull CS low    
	
		spi_access(0x0A);             // Command for sequencial reading from memory		
		spi_access(addr);
		spi_access(cmd);   // bare SPI transaction

		spi_cs_high();               			            // push CS high
		for(idx = 0;idx<1000;idx++)
	  {
		    __nop();
			  __nop();
		}
}

void acc_fifo_read(uint8_t *acc_fifo_buff_ptr,uint8_t len)
{
    static uint8_t idx;		
  	//Initialize SPI
		RESERVE_GPIO(SPI_EN, GPIO_PORT_1, GPIO_PIN_0, PID_SPI_EN); 
    RESERVE_GPIO(SPI_CLK, GPIO_PORT_0, GPIO_PIN_0, PID_SPI_CLK);
    RESERVE_GPIO(SPI_DO, GPIO_PORT_0, GPIO_PIN_6, PID_SPI_DO);
    RESERVE_GPIO(SPI_DI,  GPIO_PORT_0, GPIO_PIN_5, PID_SPI_DI);

    GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_0, OUTPUT, PID_SPI_EN, true );
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_0, OUTPUT, PID_SPI_CLK, false );
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_6, OUTPUT, PID_SPI_DO, false );	
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_5, INPUT, PID_SPI_DI, false );
	 
    SPI_Pad_t cs_pad_param;
    cs_pad_param.port = GPIO_PORT_1;
    cs_pad_param.pin = GPIO_PIN_0;
    
    spi_init(&cs_pad_param, SPI_MODE_8BIT, SPI_ROLE_MASTER, SPI_CLK_IDLE_POL_LOW, SPI_PHA_MODE_0, SPI_MINT_DISABLE, SPI_XTAL_DIV_14); 
  						
		spi_cs_high();
		spi_cs_low();            			            	// pull CS low    
	
		spi_access(0x0D);             // Command for sequencial reading from memory		
		for(idx=0;idx<len;idx++)
		{
	      *acc_fifo_buff_ptr++ = spi_access(0x00);   // bare SPI transaction
    }
		spi_cs_high();               			            // push CS high
}

void acc_initial(void)
{
    // Initialize ACC registers
	  acc_spi_write(0x2D, 0x00);
    acc_spi_write(0x1F, 0x52);
	
}

void test_acc_callback (void* addr, int handle)
{
  volatile uint16 *val = (volatile uint16 *) addr;
  (*val)++;
}

void acc_timeout_handler(void)
{
    // uint16_t acc_fifo_sample_cnt;
    uint8_t pkt_cnt;  
	  static uint8_t pt= 0;
    static int hnd= 3;
	 //test function
//    test_acc = acc_spi_read(0x00);
//	  test_acc_id = acc_spi_read(0x0C);
//	  acc_spi_write(0x2B, 0x02);
	  if (acc_started == 0) 
		{
				if (acc_spi_read(0x00) == 0xAD) 
				{
						acc_spi_write(0x2D, 0x00);// turn off;0x2D ->POWER CONTROL REGISTER ;0x00 -> standby;
						acc_spi_write(0x28, 0x02);// configure FIFO;0x28 ->FIFO CONTROL REGISTER; 0x02 -> Stream mode;
						acc_spi_write(0x2C, 0x11);// configure general device settings; 0x2C ->FLITER CONTROL REGISTER ;0x11 -> ODR(output data rate) 100Hz(10ms)
						acc_spi_write(0x2D, 0x22);// turn measurement on ;0x2D ->POWER CONTROL REGISTER ;0x22 -> Ultralow noise mode & Measurement mode;
				} 
				else
				{
					return;
				}
				acc_started = 1;
    }
		// 0x0C FIFO_ENTRIES_L  0x0D FIFO_ENTRIES_H  the number of  valid data samples present in the FIFO buffer.ranges from 0 to 512;
	  acc_fifo_sample_cnt = ((uint16_t)(acc_spi_read(0x0D) & 0x03) << 8) | ((uint16_t)acc_spi_read(0x0C)); 
    // Get 3 samples into one pkt, three packts each timeout period
    if (acc_fifo_sample_cnt > 0) 
		{
				pkt_cnt = 0;
				while ((pkt_cnt <= 10) && (acc_fifo_sample_cnt > 0)) 
				{
						if (acc_fifo_sample_cnt >= (10- acc_sample_cnt)) 
						{
								acc_fifo_read(&(acc_data[acc_sample_cnt * 2]), 2 * (10 - acc_sample_cnt));
								acc_fifo_sample_cnt -= (10 - acc_sample_cnt);
								acc_sample_cnt = 0;
								// Send value if connected and notifying
							 
							  if(check_acc_buffer_enable() > 0)//
								{
											pt = 0;
										
											while ((pt < 20) && stream_fifo_add (&acc_data[pt], STREAMDATAD_PACKET_SIZE/2, STREAMDATAD_DIR_VAL_HANDLE(hnd), L2C_CODE_ATT_HDL_VAL_NTF, test_acc_callback) >= 0)  //send as many as possible
											{
													pt += STREAMDATAD_PACKET_SIZE/2;
													// hnd++; if (hnd>=STREAMDATAD_MAX) hnd=0;
											};
											pkt_cnt++;
							  }
							 							
					 }     
					 else 
					 {
							 acc_fifo_read(&(acc_data[acc_sample_cnt * 2]), 2 * acc_fifo_sample_cnt);
							 acc_sample_cnt += acc_fifo_sample_cnt;
							 acc_fifo_sample_cnt = 0;
					 }
			  }
    }
}


void ble_acc_timer_callback(void)
{
    static uint8_t n=0;
    if (n==10)
    {
        n=0;
			  timer0_set(3000, 1500, 1500);
			  acc_timeout_handler();  			
    }
    n++;		
}

void app_acc_timer0_interrupt(void)
{
	 RESERVE_GPIO(ACCELEROMETER, GPIO_PORT_2, GPIO_PIN_3, ACC_TRIGER); 
	 GPIO_SetPinFunction(GPIO_PORT_2, GPIO_PIN_3, OUTPUT, PID_PWM0);    
   
    /* TIMER0 */
    //Enables TIMER0 clock
    set_tmr_enable(CLK_PER_REG_TMR_ENABLED);
	
    // initilalize PWM with the desired settings
    timer0_init(TIM0_CLK_32K, PWM_MODE_ONE, TIM0_CLK_NO_DIV);
  
    // set pwm Timer0 On, Timer0 'high' and Timer0 'low' reload values
    timer0_set(3000, 1500, 1500);
    
    // register callback function for SWTIM_IRQn irq
    timer0_register_callback(ble_acc_timer_callback);
    
    // enable SWTIM_IRQn irq
    timer0_enable_irq();
     
    // start pwm0
    timer0_start();
 
}

#endif  //BLE_APP_PRESENT
/// @} APP
