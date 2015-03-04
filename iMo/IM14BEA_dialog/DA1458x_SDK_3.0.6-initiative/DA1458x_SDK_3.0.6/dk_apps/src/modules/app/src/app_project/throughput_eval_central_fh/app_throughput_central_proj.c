/**
****************************************************************************************
*
* @file app_template_proj.c
*
* @brief Template project source code .
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
#include "app_throughput_central_proj.h"

#include "co_math.h"                 // Common Maths Definition
#include "gapc.h"
#if (NVDS_SUPPORT)
#include "nvds.h"                    // NVDS Definitions
#endif //(NVDS_SUPPORT)

#include "l2cc_task.h"

#if (STREAMDATA_QUEUE)
#include "app_stream_queue.h"
#endif

#if !BLE_INTEGRATED_HOST_GTL
uint8_t periph_found __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY
uint8_t periph_addr_type __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY
uint8_t periph_addr[BD_ADDR_LEN] __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY
#endif

uint8_t test_state __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

#define MAX_TX_BUFS (32)
#define STREAMDATAD_PACKET_SIZE (20)
uint8 test_pkt [MAX_TX_BUFS][STREAMDATAD_PACKET_SIZE];
int hnd;


extern uint16_t min_hdl, max_hdl;

extern struct metrics global_metrics_tosend;

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
#ifdef METRICS        
          init_metrics ();
#endif
#endif
        
#if BLE_STREAMDATA_HOST        
        app_streamdatah_enable();
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
    cmd->intv_min    = APP_ADV_INT_MIN;
    cmd->intv_max    = APP_ADV_INT_MAX;
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
#else
        app_start_scanning();
#endif                
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
/****************************************************            
            #if (BLE_DIS_SERVER)
            case (APP_DIS_TASK):
            {
                app_dis_create_db_send();
            } break;
            #endif //BLE_DIS_SERVER
******************************************************/            
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
    cmd->role = GAP_CENTRAL_MST;
	
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
    ke_state_set(TASK_APP, APP_CONNECTABLE);
    
#if BLE_INTEGRATED_HOST_GTL    
	struct app_ready_ind *ind = KE_MSG_ALLOC(APP_READY_IND, TASK_GTL, TASK_APP, app_ready_ind);
   
	ke_msg_send(ind);
#else	
    app_start_scanning();
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

/**************************************************
Database created. Ready to start Application i.e. start advertise
***************************************************/        
    
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
    
#if BLE_INTEGRATED_HOST_GTL    
            /* EXT Processor - TASK APP over UART */
            /* send scan completed indication to external app*/
            void *cmd = ke_msg_alloc( APP_EXT_SCAN_CMP_IND, TASK_GTL, TASK_APP, 0);

            // Send the message
            ke_msg_send(cmd);
#else 

        if (periph_found)
        {
            app_connect(periph_addr_type, periph_addr, 80);
        }
        else
        {
            app_start_scanning();
        }
    
#endif
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
   
    
    msg->ce_len_min = 0x20;
    msg->ce_len_max = 0x20;
    msg->con_latency = 0;
    msg->op.addr_src = GAPM_PUBLIC_ADDR;
    
    msg->superv_to = 0x1F4;// 500 -> 5000 ms ;
    msg->scan_interval = 0x180;
    msg->scan_window = 0x160;
    msg->op.code = GAPM_CONNECTION_DIRECT;
    
    app_timer_set(APP_CONN_TIMER, TASK_APP, 700);
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
    
#if  BLE_INTEGRATED_HOST_GTL
    struct app_ext_adv_report_ind *cmd = KE_MSG_ALLOC(APP_EXT_ADV_REPORT_IND,
                                               TASK_GTL, TASK_APP, app_ext_adv_report_ind);

    cmd->addr_type =  param->report.adv_addr_type;
    memcpy ((void *)cmd->addr, (void *)&param->report.adv_addr, BD_ADDR_LEN);
		
    // Send the message
    ke_msg_send(cmd);
#else
    periph_found = 1;
    periph_addr_type = param->report.adv_addr_type;
    memcpy ((void *)periph_addr, (void *)&param->report.adv_addr, BD_ADDR_LEN);
    
    //cancel scan procedure
    struct gapm_cancel_cmd *cmd = KE_MSG_ALLOC(GAPM_CANCEL_CMD,
                                               TASK_GAPM, TASK_APP,
                                               gapm_cancel_cmd);

    cmd->operation = GAPM_CANCEL;

    // Send the message
    ke_msg_send(cmd);
#endif
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
    
#if  BLE_INTEGRATED_HOST_GTL     
    struct app_ext_connect_ind *cmd = KE_MSG_ALLOC(APP_EXT_CONNECT_CMP_IND,
                                               TASK_GTL, TASK_APP, app_ext_connect_ind);
		
    cmd->state = false;
    memset(&cmd->addr_type, 0, 8);
    // Send the message
    ke_msg_send(cmd);
#else
    app_start_scanning();
#endif
    
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

/**
 ****************************************************************************************
 * @brief Send Inquiry (devices discovery) request to GAP task.
 *
 * @return void.
 ****************************************************************************************
 */
void app_start_scanning(void)
{
    
#if !BLE_INTEGRATED_HOST_GTL
    periph_found = 0;
#endif    
    
    struct gapm_start_scan_cmd *msg = KE_MSG_ALLOC(GAPM_START_SCAN_CMD, TASK_GAPM, TASK_APP, gapm_start_scan_cmd);

    msg->mode = GAP_GEN_DISCOVERY;
    msg->op.code = GAPM_SCAN_PASSIVE;
    msg->op.addr_src = GAPM_PUBLIC_ADDR;
    msg->filter_duplic = SCAN_FILT_DUPLIC_EN;
    msg->interval = 10;
    msg->window = 5;

    ke_msg_send(msg);

    return;
}

/**
 ****************************************************************************************
 * @brief Handles connection timer expiration. Connection request timedout. Cancel connection procedure.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int app_conn_timer_handler(ke_msg_id_t const msgid,
									void *param,
									ke_task_id_t const dest_id,
									ke_task_id_t const src_id)
{
    
    struct gapm_cancel_cmd *cmd = KE_MSG_ALLOC(GAPM_CANCEL_CMD,
                                               TASK_GAPM, TASK_APP,
                                               gapm_cancel_cmd);

    cmd->operation = GAPM_CANCEL;

    // Send the message
    ke_msg_send(cmd);
            
    return (KE_MSG_CONSUMED);
}

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
 * @brief Handles reception of scan command from external processor application. Starts scanning for peripherals
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int app_ext_scan_cmd_handler(ke_msg_id_t const msgid,
                             const void *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
	app_start_scanning();

	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of connect command from external processor application. Connects to peripherals
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int app_ext_connect_cmd_handler(ke_msg_id_t const msgid,
                             const struct app_ext_connect_cmd *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    app_connect(param->addr_type, (uint8_t*) &param->addr[0], param->intv);
    
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
    
        
        //set_pxact_gpio();

#ifdef METRICS
        struct app_ext_stats_report *cmd = KE_MSG_ALLOC(APP_EXT_STATS_REPORT,
                                                   TASK_GTL, TASK_APP, app_ext_stats_report);

        //memset(cmd, 0, sizeof(struct app_ext_stats_report));

        cmd->type =  0;
        cmd->conn_handle =  app_env.conhdl;
        memcpy(&cmd->timeperiod, &global_metrics_tosend, sizeof(uint16_t) * 6);
                
        // Send the message
        ke_msg_send(cmd);
#endif

    }
}
#endif //BLE_INTEGRATED_HOST_GTL


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
  
  hnd = min_hdl;
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
  static int pt=0;
    
  if (test_state == APP_STREAM_START)
  {    
      while (stream_fifo_add(&test_pkt [pt], STREAMDATAD_PACKET_SIZE, hnd, L2C_CODE_ATT_WR_CMD, test_callback) >= 0)  //send as many as possible
      {
        pt++; if (pt>=MAX_TX_BUFS) pt=0;
        hnd+=4; if (hnd>max_hdl) hnd=min_hdl;
        
      };
  }
}
#endif

#endif  //BLE_APP_PRESENT
/// @} APP
