/**
****************************************************************************************
*
* @file app_task.c
*
* @brief Handling of ble events and responses.
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

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app.h" 
#include "app_button_led.h"
#include "gap_task.h" 
#include "proxr_task.h" 
#include "proxr.h"
#include "llc_task.h" 
#include "smpc_task.h" 
#include "ble_msg.h" 

														
extern unsigned int proxm_trans_in_prog;

//application allert state structrure
app_alert_state alert_state;

/**
 ****************************************************************************************
 * @brief Extracts device name from adv data if present and copy it to app_env.
 *
 * @param[in] adv_data		Pointer to advertise data.
 * @param[in] adv_data_len  Advertise data length.
 * @param[in] dev_indx		Devices index in device list.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
void app_find_device_name(unsigned char * adv_data, unsigned char adv_data_len, unsigned char dev_indx)
{
	unsigned char indx = 0;

	while (indx < adv_data_len)
	{
		if (adv_data[indx+1] == 0x09)
		{
			memcpy(app_env.devices[dev_indx].data, &adv_data[indx+2], (size_t) adv_data[indx]);
			app_env.devices[dev_indx].data_len = (unsigned char ) adv_data[indx];
		}
		indx += (unsigned char ) adv_data[indx]+1;
	}

}

/**
 ****************************************************************************************
 * @brief Handles Set Mode completion.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
/*Wei
int gap_set_mode_req_cmp_evt_handler(ke_msg_id_t msgid,
                                            struct gap_event_common_cmd_complete *param,
                                            ke_task_id_t dest_id,
                                            ke_task_id_t src_id)
{
	if (param->status ==  CO_ERROR_NO_ERROR)
	{
		app_env.state = APP_SCAN;

		Sleep(100);

		app_inq();	//start scanning

		ConsoleScan();
	}

    return (KE_MSG_CONSUMED);
}
*/
int gapm_set_mode_req_evt_handler(ke_msg_id_t msgid,
                                            struct gapm_cmp_evt *param,
                                            ke_task_id_t dest_id,
                                            ke_task_id_t src_id)
{
	app_env.state = APP_CONNECTABLE;

	//Sleep(100);

	app_adv_start();	//start advertising

	//ConsoleScan();

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles ready indication from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int gap_ready_evt_handler(ke_msg_id_t msgid,
                               void *param,
                               ke_task_id_t dest_id,
                               ke_task_id_t src_id)
{

    // We are now in Connectable State
	if (dest_id == TASK_APP)
	{

		//Proximity service in the DB
		//app_set_keys();
#if 0
		alert_state.ll_alert_lvl = 2;//PROXR_ALERT_HIGH;//Link Loss default Alert Level
		alert_state.adv_toggle = 0; //clear advertise toggle
		app_proxr_db_create();
#endif
		app_rst_gap();

	}
	
    return 0;
}

/**
 ****************************************************************************************
 * @brief Handles Inquiry Request completion.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int gap_dev_inq_req_cmp_evt_handler(ke_msg_id_t msgid,
                                      void *param,
                                      ke_task_id_t dest_id,
                                      ke_task_id_t src_id)
{
	
	app_env.state = APP_IDLE;
			
	//ConsoleIdle();
	
	return 0;
}

/**
 ****************************************************************************************
 * @brief Handles Inquiry result event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int gap_dev_inq_result_handler(ke_msg_id_t msgid,
                                      struct gapm_adv_report_ind *param,
                                      ke_task_id_t dest_id,
                                      ke_task_id_t src_id)
{
	 
	if (app_env.state != APP_SCAN)
		return -1;

    //app_adv_start();	//start advertising

/*    if (param->nb_resp >0)
	{
		recorded = app_device_recorded(&param->adv_rep.adv_addr); 

		if (recorded <MAX_SCAN_DEVICES) //update Name
		{
			app_find_device_name(param->adv_rep.data,param->adv_rep.data_len, recorded); 
			ConsoleScan();
		}
		else
		{
			app_env.devices[app_env.num_of_devices].free = false;
			app_env.devices[app_env.num_of_devices].rssi = param->adv_rep.rssi;
			memcpy(app_env.devices[app_env.num_of_devices].adv_addr.addr, param->adv_rep.adv_addr.addr, BD_ADDR_LEN );
			app_find_device_name(param->adv_rep.data,param->adv_rep.data_len, app_env.num_of_devices); 
			ConsoleScan();
            app_env.num_of_devices++;
		}		
	}	*/
			
    return 0;
}


/**
 ****************************************************************************************
 * @brief Handles Connection request completion event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
 unsigned int start_pair;
int gap_le_create_conn_req_ind_handler(ke_msg_id_t msgid,
                                                  struct gapc_connection_req_ind *param,
                                                  ke_task_id_t dest_id,
                                                  ke_task_id_t src_id)
{

  //start_pair = 1;

  if (app_env.state == APP_IDLE || app_env.state == APP_CONNECTABLE)
  {
	// Check the status

	// We are now connected
		turn_off_led();
				
		app_env.state = APP_CONNECTED;

		// Retrieve the connection info from the parameters
		app_env.proxr_device.device.conhdl = param->conhdl;
/*
		// On Reconnection check if device is bonded and send pairing request. Otherwise it is not bonded.
		if (bdaddr_compare(&app_env.proxr_device.device.adv_addr, &param->conn_info.peer_addr))
		{
			if (app_env.proxr_device.bonded)
				start_pair = 0;
		}
*/				
		memcpy(app_env.proxr_device.device.adv_addr.addr, param->peer_addr.addr, sizeof(struct bd_addr));
		app_env.proxr_device.rssi = 0xFF;
		app_env.proxr_device.txp = 0xFF;
		app_env.proxr_device.llv = 0xFF;						

		// Send a request to read the LLC TX power so that we can update the PROXR service database.
		{
			struct llc_rd_tx_pw_lvl_cmd * req;
					
			req = (struct llc_rd_tx_pw_lvl_cmd *) BleMsgAlloc(LLC_RD_TX_PW_LVL_CMD, TASK_LLC, TASK_APP,
												sizeof(struct llc_rd_tx_pw_lvl_cmd));
		
			req->conhdl = app_env.proxr_device.device.conhdl;
			req->type = TX_LVL_CURRENT;

			// Send the message
			BleSendMsg((void *) req);
		}

		app_connect_confirm(GAP_AUTH_REQ_NO_MITM_NO_BOND);

		//START PAIRING REMOVED FROM HERE
	}

    return 0;
}


/**
 ****************************************************************************************
 * @brief Handles Discconnection completion event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gap_discon_cmp_evt_handler(ke_msg_id_t msgid,
                                  struct gapc_disconnect_ind *param,
                                  ke_task_id_t dest_id,
                                  ke_task_id_t src_id)
{
//	struct gapc_disconnect_ind *msg;

	if (param->conhdl == app_env.proxr_device.device.conhdl)
    {

	   app_send_disconnect(TASK_PROXR, param->conhdl, param->reason);

		app_env.state = APP_IDLE;

//		Sleep(100);

//		printf("Device Disconnected\n");

		app_set_mode();

		
    }

    return 0;
}

/**
 ****************************************************************************************
 * @brief Handles RSSI read request completion event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int gap_read_rssi_cmp_evt_handler(ke_msg_id_t msgid,
                                   struct gapc_con_rssi_ind /*gap_read_rssi_req_cmp_evt*/ *param,
                                  ke_task_id_t dest_id,
                                  ke_task_id_t src_id)
{

	//GZ if (param->status == CO_ERROR_NO_ERROR)
	{
		app_env.proxr_device.rssi = param->rssi;
//		ConsoleConnected(1);
	}

	return 0;
}


/**
 ****************************************************************************************
 * @brief Handle Bond indication.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gap_bond_ind_handler(ke_msg_id_t msgid,
                                    struct gapc_bond_ind *param,
                                    ke_task_id_t dest_id,
                                    ke_task_id_t src_id)
{
    
	switch (param->info)
	{
		case GAPC_PAIRING_SUCCEED:
			if (param->data.auth | GAP_AUTH_BOND)
				app_env.proxr_device.bonded = 1;
//				ConsoleConnected(0);
			
			break;
		case GAPC_IRK_EXCH:
			memcpy (app_env.proxr_device.irk.irk.key, param->data.irk.irk.key, KEY_LEN);
			memcpy (app_env.proxr_device.irk.addr.addr.addr, param->data.irk.addr.addr.addr, KEY_LEN);
			app_env.proxr_device.irk.addr.addr_type = param->data.irk.addr.addr_type;
			
			break;
		case GAPC_LTK_EXCH:
			app_env.proxr_device.ltk.ediv = param->data.ltk.ediv;
			memcpy (app_env.proxr_device.ltk.randnb.nb, param->data.ltk.randnb.nb, RAND_NB_LEN);
			app_env.proxr_device.ltk.key_size = param->data.ltk.key_size;
			memcpy (app_env.proxr_device.ltk.ltk.key, param->data.ltk.ltk.key, param->data.ltk.key_size);
			app_env.proxr_device.ltk.ediv = param->data.ltk.ediv;
			memcpy (app_env.proxr_device.ltk.randnb.nb, param->data.ltk.randnb.nb, RAND_NB_LEN);
			break;
		case GAPC_PAIRING_FAILED:
			app_env.proxr_device.bonded = 0;
			app_disconnect();
			break;
	}

    return 0;
}
 

/**
 ****************************************************************************************
 * @brief Handle reset GAP request.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_reset_req_evt_handler(ke_msg_id_t msgid,
                                    struct gapm_cmp_evt *param,
                                    ke_task_id_t dest_id,
                                    ke_task_id_t src_id)
{
    
	// We are now in Connectable State
	if (dest_id == TASK_APP)
	{
		app_env.state = APP_IDLE;			
		alert_state.ll_alert_lvl = 2;//PROXR_ALERT_HIGH;//Link Loss default Alert Level
		alert_state.adv_toggle = 0; //clear advertise toggle
        app_proxr_db_create();

		//app_set_mode(); //initialize gap mode 

	}
	
    return 0;
}

#if 0 
int  resource_access_rsp_handler(ke_msg_id_t msgid,
                                   struct gatt_resource_access_req *param,
                                   ke_task_id_t dest_id,
                                   ke_task_id_t src_id)
{
    // Allocate the message
    struct gatt_resource_access_rsp * req = BleMsgAlloc(GATT_RESOURCE_ACCESS_RSP, TASK_GATT, TASK_APP,
                                             sizeof(struct gatt_resource_access_rsp));

	req->conhdl = param->conhdl;
    
    // Send the message
    BleSendMsg(req);
	
	return 0;
}
#endif

/**
 ****************************************************************************************
 * @brief 
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gap_bond_req_ind_handler(ke_msg_id_t msgid,
                                    struct gapc_bond_req_ind *param,
                                    ke_task_id_t dest_id,
                                    ke_task_id_t src_id)
{
    
	app_gap_bond_cfm(param);
	
    return 0;
}

int gapc_encrypt_req_ind_handler(ke_msg_id_t const msgid,
									struct gapc_encrypt_req_ind * param,
									ke_task_id_t const dest_id,
									ke_task_id_t const src_id)
{

	struct gapc_encrypt_cfm* cfm = BleMsgAlloc(GAPC_ENCRYPT_CFM, src_id, dest_id, sizeof (struct gapc_encrypt_cfm));

	if(((app_env.proxr_device.bonded)
		&& (memcmp(&(app_env.proxr_device.ltk.randnb), &(param->rand_nb), RAND_NB_LEN) == 0)
		&& (app_env.proxr_device.ltk.ediv == param->ediv)))
    {
        cfm->found = true;
		cfm->key_size = app_env.proxr_device.ltk.key_size;
		memcpy(&(cfm->ltk), &(app_env.proxr_device.ltk.ltk), app_env.proxr_device.ltk.key_size);
        // update connection auth
        app_connect_confirm(GAP_AUTH_REQ_NO_MITM_BOND);
    }
    else
    {
        cfm->found = false;
    }

    BleSendMsg(cfm);

    return (KE_MSG_CONSUMED);

}


int gapc_encrypt_ind_handler(ke_msg_id_t const msgid,
                             struct gapc_encrypt_ind *param,
						     ke_task_id_t const dest_id,
							 ke_task_id_t const src_id)
{
    //app_param_update_start();
//	printf("Received GAPC_ENCRYPT_IND auth = %d\n", param->auth);

    return (KE_MSG_CONSUMED);
}

#if 0
int smpc_ltk_req_ind_handler(ke_msg_id_t const msgid,
                                      struct smpc_ltk_req_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{

		uint8_t random_nb[RAND_NB_LEN] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x06, 0x77};
	
        struct smpc_ltk_req_rsp *rsp;

    if (app_env.state == APP_CONNECTED)
	{
		rsp = (struct smpc_ltk_req_rsp *) BleMsgAlloc(SMPC_LTK_REQ_RSP, src_id, TASK_APP, sizeof(struct smpc_ltk_req_rsp));
		
		rsp->status = CO_ERROR_NO_ERROR;
		rsp->idx = param->idx;
		rsp->sec_prop = SMP_KSEC_UNAUTH_NO_MITM;
		memcpy(rsp->ltk.key , "DA14580_BND_TEST", 16);
		rsp->ediv = 0x4321; 
		memcpy (rsp->nb.nb, random_nb, RAND_NB_LEN);  

//		printf("Send SMPC_LTK_REQ_RSP\n");
        // Send the message
        BleSendMsg(rsp);
    }

    return 0;
}
#endif 

/**
 ****************************************************************************************
 * @brief Handles Read Tx Power Level response from llc task.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int llc_rd_tx_pw_lvl_cmp_evt_handler(ke_msg_id_t const msgid,
                                      struct llc_rd_tx_pw_lvl_cmd_complete const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
	{
		if (app_env.proxr_device.device.conhdl == param->conhdl)  
			alert_state.txp_lvl = param->tx_pow_lvl;
		else
			alert_state.txp_lvl = 0x00;
	}
			
    // Enable the Proximity Reporter profile
	app_proxr_enable();
    
    return (KE_MSG_CONSUMED);
}

int app_create_db_cfm_handler(ke_task_id_t const dest_id)
{
    // If state is not idle, ignore the message
    if (app_env.state == APP_IDLE)
    {						
										
		app_set_mode(); //initialize gap mode 

    }

    return (KE_MSG_CONSUMED);

}

void app_proxr_alert_ind(ke_msg_id_t msgid,
                         struct proxr_alert_ind *param,
                         ke_task_id_t dest_id,
                         ke_task_id_t src_id)
{
	
	if (param->alert_lvl == PROXR_ALERT_NONE)
		turn_off_led();
	else if (param->alert_lvl == PROXR_ALERT_MILD)
		green_blink_slow();
	else if (param->alert_lvl == PROXR_ALERT_HIGH)
		green_blink_fast();
}
