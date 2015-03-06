/**
 ****************************************************************************************
 *
 * @file rwble.c
 *
 * @brief Entry points the BLE software
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "arch.h"    
#include <string.h>
#include "co_version.h"
#include "rwble.h"
#if BLE_HCIC_ITF
#include "hcic.h"
#endif //BLE_HCIC_ITF
#include "ke_event.h"
#include "ke_timer.h"
#include "co_buf.h"
#include "lld.h"
#include "llc.h"
#include "llm.h"
#include "dbg.h"
#include "lld_evt.h"
#include "reg_blecore.h"

#if (NVDS_SUPPORT)
#include "nvds.h"         // NVDS definitions
#endif // NVDS_SUPPORT

#include "gtl_eif.h"
#include "uart.h"
#include "reg_ble_em_rx.h"
#include "ke_mem.h"
#include "pll_vcocal_lut.h"
#include "gpio.h"
#include "rf_580.h"
#include "periph_setup.h"
#include "arch_sleep.h"

uint8_t rwble_last_event __attribute__((section("retention_mem_area0"), zero_init));

extern uint32_t lp_clk_sel;
/*
 * FORWARD DECLARATION OF GLOBAL FUNCTIONS
 ****************************************************************************************
 */
void ble_regs_pop (void);


/*
 * GLOBAL VARIABLES DECLARATION
 ****************************************************************************************
 */
extern uint8_t func_check_mem_flag;

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
 #if (!BLE_HOST_PRESENT)
/**
 ****************************************************************************************
 * @brief Initializes the diagnostic port.
 ****************************************************************************************
 */
static void rwble_diagport_init(void)
{
    #if (NVDS_SUPPORT)
    uint8_t diag_cfg[NVDS_LEN_DIAG_BLE_HW];
    uint8_t length = NVDS_LEN_DIAG_BLE_HW;

    // Read diagport configuration from NVDS
    if(nvds_get(NVDS_TAG_DIAG_BLE_HW, &length, diag_cfg) == NVDS_OK)
    {
        ble_diagcntl_pack(1, diag_cfg[3], 1, diag_cfg[2], 1, diag_cfg[1], 1, diag_cfg[0]);
    }
    else
    #endif // NVDS_SUPPORT
    {
        ble_diagcntl_set(0);
    }
}
 
void rwble_init(void)
{
    #if RW_BLE_SUPPORT
    // Initialize buffers
    co_buf_init();
    #endif // #if RW_BLE_SUPPORT

    #if (BLE_HCIC_ITF)
    // In split mode, initialize the HCI
    hci_init(rwip_eif_get(RWIP_EIF_HCIC));
    #endif //BLE_HCIC_ITF

    // Initialize the Link Layer Driver
    lld_init(false);

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Initialize the Link Layer Controller
    llc_init();
    #endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */
    // Initialize the Link Layer Manager
    llm_init();

    #if RW_BLE_SUPPORT
    // Initialize the debug process
    dbg_init();
    #endif //RW_BLE_SUPPORT

    // Initialize diagport - for test and debug only
    rwble_diagport_init();

    #if BLE_HOST_PRESENT
    // Signal the completion of the boot process to the host layers
    llm_ble_ready();
    #endif //BLE_HOST_PRESENT
}

#endif

#if RW_BLE_SUPPORT && HCIC_ITF

void rwble_send_message(uint32_t error)
{
    // Send HCI_Hw_Error_Evt message
    struct llm_hw_error_evt *evt;

    if(error == RESET_TO_ROM || error == RESET_AND_LOAD_FW)
    {
        // Send platform reset command complete if requested by user
        dbg_platform_reset_complete(error);
    }
    else
    {
        // Allocate a message structure for hardware error event
        evt = (struct llm_hw_error_evt *)KE_MSG_ALLOC(LLM_HW_ERROR_EVT, HOST_GAP_TASK,
                TASK_LLM, llm_hw_error_evt);

        // Fill the HW error code
        switch(error)
        {
            case RESET_MEM_ALLOC_FAIL: evt->hw_code = CO_ERROR_HW_MEM_ALLOC_FAIL; break;
            default: ASSERT_INFO(0, error, 0); break;
        }

        // Send the message
        ke_msg_send(evt);
    }
}

#endif //RW_BLE_SUPPORT && HCIC_ITF
 

#if (STREAMDATA_QUEUE) 
unsigned char cpt_event = 0;
int transmitting_data = 0;

#ifdef METRICS
void add_err_packet();

void measure_errors_received(unsigned char pkts)
{
    uint8_t rx_cnt = pkts;
    uint8_t rx_hdl = co_buf_rx_current_get();
    struct lld_evt_tag *evt;
    evt = (struct lld_evt_tag *)co_list_pick(&lld_evt_env.evt_prog);
    ke_task_id_t destid = (evt->conhdl == LLD_ADV_HDL) ? TASK_LLM : KE_BUILD_ID(TASK_LLC, evt->conhdl);
    uint16_t conhdl = KE_IDX_GET(destid);

    while(rx_cnt--)
    {
        struct co_buf_rx_desc *rxdesc = co_buf_rx_get(rx_hdl);
        unsigned char status = rxdesc->rxstatus & 0x7F;
        if ((status & (BLE_MIC_ERR_BIT | BLE_CRC_ERR_BIT | BLE_LEN_ERR_BIT | BLE_TYPE_ERR_BIT | BLE_SYNC_ERR_BIT))\
           && (llc_env[conhdl]->rssi > llm_get_min_rssi()))
        {
            add_err_packet();
        }
        rx_hdl = co_buf_rx_next(rx_hdl);
    }
}
#endif
#endif //(STREAMDATA_QUEUE) 



// ============================================================================================
// ==================== DEEP SLEEP PATCH - THIS CODE MUST STAY IN RAM =========================
// ============================================================================================
extern void rf_workaround_init(void);
extern void rf_reinit(void);

void lld_sleep_compensate_func_patched(void)
{
    uint32_t dur_us;
    uint32_t slot_corr;
    uint32_t fine_corr;
    
    // Get the number of low power sleep period
    uint32_t slp_period = ble_deepslstat_get();

    // Sanity check: The duration of a sleep is limited
    ASSERT_ERR(slp_period < LLD_EVT_MAX_SLEEP_DURATION);

    // Convert sleep duration into us
    dur_us = lld_sleep_lpcycles_2_us_sel_func(slp_period);
    slot_corr = dur_us / 625;
    fine_corr = 624 - (dur_us % 625);
    if (fine_corr == 0)
        fine_corr = 1;

    // The correction values are then deduced from the sleep duration in us
    ble_basetimecntcorr_set(slot_corr);
    ble_finecntcorr_set(fine_corr);

    // Start the correction
    ble_deep_sleep_corr_en_setf(1);
}



/*********************************************************************************
 *** WAKEUP_LP_INT ISR
 ***/
void BLE_WAKEUP_LP_Handler(void)
{    
	volatile long t=0;

#if !(USE_WDOG)
    SetWord16(SET_FREEZE_REG, FRZ_WDOG); //Prepare WDOG, i.e. stop
#endif
    
//    // Gives 1dB higher sensitivity - UNTESTED
//    if (GetBits16(ANA_STATUS_REG, BOOST_SELECTED) == 0x1) 
//    { 
//        // Boost-mode
//        SetBits16(DCDC_CTRL2_REG, DCDC_CUR_LIM, 0x8); // 80mA
//    }
//    else 
//    { 
//        // Buck-mode
//        SetBits16(DCDC_CTRL2_REG, DCDC_CUR_LIM, 0x4); // 40mA
//    }
    
	/*
	 * Wait and Switch to XTAL 16MHz
	 * (default after each wake-up is RC 16MHz, but XTAL initialization sequence has been already initiated by PMU)
	 * NOTE 1: 
	 *       1. If app does not need XTAL16MHz but RC16MHz is enough then skip this section!
	 *       2. Wait-loop BEFORE activating PERIPH_PD in order to save some power...
     *
     * NOTE 2:
     *       1. It will save some power if you lower the clock while waiting for XTAL16 to settle.
     *       2. Could also switch to 32KHz, but then processing power is dramatically reduced (e.g. patching() routine may be too slow).
	 */
    SetBits16(CLK_AMBA_REG, PCLK_DIV, 3);   // lowest is 2MHz (div 8, source is RC @16MHz)
    SetBits16(CLK_AMBA_REG, HCLK_DIV, 3);

    while ( !GetBits16(SYS_STAT_REG, XTAL16_SETTLED) )  // this takes some mili seconds
        __NOP(), __NOP(), __NOP();          // reduce some APB activity

    SetBits16(CLK_CTRL_REG, SYS_CLK_SEL, 0);// select XTAL 16MHz
    SetBits16(CLK_16M_REG, RC16M_ENABLE, 0);// save power from RC 16MHz
    
    // and restore clock rates (refer to a couple of lines above)
    SetBits16(CLK_AMBA_REG, PCLK_DIV, 0);
    SetBits16(CLK_AMBA_REG, HCLK_DIV, 0);
    
    /*
     * Init System Power Domain blocks: GPIO, WD Timer, Sys Timer, etc.
	 * Power up and init Peripheral Power Domain blocks, and finally release the pad latches.
	 */
    if(GetBits16(SYS_STAT_REG, PER_IS_DOWN))
        periph_init();

    
	/*
	 * Since XTAL 16MHz is activated, power-up the Radio Subsystem (including BLE)
	 *
	 * Note that BLE core clock is masked in order to handle the case where RADIO_PD does not get into power down state.
	 * The BLE clock should be active only as long as system is running at XTAL 16MHz (not at RC16 or 32KHz).
	 * Also BLE clock should be enabled before powering up the RADIO Power Domain !
	 */
	SetBits16(CLK_RADIO_REG, BLE_ENABLE, 1); // BLE clock enable
	SetBits16(PMU_CTRL_REG, RADIO_SLEEP, 0); // Power up! Note: System must run at 16MHz when powering up RADIO_PD.
	while (!(GetWord16(SYS_STAT_REG) & RAD_IS_UP)) {}; // this may take up to 1/2 of the 32KHz clock period


    /* 
     * Wait for at least one Low Power clock edge after the power up of the Radio Power Domain *e.g. with ble_wait_lp_clk_posedge() )
     * or even better check the BLE_CNTL2_REG[WAKEUPLPSTAT] !
     * Thus you assure that BLE_WAKEUP_LP_IRQ is deasserted and BLE_SLP_IRQ is asserted.
     * After this check exit this ISE in order to proceed with BLE_SLP_Handler().
     */
	while ( GetBits32(BLE_CNTL2_REG, WAKEUPLPSTAT) || !GetBits32(BLE_INTSTAT_REG, SLPINTSTAT))
		if (t) break;

	// Now BLE_WAKEUP_LP_IRQ is deasserted and BLE_SLP_IRQ is asserted, so exit in order to proceed with BLE_SLP_Handler().
	// NOTE: If returning from BLE_WAKEUP_LP_Handler() will not cause BLE_SLP_Handler() to start, 
	//       but the code after __WFI() is executed, then THERE WAS A SW SETUP PROBLEM !!! 
	//			 so it is recommended to place a check after __WFI().
}



/*********************************************************************************
 *** CSCNT_INT ISR
 ***/
#ifdef __GNUC__
void __real_BLE_CSCNT_Handler(void);
#else
void $Super$$BLE_CSCNT_Handler(void);
#endif

#ifdef __GNUC__
void __wrap_BLE_CSCNT_Handler(void)
#else
void $Sub$$BLE_CSCNT_Handler(void)
#endif
{
    // Call the ROM handler
#ifdef __GNUC__
    __real_BLE_CSCNT_Handler();
#else
    $Super$$BLE_CSCNT_Handler();
#endif
    
    // Re-schedule any past events here
    lld_evt_start_isr();
}




/*********************************************************************************
 *** SLP_INT ISR
 ***/
void BLE_SLP_Handler(void)
{
	ble_regs_pop();
//	smpc_regs_pop();
    

	SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, 0);   

	SetBits16(GP_CONTROL_REG, BLE_WAKEUP_REQ, 0);   //just to be sure    

    if((jump_table_struct[0] == TASK_GTL) || (BLE_INTEGRATED_HOST_GTL == 1 ))
	{
		// UART and pads have already been activated by periph_init() which is called
		// at initialization by main_func() and during wakeup by BLE_WAKEUP_LP_Handler().
		
#if BLE_HOST_PRESENT		
		gtl_eif_init();
#else
		hci_eif_init();
#endif		
	}

	SetBits32(BLE_INTACK_REG, SLPINTACK, 1);

#if DEEP_SLEEP //Needed only for compilation. Remove when ROM code is ready.
#if RW_BLE_SUPPORT
	rwip_wakeup();
#endif //RW_BLE_SUPPORT
#endif //DEEP_SLEEP
    
	/*
	* Radio Subsystem initialization. Execute here after making sure that BLE core is awake.
	*/
	rf_workaround_init();
	rf_reinit();	

    if (lp_clk_sel == LP_CLK_RCX20)
        calibrate_rcx20(20);
    
    rwble_last_event = BLE_EVT_SLP;
}


#if 0
/*********************************************************************************
 *** ERROR_INT ISR
 ***/
void BLE_ERROR_Handler(void)
{
#if DEVELOPMENT_DEBUG
	// Break into the debugger if one is attached otherwise a hardfault int will be triggerd
	__asm("BKPT #0\n"); 
#endif	
	
	// Handle exception accordingly
	
	// Acknowledge interrupt
    SetBits32(BLE_INTACK_REG, ERRORINTACK, 1);
	SetBits32(BLE_CNTL2_REG, EMACCERRACK, 1);
}
#endif //if 0

/*********************************************************************************
 *** RX_INT ISR
 ***/
#ifdef __GNUC__
void __wrap_BLE_RX_Handler(void)
#else
void $Sub$$BLE_RX_Handler(void)
#endif
{
    
#ifdef METRICS    
    measure_errors_received(LLD_RX_IRQ_THRES);
#endif    
    lld_evt_rx_isr();
    //SetBits32(BLE_INTACK_REG, RXINTACK, 1);
}

/*********************************************************************************
 *** EVENT_INT ISR
 ***/
#ifdef __GNUC__
void __wrap_BLE_EVENT_Handler(void)
#else
void $Sub$$BLE_EVENT_Handler(void)
#endif
{
    // Check BLE interrupt status and call the appropriate handlers
    //uint32_t irq_stat = ble_intstat_get();
    uint32_t irq_stat = GetWord16(BLE_INTSTAT_REG);

    // End of event interrupt
    if (irq_stat & BLE_EVENTINTSTAT_BIT)
    {
#if (STREAMDATA_QUEUE)             
     cpt_event = 1;        
#ifdef METRICS
        {
            struct lld_evt_tag *evt;
             evt = (struct lld_evt_tag *)co_list_pick(&lld_evt_env.evt_prog);
             uint8_t rx_cnt = ble_rxdesccnt_getf(evt->conhdl);
            // Event has been processed, handle the transmitted and received data
             measure_errors_received( rx_cnt - evt->rx_cnt);
        }            
#endif
#endif
        lld_evt_end_isr();
        
        rwble_last_event = BLE_EVT_END;
    }
}


void $Sub$$BLE_RF_DIAG_Handler(void)
{
    uint32 irq_scr;
    uint16_t cn;    
    cn = GetWord16(RF_BMCW_REG) & 0x003F;    

    irq_scr = GetWord32(BLE_RF_DIAGIRQ_REG);  // read BLE_RF_DIAGIRQ_REG so that you clear DIAGIRQ_STAT_0 (otherwise interrupt is activated again and again!)

#if LUT_PATCH_ENABLED 
    const volatile struct LUT_CFG_struct *pLUT_CFG;	// = (const volatile struct LUT_CFG_struct *)(jump_table_struct[lut_cfg_pos]);
    pLUT_CFG = (const volatile struct LUT_CFG_struct *)(jump_table_struct[lut_cfg_pos]);
    if(!pLUT_CFG->HW_LUT_MODE)
    { 
        set_rf_cal_cap(cn); 
    }
#endif
  
#if MGCKMODA_PATCH_ENABLED 
    if(GetBits16(RF_MGAIN_CTRL_REG, GAUSS_GAIN_SEL) && (irq_scr & DIAGIRQ_STAT_0) && (irq_scr & DIAGIRQ_MASK_0))  // If GAUSS_GAIN_SEL==0x1 AND it is an TX_EN interrupt (for RX_EN int it is not necessary to run)
    { 
        set_gauss_modgain(cn); 
    }
#endif
  
	if(( irq_scr & DIAGIRQ_STAT_0) && ( irq_scr & DIAGIRQ_MASK_0))//check TXEN posedge 
	{	
#if (STREAMDATA_QUEUE)   
        transmitting_data=1;
#endif  
        rwble_last_event = BLE_EVT_TX;
        
#ifdef PRODUCTION_TEST  
		test_tx_packet_nr++;
#endif
	}
    
	if(( irq_scr & DIAGIRQ_STAT_1) && ( irq_scr & DIAGIRQ_MASK_1))//check RXEN posedge 
	{	
        rwble_last_event = BLE_EVT_RX;
        
#ifdef PRODUCTION_TEST          
		test_rx_irq_cnt++;
#endif
	}  
}


/// @} RWBLE
