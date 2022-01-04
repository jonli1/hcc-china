/**
 ****************************************************************************************
 *
 * @file user_proxr.c
 *
 * @brief Proximity reporter project source code.
 *
 * Copyright (C) 2015-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
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
#include "string.h"
#include "rwip_config.h"
#include "gapc_task.h"
#include "user_periph_setup.h"
#include "wkupct_quadec.h"
#include "app_easy_msg_utils.h"
#include "gpio.h"
#include "app_security.h"
#include "user_proxr.h"
#include "arch.h"
#include "arch_api.h"
#if defined (__DA14531__) && (defined (CFG_APP_GOTO_HIBERNATION) || defined (CFG_APP_GOTO_STATEFUL_HIBERNATION))
#include "arch_hibernation.h"
#endif
#include "app_task.h"
#include "app_proxr.h"

#if defined (__DA14531__)
#include "rtc.h"
#include "timer1.h"
#endif

#if (BLE_SUOTA_RECEIVER)
#include "app_suotar.h"
#endif

#if defined (CFG_SPI_FLASH_ENABLE)
#include "spi_flash.h"
#endif

#if (BLE_APP_SEC)
#include "app_easy_security.h"
#include "app_security.h"
#endif // (BLE_APP_SEC)

#include "app_callback.h"
#include "user_callback_config.h"

#include "user_led.h"
#include "app_console.h"
#include "app_pwm.h"
#include "app_easy_timer.h"
#include "custs1_task.h"
#include "user_custs1_def.h"
#include "ke_msg.h"
#include "arch_console.h"

#include "app_console.h"

#include "SEGGER_SWD_printf.h"


//#define APP_CONFIG_LOCK

/******************************************
 * Default sleep mode. Possible values are:
 *
 * - ARCH_SLEEP_OFF
 * - ARCH_EXT_SLEEP_ON
 * - ARCH_EXT_SLEEP_OTP_COPY_ON
 *
 ******************************************
 */
sleep_state_t app_sleep_mode = ARCH_EXT_SLEEP_ON;
timer_hnd loop_timer;
uint8_t devices_onoff = APP_DEVICES_OFF;
uint8_t ble_val[9] = {"0"};
static uint8_t time_on;
//static uint8_t app_key_en;

uint8_t usb_in_flag;

uint32_t onoff_time_flag;
uint32_t onoff_time = APP_ONOFF_TIMER_30MIN;
timer_hnd app_led_pwm_timer __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
//static void put_system_into_deep_sleep(void);
static void app_led_pwm_timer_callback(void);
void app_loop_timer_callback(void);
void app_usb_input_cb(void);
/**
 ****************************************************************************************
 * @brief Handles APP_WAKEUP_MSG sent when device exits deep sleep. Triggered by button press.
 ****************************************************************************************
*/
static void app_wakeup_cb(void)
{
    // If state is not idle, ignore the message
    if (ke_state_get(TASK_APP) == APP_CONNECTABLE)
    {
    //    default_advertise_operation();
    }
}

/**
 ****************************************************************************************
 * @brief Routine to resume system from sleep state.
 ****************************************************************************************
 */
static void app_resume_system_from_sleep(void)
{
#if !defined (__DA14531__)
    if (GetBits16(SYS_STAT_REG, PER_IS_DOWN))
	   // {
       // periph_init();
   // }
#endif

    if (arch_ble_ext_wakeup_get())
    {
				app_sleep_mode = ARCH_SLEEP_OFF;
        arch_set_sleep_mode(app_sleep_mode);
        arch_ble_force_wakeup();
        arch_ble_ext_wakeup_off();
        app_easy_wakeup();
    }
}

void app_goto_sleep_t(void){
	arch_ble_ext_wakeup_on();
	app_open_butt_enable();
	
	app_sleep_mode = ARCH_EXT_SLEEP_ON;
	arch_set_sleep_mode(app_sleep_mode);
}

void devices_on(void)
{
	onoff_time_flag = 0;
#if APP_CONFIG_LED
	app_led_start();
#endif
#if APP_CONFIG_PWM
	app_led_timer2_pwm_start();
#endif

	app_led_pwm_timer = app_easy_timer(50,app_led_pwm_timer_callback);	
	default_advertise_operation();
}
//***************** devices off *********************//
void devices_off(void)
{
	devices_onoff = APP_DEVICES_OFF;
	
//	app_key_en = 1;

#if APP_CONFIG_LED
	app_led_stop();
#endif
	
#if APP_CONFIG_PWM	
	app_led_timer2_pwm_stop();
#endif
	if (ke_state_get(TASK_APP) == APP_CONNECTED)
	{
		app_easy_gap_disconnect(app_env[0].conidx);
	}else{
		app_easy_gap_advertise_stop();
		if(!usb_in_flag){
			arch_ble_ext_wakeup_on();
		
			app_open_butt_enable();
	
			app_sleep_mode = ARCH_EXT_SLEEP_ON;
			arch_set_sleep_mode(app_sleep_mode);
		}
	}
}

/***
 * *loop timer callback 
 ***************************/
void app_loop_timer_callback(void)
{
	if((GPIO_GetPinStatus(GPIO_BUTTON_PORT,GPIO_BUTTON_PIN))&&(devices_onoff == APP_DEVICES_PRE)){
		devices_onoff = APP_DEVICES_ON;
		time_on = 0;
		//app_key_en = 1;
	}
	else if((GPIO_GetPinStatus(GPIO_BUTTON_PORT,GPIO_BUTTON_PIN))&&(devices_onoff == APP_DEVICES_ON)){
		if(time_on >= 30){
			devices_off();			//goto deep sleep
		}
		else if((time_on >= 2)&&(time_on < 30)){
			// short press key
#if APP_CONFIG_PWM
			if(!lr_pwm.lr_mode_chg)
				app_led_pwm_mode_change();
#endif
			onoff_time_flag = 0;
		}
		time_on = 0;
	}
#if APP_CONFIG_PWM
	if((devices_onoff != APP_DEVICES_OFF)&&(timer_s_flag)){
	switch(lr_pwm.lr_mode){
		case APP_LR_PWM_IDLE:
			break;
		case APP_LR_PWM_RED:			
			if(lr_pwm.led_r_duty > lr_pwm.led_r_duty_chg){
				lr_pwm.led_r_duty_chg++;
			}else if(lr_pwm.led_r_duty < lr_pwm.led_r_duty_chg){
				lr_pwm.led_r_duty_chg--;
			}
			if(lr_pwm.led_r_duty != lr_pwm.led_r_duty_chg){
				app_led_timer2_pwm_set(APP_LR_PWM_RED,lr_pwm.led_r_duty_chg);
			}
			break;
		case APP_LR_PWM_GREEN:		
			if(lr_pwm.led_g_duty > lr_pwm.led_g_duty_chg){
				lr_pwm.led_g_duty_chg++;
			}else if(lr_pwm.led_g_duty < lr_pwm.led_g_duty_chg){
				lr_pwm.led_g_duty_chg--;
			}
			if(lr_pwm.led_g_duty != lr_pwm.led_g_duty_chg){
				app_led_timer2_pwm_set(APP_LR_PWM_GREEN,lr_pwm.led_g_duty_chg);
			}
			break;
		case APP_LR_PWM_CHANGE:
			break;
	}
}
#endif
	loop_timer = app_easy_timer(5,app_loop_timer_callback);
}
/**
 ****************************************************************************************
 * @brief Button press callback function. Registered in WKUPCT driver.
 ****************************************************************************************
 */
static void app_button_press_cb(void)
{
	
#if (BLE_PROX_REPORTER)
    if (alert_state.lvl != PROXR_ALERT_NONE)
    {
        app_proxr_alert_stop();
    }
#endif
    app_resume_system_from_sleep();
	
	if(!usb_in_flag){
	if(devices_onoff == APP_DEVICES_OFF){
		
		if(!(GPIO_GetPinStatus(GPIO_BUTTON_PORT,GPIO_BUTTON_PIN))&&(time_on <= 30)&&(devices_onoff != APP_DEVICES_PRE)){
			time_on++;
			}else{
			time_on = 0;
			}
		if(time_on >= 30){
			devices_on();
			devices_onoff = APP_DEVICES_PRE;
			}
		}
		else if(devices_onoff == APP_DEVICES_ON){
		if(!(GPIO_GetPinStatus(GPIO_BUTTON_PORT,GPIO_BUTTON_PIN)))
			time_on++;
			}
		}

	app_button_enable();
}
void app_usb_input_cb_t(void){
	app_resume_system_from_sleep();
}
void app_open_butt_enable(void)
{
	app_easy_wakeup_set(app_wakeup_cb);
  wkupct2_register_callback(app_button_press_cb);
	wkupct_register_callback(app_usb_input_cb_t);
	
  wkupct2_enable_irq(WKUPCT_PIN_SELECT(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN), // select pin (GPIO_BUTTON_PORT, GPIO_BUTTON_PIN)
                          WKUPCT_PIN_POLARITY(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, WKUPCT_PIN_POLARITY_LOW), // polarity low
                          1, // 1 event
                          100); // debouncing time = 0
	wkupct_enable_irq(WKUPCT_PIN_SELECT(GPIO_USB_IN_PORT, GPIO_USB_IN_PIN), // select pin (GPIO_BUTTON_PORT, GPIO_BUTTON_PIN)
                          WKUPCT_PIN_POLARITY(GPIO_USB_IN_PORT, GPIO_USB_IN_PIN, WKUPCT_PIN_POLARITY_HIGH), // polarity low
                          1, // 1 event
                          100); // debouncing time = 0
	
}

void app_button_enable(void)
{
		GPIO_EnableIRQ( GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, GPIO2_IRQn, true, false, 100);
    GPIO_RegisterCallback(GPIO2_IRQn, app_button_press_cb);
}
#if APP_CONFIG_USBIN
void app_usb_input_cb(void){
#if 1
	if(!usb_in_flag){
		usb_in_flag = 1;
		app_resume_system_from_sleep();
		if(devices_onoff == 1/*APP_DEVICES_ON*/){
			devices_off();
		}
		app_led_mode_flag = 1;//APP_LED_MODE_CHANGE;
	}else{
		usb_in_flag = 0;
		app_led_mode_flag = 2;//APP_LED_MODE_PON;
#if APP_CONFIG_LED
	app_led_stop();
#endif
		app_goto_sleep_t();
	}
#endif
	app_usbin_enable();
}

void app_usbin_enable(void)
{
	if(!usb_in_flag)
		GPIO_EnableIRQ( GPIO_USB_IN_PORT, GPIO_USB_IN_PIN, GPIO3_IRQn, false, true, 100);
	else
		GPIO_EnableIRQ( GPIO_USB_IN_PORT, GPIO_USB_IN_PIN, GPIO3_IRQn, true, true, 100);
  GPIO_RegisterCallback(GPIO3_IRQn, app_usb_input_cb);

}
#endif
#if (BLE_SUOTA_RECEIVER)
void on_suotar_status_change(const uint8_t suotar_event)
{
#if (!SUOTAR_SPI_DISABLE)
    uint8_t dev_id;

    // Release the SPI flash memory from power down
    spi_flash_release_from_power_down();

    // Disable the SPI flash memory protection (unprotect all sectors)
    spi_flash_configure_memory_protection(SPI_FLASH_MEM_PROT_NONE);

    // Try to auto-detect the SPI flash memory
    spi_flash_auto_detect(&dev_id);

    if (suotar_event == SUOTAR_END)
    {
        // Power down the SPI flash memory
        spi_flash_power_down();
    }
#endif
}
#endif

static void app_led_pwm_timer_callback(void){
	

	if(!memcmp(&ble_val,"AT",2)){
		if(!memcmp((&(ble_val)),"AT+LRP",6)){
#if APP_CONFIG_PWM
			if(lr_pwm.lr_mode == APP_LR_PWM_RED){
				//lr_pwm.led_r_duty = ble_val[7];
				lr_pwm.led_r_duty = (uint8_t)(ble_val[7] - 0x30)*10 + (uint8_t)(ble_val[8] - 0x30);
			}
		else if(lr_pwm.lr_mode == APP_LR_PWM_GREEN){
				//lr_pwm.led_g_duty = ble_val[7];
				lr_pwm.led_g_duty = (uint8_t)(ble_val[7] - 0x30)*10 + (uint8_t)(ble_val[8] - 0x30);
			}
#endif
			onoff_time_flag = 0;
		}else if(!memcmp((&(ble_val)),"AT+LRM",6)){
#if APP_CONFIG_PWM
			#if 0
			if(ble_val[7] == 0x31)
				app_led_pwm_mode_change();
			#else
			if(ble_val[7] == 0x31){
				if(lr_pwm.lr_mode != APP_LR_PWM_GREEN){
					app_led_pwm_mode_change();
				}
			}else if(ble_val[7] == 0x32){
				if(lr_pwm.lr_mode != APP_LR_PWM_RED){
					app_led_pwm_mode_change();
				}
			}
			#endif
#endif
			onoff_time_flag = 0;
		}else if(!memcmp((&(ble_val)),"AT+TRL",6)){
			switch(ble_val[7]){
				case APP_ONOFF_TIME_MODE_20MIN:
					onoff_time = APP_ONOFF_TIMER_20MIN;
					break;
				case APP_ONOFF_TIME_MODE_30MIN:
					onoff_time = APP_ONOFF_TIMER_30MIN;
					break;
				case APP_ONOFF_TIME_MODE_40MIN:
					onoff_time = APP_ONOFF_TIMER_40MIN;
					break;
			}
			onoff_time_flag = 0;
		}
		memset(ble_val, 0, sizeof(ble_val));
	}
	onoff_time_flag++;
	if(onoff_time_flag >= onoff_time){
		//shut down
		if(!usb_in_flag){
			devices_off();
		}
	}
	
	if(devices_onoff != APP_DEVICES_OFF){
	app_led_pwm_timer = app_easy_timer(50,app_led_pwm_timer_callback);
	}
}

void app_app_on_connection(uint8_t conidx, struct gapc_connection_req_ind const *param)
{
    if (app_env[conidx].conidx != GAP_INVALID_CONIDX)
    {
        if (user_default_hnd_conf.adv_scenario == DEF_ADV_WITH_TIMEOUT)
        {
            app_easy_gap_advertise_with_timeout_stop();
        }

        // Enable the created profiles/services
        app_prf_enable(conidx);

        #if (BLE_APP_SEC)
        if (user_default_hnd_conf.security_request_scenario == DEF_SEC_REQ_ON_CONNECT)
        {
            app_easy_security_request(conidx);
        }
				
				//ke_state_set(TASK_APP, APP_CONNECTED);
				
        #endif // (BLE_APP_SEC)
#if APP_CONFIG_LED
				app_bl_led_mode(APP_BL_LED_CON);
#endif
    }
    else
    {
       // No connection has been established, restart advertising
      // CALLBACK_ARGS_0(user_default_app_operations.default_operation_adv)
			default_advertise_operation();
    }
}

void user_app_on_disconnect(struct gapc_disconnect_ind const *param)
{
	if(!usb_in_flag){
    //default_app_on_disconnect(NULL);
	if(devices_onoff != APP_DEVICES_OFF){
		default_advertise_operation();
	}else{

	}
#if APP_CONFIG_LED	
	app_bl_led_mode(APP_BL_LED_NCON);
#endif
	}
	
#if (BLE_BATT_SERVER)
    app_batt_poll_stop();
#endif

#if (BLE_SUOTA_RECEIVER)
    // Issue a platform reset when it is requested by the suotar procedure
    if (suota_state.reboot_requested)
    {
        // Reboot request will be served
        suota_state.reboot_requested = 0;

        // Platform reset
        platform_reset(RESET_AFTER_SUOTA_UPDATE);
    }
#endif
	//app_button_enable();

#if BLE_PROX_REPORTER
    app_proxr_alert_stop();
#endif
}

#if defined (__DA14531__)

#if defined (CFG_EXT_SLEEP_WAKEUP_RTC) || defined (CFG_DEEP_SLEEP_WAKEUP_RTC)
/**
 ****************************************************************************************
 * @brief RTC interrupt handler routine for wakeup.
 ****************************************************************************************
*/

static void rtc_interrupt_hdlr(uint8_t event)
{
#if defined (CFG_EXT_SLEEP_WAKEUP_RTC)
    app_resume_system_from_sleep();
#endif
}

/**
 ****************************************************************************************
 * @brief Configure RTC to generate an interrupt after 10 seconds.
 ****************************************************************************************
*/
static void configure_rtc_wakeup(void)
{
    rtc_time_t alarm_time;

    // Init RTC
    rtc_reset();

    // Configure the RTC clock; RCX is the RTC clock source (14420 Hz)
    rtc_clk_config(RTC_DIV_DENOM_1000, 14420);
    rtc_clock_enable();

    rtc_config_t cfg = {.hour_clk_mode = RTC_HOUR_MODE_24H, .keep_rtc = 0};

    rtc_time_t time = {.hour_mode = RTC_HOUR_MODE_24H, .pm_flag = 0, .hour = 11,
                       .minute = 55, .sec = 30, .hsec = 00};

    // Alarm interrupt in ten seconds
    alarm_time = time;
    alarm_time.sec += 10;

    // Initialize RTC, set time and data, register interrupt handler callback function and enable seconds interrupt
    rtc_init(&cfg);

    // Start RTC
    rtc_set_time_clndr(&time, NULL);
    rtc_set_alarm(&alarm_time, NULL, RTC_ALARM_EN_SEC);

    // Clear pending interrupts
    rtc_get_event_flags();
    rtc_register_intr(rtc_interrupt_hdlr, RTC_INTR_ALRM);
#if defined (CFG_EXT_SLEEP_WAKEUP_RTC)
    app_easy_wakeup_set(app_wakeup_cb);
#endif
}
#endif

#if defined (CFG_EXT_SLEEP_WAKEUP_TIMER1) || defined (CFG_DEEP_SLEEP_WAKEUP_TIMER1)
/**
 ****************************************************************************************
 * @brief Timer1 interrupt handler routine for wakeup.
 ****************************************************************************************
*/

static void timer1_interrupt_hdlr(void)
{
#if defined (CFG_EXT_SLEEP_WAKEUP_TIMER1)
    app_resume_system_from_sleep();
#endif
}

/**
 ****************************************************************************************
 * @brief Configure Timer1 to generate an interrupt when it reaches its max value.
 ****************************************************************************************
*/
static void configure_timer1_wakeup(void)
{
    timer1_count_options_t count_options = {.input_clk = TIM1_CLK_SRC_LP,
                                            .free_run = TIM1_FREE_RUN_ON,
                                            .irq_mask = TIM1_IRQ_MASK_OFF,
                                            .count_dir = TIM1_CNT_DIR_UP,
                                            .reload_val = TIM1_RELOAD_MAX,
    };
    // Enable Timer1 interrupt
    timer1_enable_irq();
#if defined (CFG_EXT_SLEEP_WAKEUP_TIMER1)
    app_easy_wakeup_set(app_wakeup_cb);
#endif
    timer1_count_config(&count_options, timer1_interrupt_hdlr);

    // Start the Timer
    timer1_start();
}
#endif

#if defined (CFG_APP_GOTO_DEEP_SLEEP)
/**
 ****************************************************************************************
 * @brief Put the system into deep sleep mode. It demonstrates the deep sleep mode usage
 *        and how the system can wake up from it. The exit from the deep sleep state causes 
 *        a system reboot.
 * @note  The system can wake up from deep sleep by:
 *          - external wake up interrupt, caused e.g. by button press (properly configured GPIO pin)
 *          - power on reset, caused e.g. by button press (properly configured GPIO pin)
 *          - interrupt generated from RTC
 *          - interrupt generated from Timer1
 *        When the system exits deep sleep state, the boot process is triggered.
 *        The application code has to be programmed in an external memory resource or
 *        in the OTP memory, in order for the system to reboot properly.
 ****************************************************************************************
*/
static void put_system_into_deep_sleep(void)
{
#if defined (CFG_DEEP_SLEEP_WAKEUP_POR)
    // Configure button for POR
    GPIO_EnablePorPin(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, GPIO_POR_PIN_POLARITY_LOW, GPIO_GetPorTime());
#endif

#if defined (CFG_DEEP_SLEEP_WAKEUP_GPIO)
    wkupct_enable_irq(WKUPCT_PIN_SELECT(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN), // Select pin
                      WKUPCT_PIN_POLARITY(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, WKUPCT_PIN_POLARITY_LOW), // Polarity low
                      1, // 1 event
                      30); // debouncing time = 0
#endif

#if defined (CFG_DEEP_SLEEP_WAKEUP_RTC)
    configure_rtc_wakeup();
#endif

#if defined (CFG_DEEP_SLEEP_WAKEUP_TIMER1)
    configure_timer1_wakeup();
#endif

    // Go to deep sleep
    arch_set_deep_sleep(CFG_DEEP_SLEEP_RAM1,
                        CFG_DEEP_SLEEP_RAM2,
                        CFG_DEEP_SLEEP_RAM3,
                        CFG_DEEP_SLEEP_PAD_LATCH_EN);
}
#endif // (CFG_APP_GOTO_DEEP_SLEEP)

#else //__DA14531__

#if defined (CFG_APP_GOTO_DEEP_SLEEP)

/**
 ****************************************************************************************
 * @brief Put the system into deep sleep mode. It demonstrates the deep sleep mode usage
 *        and how the system can wake up from it. Once the system enters deep sleep state
 *        it retains NO RAM blocks. The exit from the deep sleep state causes a system
 *        reboot.
 * @note  The system can wake up from deep sleep by:
 *          - external wake up interrupt, caused e.g. by button press (properly configured GPIO pin)
 *          - power on reset, caused e.g. by button press (properly configured GPIO pin)
 *          - H/W reset button press or power cycle (at any time)
 *        When the system exits deep sleep state, the boot process is triggered.
 *        The application code has to be programmed in an external memory resource or
 *        in the OTP memory, in order for the system to reboot properly.
 ****************************************************************************************
*/
static void put_system_into_deep_sleep(void)
{
#if defined (CFG_DEEP_SLEEP_WAKEUP_GPIO)
    // Configure button for wake-up interrupt
    app_button_enable();

    // Set deep sleep - external interrupt wake up
    arch_set_deep_sleep(true);

#elif defined (CFG_DEEP_SLEEP_WAKEUP_POR)
    // Configure button for POR
    GPIO_EnablePorPin(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, GPIO_POR_PIN_POLARITY_LOW, GPIO_GetPorTime());

    // Set deep sleep - POR wake up
    arch_set_deep_sleep(false);

#else
    // Do nothing.
    // The system will eventually enter the selected Extended sleep state.
    // A button press will wake up the system if the respective GPIO is configured as a wake up interrupt.
#endif
}

#endif //(CFG_APP_GOTO_DEEP_SLEEP)

#endif

void app_advertise_operation(void)
{
#if APP_CONFIG_LED
	app_led_init();
#endif	
#if APP_CONFIG_PWM		
	app_led_timer2_pwm_init();
#endif
	loop_timer = app_easy_timer(20,app_loop_timer_callback);
	
	//default_advertise_operation();
	
	arch_ble_ext_wakeup_on();
}

void app_advertise_complete(const uint8_t status)
{
	
    if ((status == GAP_ERR_NO_ERROR) || (status == GAP_ERR_CANCELED))
    {

#if (BLE_PROX_REPORTER)
        app_proxr_alert_stop();
#endif
    }

    if (status == GAP_ERR_CANCELED)
    {
        arch_ble_ext_wakeup_on();

#if defined (__DA14531__)
        // Close PD_TIM
        SetBits16(PMU_CTRL_REG, TIM_SLEEP, 1);
        // Wait until PD_TIM is closed
        while ((GetWord16(SYS_STAT_REG) & TIM_IS_DOWN) != TIM_IS_DOWN);
#endif

#if defined (CFG_APP_GOTO_DEEP_SLEEP)
        // Put system into deep sleep
//        put_system_into_deep_sleep();
#else
        // Configure button to trigger wake-up interrupt from extended sleep
        app_button_enable();
#endif
    }
}
/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void user_svc1_ctrl_wr_ind_handler(ke_msg_id_t const msgid,
                                      struct custs1_val_write_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t val = 0;
    memcpy(&val, &param->value[0], param->length);

	#if 0
    if (val != CUSTS1_CP_ADC_VAL1_DISABLE)
    {
        timer_used = app_easy_timer(APP_PERIPHERAL_CTRL_TIMER_DELAY, app_adcval1_timer_cb_handler);
    }
    else
    {
        if (timer_used != EASY_TIMER_INVALID_TIMER)
        {
            app_easy_timer_cancel(timer_used);
            timer_used = EASY_TIMER_INVALID_TIMER;
        }
    }
		#endif
}

void user_svc1_F002_wr_ind_handler(ke_msg_id_t const msgid,
                                     struct custs1_val_write_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    
    memcpy(&ble_val, &param->value, param->length);
	
	    struct custs1_val_ntf_ind_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_NTF_REQ,
                                                          prf_get_task_from_id(TASK_ID_CUSTS1),
                                                          TASK_APP,
                                                          custs1_val_ntf_ind_req,
                                                          DEF_SVC1_CTRL_POINT_CHAR_LEN);
    req->handle = SVC1_IDX_CONTROL_POINT_VAL;
    req->length = 9;
    req->notification = true;
		memcpy(req->value, &ble_val, 9);
		
    // Send the message
    ke_msg_send(req);
}
void user_svc1_f003_val_1_cfg_ind_handler(ke_msg_id_t const msgid,
                                            struct custs1_val_write_ind const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
}

void user_catch_rest_hndl(ke_msg_id_t const msgid,
                          void const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case CUSTS1_VAL_WRITE_IND:
        {
            struct custs1_val_write_ind const *msg_param = (struct custs1_val_write_ind const *)(param);

            switch (msg_param->handle)
            {
                case SVC1_IDX_CONTROL_POINT_VAL:
                    user_svc1_ctrl_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case SVC1_IDX_F002_VAL:
                    user_svc1_F002_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case SVC1_IDX_F003_VAL:
                    user_svc1_f003_val_1_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                default:
                    break;
            }
        } break;

        case GAPC_PARAM_UPDATED_IND:
        {
            // Cast the "param" pointer to the appropriate message structure
            struct gapc_param_updated_ind const *msg_param = (struct gapc_param_updated_ind const *)(param);

            // Check if updated Conn Params filled to preferred ones
            if ((msg_param->con_interval >= user_connection_param_conf.intv_min) &&
                (msg_param->con_interval <= user_connection_param_conf.intv_max) &&
                (msg_param->con_latency == user_connection_param_conf.latency) &&
                (msg_param->sup_to == user_connection_param_conf.time_out))
            {
            }
        } break;

        default:
            break;
    }
}


/// @} APP
