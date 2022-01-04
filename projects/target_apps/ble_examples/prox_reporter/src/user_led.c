

#include "user_periph_setup.h"
#include "gpio.h"
#include "app.h"
#include "arch_console.h"
#include "app_easy_timer.h"
#include "battery.h"

#include "user_led.h"
#include "app_console.h"


uint8_t app_led_mode_flag = APP_LED_MODE_IDLE;
extern uint8_t devices_onoff;
extern uint8_t usb_in_flag;

#if APP_CONFIG_LED
/*
 * variable
 */
app_led_state led1_start; 
app_led_state led2_start; 
app_led_state led3_start;
app_led_state bl_led_start;


uint8_t app_batt_lvl_flag = 0;

timer_hnd app_led_timer __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
/******
 * Function
 *
 ******/

static void app_led_timer_callback(void);
/*
	led init.
*/
void app_led_init(void)
{
		led1_start.led_pin.port = GPIO_LED1_RED_PORT;
		led1_start.led_pin.pin = GPIO_LED1_RED_PIN;
		led2_start.led_pin.port = GPIO_LED2_RED_PORT;
		led3_start.led_pin.pin = GPIO_LED2_RED_PIN;
		led3_start.led_pin.port = GPIO_LED3_RED_PORT;
		led3_start.led_pin.pin = GPIO_LED3_RED_PIN;
	
	app_led_timer = app_easy_timer(50,app_led_timer_callback);
}
void app_led_start(void)
{	
	app_bl_led_mode(APP_BL_LED_NCON);
		///GPIO_ConfigurePin(GPIO_LED1_RED_PORT, GPIO_LED1_RED_PIN, OUTPUT, PID_GPIO, false);
		//GPIO_ConfigurePin(GPIO_LED2_RED_PORT, GPIO_LED2_RED_PIN, OUTPUT, PID_GPIO, false);
		//GPIO_ConfigurePin(GPIO_LED3_RED_PORT, GPIO_LED3_RED_PIN, OUTPUT, PID_GPIO, false);
	app_led_mode_flag = APP_LED_MODE_PON; // APP_LED_MODE_CHANGE;//
	
	//app_led_timer = app_easy_timer(50,app_led_timer_callback);
}

void app_bl_led_mode(uint8_t bl_led_mode)
{
	switch(bl_led_mode){
		case APP_BL_LED_IDLE:
			GPIO_ConfigurePin(GPIO_BL_LED_PORT, GPIO_BL_LED_PIN, INPUT, PID_GPIO, false);
			break;
		case APP_BL_LED_CON:
			GPIO_SetActive(GPIO_BL_LED_PORT,GPIO_BL_LED_PIN);
			break;
		case APP_BL_LED_NCON:
			GPIO_SetInactive(GPIO_BL_LED_PORT,GPIO_BL_LED_PIN);
			break;
	}
}
#if 1

static void app_blue_on(void)
{
	GPIO_SetInactive(GPIO_LED_BLUE_PORT,GPIO_LED_BLUE_PIN);
}

void app_blue_off(void)
{
	GPIO_SetActive(GPIO_LED_BLUE_PORT,GPIO_LED_BLUE_PIN);
}

static void app_batt_led(uint8_t lvl_mode)
{
	switch(lvl_mode){
		case APP_BATT_LED_LVL1:
			GPIO_SetInactive(GPIO_LED1_RED_PORT,GPIO_LED1_RED_PIN);
			GPIO_SetActive(GPIO_LED2_RED_PORT,GPIO_LED2_RED_PIN);
			GPIO_SetActive(GPIO_LED3_RED_PORT,GPIO_LED3_RED_PIN);
			break;
		case APP_BATT_LED_LVL2:
			GPIO_SetInactive(GPIO_LED1_RED_PORT,GPIO_LED1_RED_PIN);
			GPIO_SetInactive(GPIO_LED2_RED_PORT,GPIO_LED2_RED_PIN);
			GPIO_SetActive(GPIO_LED3_RED_PORT,GPIO_LED3_RED_PIN);
			break;
		case APP_BATT_LED_LVL3:
			GPIO_SetInactive(GPIO_LED1_RED_PORT,GPIO_LED1_RED_PIN);
			GPIO_SetInactive(GPIO_LED2_RED_PORT,GPIO_LED2_RED_PIN);
			GPIO_SetInactive(GPIO_LED3_RED_PORT,GPIO_LED3_RED_PIN);
			break;
		case APP_BATT_LED_LVL0:
			GPIO_SetActive(GPIO_LED1_RED_PORT,GPIO_LED1_RED_PIN);
			GPIO_SetActive(GPIO_LED2_RED_PORT,GPIO_LED2_RED_PIN);
			GPIO_SetActive(GPIO_LED3_RED_PORT,GPIO_LED3_RED_PIN);
			break;
		case APP_BATT_LED_LVL4:
			GPIO_SetInactive(GPIO_LED1_RED_PORT,GPIO_LED1_RED_PIN);
			GPIO_SetActive(GPIO_LED2_RED_PORT,GPIO_LED2_RED_PIN);
			GPIO_SetActive(GPIO_LED3_RED_PORT,GPIO_LED3_RED_PIN);
			break;
	}

}


static void app_led_timer_callback(void){
	
	static uint8_t count_200ms = 0;
	static uint8_t lp_flag = 0;
	static uint8_t app_batt_lvl_old_val = 0;
	
	app_batt_lvl_flag = battery_get_lvl(BATT_CR2032);
	if(!app_batt_lvl_old_val){
		app_batt_lvl_old_val = app_batt_lvl_flag;
	}

	if(usb_in_flag){
		if(app_batt_lvl_old_val > app_batt_lvl_flag){
			app_batt_lvl_flag = app_batt_lvl_old_val;
		}
	}else{
		if(app_batt_lvl_old_val < app_batt_lvl_flag){
			app_batt_lvl_flag = app_batt_lvl_old_val;
		}
	}
		//arch_printf("[led] blink 1\n");
		switch(app_led_mode_flag){
			case APP_LED_MODE_IDLE:
				break;
			case APP_LED_MODE_CHANGE:
				count_200ms++;
				if(app_batt_lvl_flag < 99){
				if(count_200ms >= 2){
					count_200ms = 0;
					if(led1_start.blink_onoff){
						led1_start.blink_onoff = 0;
						led2_start.blink_onoff = 1;
						led3_start.blink_onoff = 0;
						app_batt_led(APP_BATT_LED_LVL1);
					}else if(led2_start.blink_onoff){
						led1_start.blink_onoff = 0;
						led2_start.blink_onoff = 0;
						led3_start.blink_onoff = 1;
						app_batt_led(APP_BATT_LED_LVL2);
					}else if(led3_start.blink_onoff){
						led1_start.blink_onoff = 1;
						led2_start.blink_onoff = 0;
						led3_start.blink_onoff = 0;
						app_batt_led(APP_BATT_LED_LVL3);
					}else{
						led1_start.blink_onoff = 0;
						led2_start.blink_onoff = 1;
						led3_start.blink_onoff = 0;
						app_batt_led(APP_BATT_LED_LVL1);
					}
				}
			}else{
				app_batt_led(APP_BATT_LED_LVL0);
				app_blue_on();
			}
				break;
			case APP_LED_MODE_PON:
				if(devices_onoff == APP_DEVICES_ON){
				if(app_batt_lvl_flag >= 99)
					app_batt_led(APP_BATT_LED_LVL3);
				else if((app_batt_lvl_flag >= 75)&&(app_batt_lvl_flag < 99))
					app_batt_led(APP_BATT_LED_LVL2);
				else if((app_batt_lvl_flag > 25)&&(app_batt_lvl_flag < 75))
					app_batt_led(APP_BATT_LED_LVL1);
				else if(app_batt_lvl_flag <= 25){
					app_led_mode_flag = APP_LED_MODE_LP;
				}
				}
				break;
			case APP_LED_MODE_LP:
				count_200ms++;
				if(count_200ms >= 2){
					count_200ms = 0;
					if(lp_flag){
						lp_flag = 0;
						app_batt_led(APP_BATT_LED_LVL4);
					}else{
						lp_flag = 1;
						app_batt_led(APP_BATT_LED_LVL0);
					}
				}
				if(app_batt_lvl_flag > 25){
					app_led_mode_flag = APP_LED_MODE_PON;
					count_200ms = 0;
				}
				break;
		}
		
	app_batt_lvl_old_val = app_batt_lvl_flag;
	
	app_led_timer = app_easy_timer(50,app_led_timer_callback);
	
}

void app_led_stop(void)
{
	app_blue_off();
	
	app_led_mode_flag = APP_LED_MODE_IDLE;

			GPIO_SetActive(GPIO_LED1_RED_PORT,GPIO_LED1_RED_PIN);
			GPIO_SetActive(GPIO_LED2_RED_PORT,GPIO_LED2_RED_PIN);
			GPIO_SetActive(GPIO_LED3_RED_PORT,GPIO_LED3_RED_PIN);
	
			GPIO_SetActive(GPIO_LED_BLUE_PORT,GPIO_LED_BLUE_PIN);

  //app_easy_timer_cancel(app_led_timer);
	app_bl_led_mode(APP_BL_LED_NCON);
}
#endif

#endif

