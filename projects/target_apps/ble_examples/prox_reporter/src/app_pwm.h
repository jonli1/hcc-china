

#include <stdio.h>
#include <stdint.h>


/*
 * CONSOLE DEFINITIONS
 ****************************************************************************************
 */
#define APP_LR_PWM_DUTY_NULL 111
#define APP_ONOFF_TIMER_20MIN	2400
#define APP_ONOFF_TIMER_30MIN	3600
#define APP_ONOFF_TIMER_40MIN	4800

enum app_onoff_timer_flag {
	APP_ONOFF_TIME_MODE_IDEL		=		0x30,
	APP_ONOFF_TIME_MODE_20MIN,
	APP_ONOFF_TIME_MODE_30MIN,
	APP_ONOFF_TIME_MODE_40MIN,
};
 
enum app_lr_mode{
	APP_LR_PWM_IDLE,
	APP_LR_PWM_RED,
	APP_LR_PWM_GREEN,
	APP_LR_PWM_CHANGE,
	APP_LR_PWM_NULL,
};

typedef struct app_lr_pwm{
	uint8_t lr_mode;
	uint8_t lr_mode_chg;
	uint16_t led_g_duty_chg;
	uint16_t led_g_duty;
	uint16_t led_r_duty_chg;
	uint16_t led_r_duty;
} app_lr_pwm_t;

/*
 * Variable definition
 ****************************************************************************************
 */
extern app_lr_pwm_t lr_pwm;
extern uint8_t timer_s_flag;
//extern uint8_t ble_val[8];
/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/*init timer2*/
void app_led_timer2_pwm_init(void);
/*start timer2 pwm*/
void app_led_timer2_pwm_start(void);
/*resume pwm*/
void app_led_timer2_pwm_resume(void);
/*set timer2 pwm*/
void app_led_timer2_pwm_set(uint8_t lr_mode,uint8_t led_duty);
/*change timer2 pwm*/
void app_led_pwm_mode_change(void);
/*stop timer2 pwm*/
void app_led_timer2_pwm_stop(void);
/*
 * END
 ****************************************************************************************
 */
