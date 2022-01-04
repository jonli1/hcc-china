/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief DA14585/DA14586 Timer2 example.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "arch_system.h"
#include "user_periph_setup.h"
#include "timer0_2.h"
#include "timer2.h"
#include "app_easy_timer.h"

#include "app_pwm.h"
#include "app_console.h"

#if APP_CONFIG_PWM
/**
 ****************************************************************************************
 * @brief Variable definition
 ****************************************************************************************
 */
app_lr_pwm_t lr_pwm;
uint8_t timer_s_flag;

/**
 ****************************************************************************************
 * @brief Timer2 function
 ****************************************************************************************
 */
//void timer2_test(void);


/**
 ****************************************************************************************
 * @brief Simple delay function
 ****************************************************************************************
 */
void simple_delay(void);


static tim0_2_clk_div_config_t clk_div_config =
{
    .clk_div  = TIM0_2_CLK_DIV_2
};

static tim2_config_t config =
{
		.clk_source = TIM2_CLK_LP,
    .hw_pause = TIM2_HW_PAUSE_OFF
};

static tim2_pwm_config_t pwm_config;


/**
 ****************************************************************************************
 * @brief Main routine of the timer2 example
 ****************************************************************************************
 */
void app_led_timer2_pwm_init(void)
{
		lr_pwm.lr_mode_chg = 0;
		lr_pwm.lr_mode = APP_LR_PWM_GREEN;
		lr_pwm.led_g_duty = 100;
		lr_pwm.led_r_duty = 0;
}
void app_led_timer2_pwm_start(void)
{
#if 1
	    // Enable the Timer0/Timer2 input clock
    timer0_2_clk_enable();
    // Set the Timer0/Timer2 input clock division factor
    timer0_2_clk_div_set(&clk_div_config);

    timer2_config(&config);
#endif	
	timer2_pwm_freq_set(260000, 16000000 / 2);
	
  timer2_start();
	timer_s_flag = 1;
}

void app_led_pwm_mode_change(void){
	
			if(lr_pwm.lr_mode == APP_LR_PWM_RED){
				lr_pwm.led_g_duty = lr_pwm.led_r_duty;
				lr_pwm.led_r_duty = 0;
				lr_pwm.led_g_duty_chg = lr_pwm.led_r_duty_chg;
				lr_pwm.led_r_duty_chg = 0;
				lr_pwm.lr_mode = APP_LR_PWM_GREEN;
				app_led_timer2_pwm_set(APP_LR_PWM_RED,lr_pwm.led_r_duty);
				app_led_timer2_pwm_set(lr_pwm.lr_mode,lr_pwm.led_g_duty);
			}else if(lr_pwm.lr_mode == APP_LR_PWM_GREEN){
				lr_pwm.led_r_duty = lr_pwm.led_g_duty;
				lr_pwm.led_g_duty = 0;
				lr_pwm.led_r_duty_chg = lr_pwm.led_g_duty_chg;
				lr_pwm.led_g_duty_chg = 0;
				lr_pwm.lr_mode = APP_LR_PWM_RED;
				app_led_timer2_pwm_set(APP_LR_PWM_GREEN,lr_pwm.led_g_duty);
				app_led_timer2_pwm_set(lr_pwm.lr_mode,lr_pwm.led_r_duty);
			}
}

void app_led_timer2_pwm_resume(void){
	//timer2_pause();
	//timer2_pwm_freq_set(80000, 16000000 / 8);
	//timer2_resume();
}

void app_led_timer2_pwm_set(uint8_t lr_mode,uint8_t led_duty)
{
	if(lr_mode == APP_LR_PWM_GREEN){
	      pwm_config.pwm_signal = TIM2_PWM_2;
        pwm_config.pwm_dc     = led_duty;
        timer2_pwm_signal_config(&pwm_config);
	}else if(lr_mode == APP_LR_PWM_RED){
        // Set PWM3 duty cycle
        pwm_config.pwm_signal = TIM2_PWM_3;
        pwm_config.pwm_dc     = led_duty;
        timer2_pwm_signal_config(&pwm_config);
	}
	
        timer2_resume();
}

void app_led_timer2_pwm_stop(void)
{
	
#if 1
		lr_pwm.led_r_duty_chg = 0;
		lr_pwm.led_g_duty_chg = 0;
		app_led_timer2_pwm_set(lr_pwm.lr_mode,lr_pwm.led_r_duty_chg);
#endif
	  timer2_stop();
	timer_s_flag = 0;
}



void simple_delay(void)
{
    uint32_t i;

    for (i = 0x1FFFF; i != 0; --i)
        ;
}
#endif
