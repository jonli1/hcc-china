/**
 ****************************************************************************************
 *
 * @file timer2.c
 *
 * @brief DA14585/DA14586/DA14531 Timer2 driver source file.
 *
 * Copyright (C) 2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <stdint.h>
#include "timer2.h"
#include "datasheet.h"
#include "arch.h"

#include "SEGGER_SWD_printf.h"
#include "app_pwm.h"

//uint32_t app_counter, app_dc;

void timer2_config(tim2_config_t *config)
{
#if defined (__DA14531__)
    SetBits16(TRIPLE_PWM_CTRL_REG, TRIPLE_PWM_CLK_SEL, config->clk_source);
#endif
    SetBits16(TRIPLE_PWM_CTRL_REG, HW_PAUSE_EN, config->hw_pause);
}

void timer2_pwm_freq_set(uint32_t pwm_freq, uint32_t input_freq)
{
    SetWord16(TRIPLE_PWM_FREQUENCY, (input_freq / pwm_freq) - 1u);
}

void timer2_pwm_signal_config(tim2_pwm_config_t *pwm_config)
{
	uint32_t counter, dc, offset;
	
    ASSERT_WARNING(pwm_config->pwm_dc <= 100u);

    counter = GetWord16(TRIPLE_PWM_FREQUENCY) + 1u;
		//SWD_printf("count = %d,pwm_config->pwm_dc = %d\r\n", counter,pwm_config->pwm_dc);
		//app_counter = 100;
#if 0
	if(timer_s_flag){
	dc	= (counter * (pwm_config->pwm_dc)) / 1u;
	
#if defined (__DA14531__)
    ASSERT_WARNING(pwm_config->pwm_offset <= 1u);
    offset  = (counter * (pwm_config->pwm_offset)) / 1u;
#endif
	}else
	#endif
	{
	dc	= (counter * (pwm_config->pwm_dc)) / 100u;
	
#if defined (__DA14531__)
    ASSERT_WARNING(pwm_config->pwm_offset <= 100u);
    offset  = (counter * (pwm_config->pwm_offset)) / 100u;
#endif
	}
	
		SWD_printf("count = %d, dc = %d,offset = %d\r\n", counter,dc,offset);
		
    switch (pwm_config->pwm_signal)
    {
        case TIM2_PWM_2:
        {
#if defined (__DA14531__)
            SetWord16(PWM2_START_CYCLE, offset);
            SetWord16(PWM2_END_CYCLE, dc + offset);
#else
            SetWord16(PWM2_DUTY_CYCLE, dc);
#endif
            break;
        }

        case TIM2_PWM_3:
        {
#if defined (__DA14531__)
            SetWord16(PWM3_START_CYCLE, offset);
            SetWord16(PWM3_END_CYCLE, dc + offset);
#else
            SetWord16(PWM3_DUTY_CYCLE, dc);
#endif
            break;
        }

        case TIM2_PWM_4:
        {
#if defined (__DA14531__)
            SetWord16(PWM4_START_CYCLE, offset);
            SetWord16(PWM4_END_CYCLE, dc + offset);
#else
            SetWord16(PWM4_DUTY_CYCLE, dc);
#endif
            break;
        }

#if defined (__DA14531__)
        case TIM2_PWM_5:
        {
            SetWord16(PWM5_START_CYCLE, offset);
            SetWord16(PWM5_END_CYCLE, dc + offset);
            break;
        }

        case TIM2_PWM_6:
        {
            SetWord16(PWM6_START_CYCLE, offset);
            SetWord16(PWM6_END_CYCLE, dc + offset);
            break;
        }

        case TIM2_PWM_7:
        {
            SetWord16(PWM7_START_CYCLE, offset);
            SetWord16(PWM7_END_CYCLE, dc + offset);
            break;
        }
#endif

        default:
        {
            /* Default case added for the sake of completeness; it will
               never be reached. */
            break;
        }
    }
}
