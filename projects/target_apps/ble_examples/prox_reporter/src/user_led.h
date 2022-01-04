/*
	led header file.
*/
#include "stdint.h"
#include "gpio.h"
#include "app_console.h"

enum app_devices_onoff{
	APP_DEVICES_PRE,
	APP_DEVICES_ON,
	APP_DEVICES_OFF,
};

enum app_batt_led_state_t{
	APP_BATT_LED_LVL0,
	APP_BATT_LED_LVL1,
	APP_BATT_LED_LVL2,
	APP_BATT_LED_LVL3,
	APP_BATT_LED_LVL4,
};

enum app_led_state_t{
	APP_LED_100MS,
	APP_LED_500MS,
	APP_LED_1S,
	APP_LED_NONE,
};

enum app_led_mode_t{
	APP_LED_MODE_IDLE	=	0,
	APP_LED_MODE_CHANGE =	1,
	APP_LED_MODE_PON	=	2,
	APP_LED_MODE_LP	=	3,
};

enum app_led_Cmode_t{
	APP_LED_CMODE_IDLE,
	APP_LED_CMODE_ING,
	APP_LED_CMODE_FINASH,
};

enum app_bl_led_mode_t{
	APP_BL_LED_IDLE,
	APP_BL_LED_CON,
	APP_BL_LED_NCON,
};

typedef struct app_led_pin{
	   /// GPIO port
    GPIO_PORT port;
    /// GPIO pin
    GPIO_PIN pin;
} app_led_pin_t;

typedef struct app_work_stat{
	   /// GPIO port
    uint8_t work_mode;
    /// GPIO pin
    uint8_t work_stat;
} app_work_stat_t;

/// APP led state 
typedef struct
{
  uint8_t blink_onoff;
	uint8_t led_color;
	app_led_pin_t led_pin;
	app_work_stat_t work_mode;
} app_led_state;


#if APP_CONFIG_USBIN

extern void devices_off(void);
extern uint8_t app_led_mode_flag;
#endif
//extern uint8_t app_led_mode_flag;


void app_led_init(void);
void app_led_start(void);
void app_bl_led_mode(uint8_t bl_led_mode);

void app_led_stop(void);


