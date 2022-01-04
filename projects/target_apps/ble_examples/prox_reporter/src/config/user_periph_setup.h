/**
 ****************************************************************************************
 *
 * @file user_periph_setup.h
 *
 * @brief Peripherals setup header file.
 *
 * Copyright (C) 2012-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _USER_PERIPH_SETUP_H_
#define _USER_PERIPH_SETUP_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "spi_flash.h"
#include "i2c.h"
#include "i2c_eeprom.h"



/*
 * DEFINES
 ****************************************************************************************
 */
#define APP_CONFIG_LED				1
#define APP_CONFIG_PWM				1
#define APP_CONFIG_ADC_BATT		1
#define APP_CONFIG_USBIN			1

/****************************************************************************************/
/* UART2 configuration to use with arch_console print messages                          */
/****************************************************************************************/
// Define UART2 Tx Pad
#if defined (__DA14531__)
    #define UART2_TX_PORT           GPIO_PORT_0
    #define UART2_TX_PIN            GPIO_PIN_6
#else
    #define UART2_TX_PORT           GPIO_PORT_0
    #define UART2_TX_PIN            GPIO_PIN_4
#endif

// Define UART2 Settings
#define UART2_BAUDRATE              UART_BAUDRATE_115200
#define UART2_DATABITS              UART_DATABITS_8
#define UART2_PARITY                UART_PARITY_NONE
#define UART2_STOPBITS              UART_STOPBITS_1
#define UART2_AFCE                  UART_AFCE_DIS
#define UART2_FIFO                  UART_FIFO_EN
#define UART2_TX_FIFO_LEVEL         UART_TX_FIFO_LEVEL_0
#define UART2_RX_FIFO_LEVEL         UART_RX_FIFO_LEVEL_0

/****************************************************************************************/
/* SPI configuration                                                                    */
/****************************************************************************************/
// Define SPI Pads
#if defined (__DA14531__)
    #define SPI_EN_PORT             GPIO_PORT_0
    #define SPI_EN_PIN              GPIO_PIN_1

    #define SPI_CLK_PORT            GPIO_PORT_0
    #define SPI_CLK_PIN             GPIO_PIN_4

    #define SPI_DO_PORT             GPIO_PORT_0
    #define SPI_DO_PIN              GPIO_PIN_0

    #define SPI_DI_PORT             GPIO_PORT_0
    #define SPI_DI_PIN              GPIO_PIN_3

#elif !defined (__DA14586__)
    #define SPI_EN_PORT             GPIO_PORT_0
    #define SPI_EN_PIN              GPIO_PIN_3

    #define SPI_CLK_PORT            GPIO_PORT_0
    #define SPI_CLK_PIN             GPIO_PIN_0

    #define SPI_DO_PORT             GPIO_PORT_0
    #define SPI_DO_PIN              GPIO_PIN_6

    #define SPI_DI_PORT             GPIO_PORT_0
    #define SPI_DI_PIN              GPIO_PIN_5
#endif

// Define SPI Configuration
    #define SPI_MS_MODE             SPI_MS_MODE_MASTER
    #define SPI_CP_MODE             SPI_CP_MODE_0
    #define SPI_WSZ                 SPI_MODE_8BIT
    #define SPI_CS                  SPI_CS_0

#if defined (__DA14531__)
    #define SPI_SPEED_MODE          SPI_SPEED_MODE_4MHz
    #define SPI_EDGE_CAPTURE        SPI_MASTER_EDGE_CAPTURE
#else // (DA14585, DA14586)
    #define SPI_SPEED_MODE          SPI_SPEED_MODE_4MHz
#endif


/****************************************************************************************/
/* SPI Flash configuration                                                              */
/****************************************************************************************/
#if !defined (__DA14586__)
#define SPI_FLASH_DEV_SIZE          (256 * 1024)
#endif


/****************************************************************************************/
/* Button configuration                                                                 */
/****************************************************************************************/
#if defined (__DA14531__)
    #define GPIO_BUTTON_PORT        GPIO_PORT_0
    #define GPIO_BUTTON_PIN         GPIO_PIN_11
		
    #define GPIO_LED1_RED_PORT        GPIO_PORT_0
    #define GPIO_LED1_RED_PIN         GPIO_PIN_0
		
		#define GPIO_LED2_RED_PORT        GPIO_PORT_0
    #define GPIO_LED2_RED_PIN         GPIO_PIN_1
		
		#define GPIO_LED3_RED_PORT        GPIO_PORT_0
    #define GPIO_LED3_RED_PIN         GPIO_PIN_4			//ying jian LED2/LED3 fan le 
		
		#define GPIO_LED_BLUE_PORT        GPIO_PORT_0
    #define GPIO_LED_BLUE_PIN         GPIO_PIN_3
		
		#define GPIO_LR_GREED_PORT        GPIO_PORT_0
    #define GPIO_LR_GREED_PIN         GPIO_PIN_5
		
		#define GPIO_LR_RED_PORT        GPIO_PORT_0
    #define GPIO_LR_RED_PIN         GPIO_PIN_9
		
		#define GPIO_BL_LED_PORT        GPIO_PORT_0
    #define GPIO_BL_LED_PIN         GPIO_PIN_8
		
		#define GPIO_USB_IN_PORT        GPIO_PORT_0
    #define GPIO_USB_IN_PIN         GPIO_PIN_7

		#define GPIO_BAT_AD_PORT        GPIO_PORT_0
    #define GPIO_BAT_AD_PIN         GPIO_PIN_6
	
#else
    #define GPIO_BUTTON_PORT        GPIO_PORT_1
    #define GPIO_BUTTON_PIN         GPIO_PIN_1
#endif


/****************************************************************************************/
/* Wake-up from hibernation configuration                                               */
/****************************************************************************************/
#if defined (__DA14531__)
    #define HIB_WAKE_UP_PORT        GPIO_PORT_0
    #define HIB_WAKE_UP_PIN         GPIO_PIN_11
    #define HIB_WAKE_UP_PIN_MASK    (1 << HIB_WAKE_UP_PIN)
#endif


/****************************************************************************************/
/* Proximity Reporter pad configuration                                                 */
/****************************************************************************************/
#define USE_BAT_LEVEL_ALERT         0

#if defined (__DA14531__)
    #define GPIO_ALERT_LED_PORT     GPIO_PORT_0
    #define GPIO_ALERT_LED_PIN      GPIO_PIN_9
    #define GPIO_BAT_LED_PORT       GPIO_PORT_0
    #define GPIO_BAT_LED_PIN        GPIO_PIN_8
#else
    #define GPIO_ALERT_LED_PORT     GPIO_PORT_1
    #define GPIO_ALERT_LED_PIN      GPIO_PIN_0
    #define GPIO_BAT_LED_PORT       GPIO_PORT_1
    #define GPIO_BAT_LED_PIN        GPIO_PIN_2
#endif

/***************************************************************************************/
/* Production debug output configuration                                               */
/***************************************************************************************/
#if PRODUCTION_DEBUG_OUTPUT
#if defined (__DA14531__)
    #define PRODUCTION_DEBUG_PORT   GPIO_PORT_0
    #define PRODUCTION_DEBUG_PIN    GPIO_PIN_7
#else
    #define PRODUCTION_DEBUG_PORT   GPIO_PORT_2
    #define PRODUCTION_DEBUG_PIN    GPIO_PIN_5
#endif
#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Enable pad and peripheral clocks assuming that peripheral power domain
 *        is down. The UART and SPI clocks are set.
 ****************************************************************************************
 */
void periph_init(void);

/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 ****************************************************************************************
 */
void GPIO_reservations(void);

/**
 ****************************************************************************************
 * @brief Set gpio port function mode
 ****************************************************************************************
 */
void set_pad_functions(void);

#endif // _USER_PERIPH_SETUP_H_
