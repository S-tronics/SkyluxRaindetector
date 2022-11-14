/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD20 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define PA02 GPIO(GPIO_PORTA, 2)		//ADC_0 (SNS_LDR)
#define PA08 GPIO(GPIO_PORTA, 8)		//SNS_SCL
#define PA09 GPIO(GPIO_PORTA, 9)		//SNS_SDA
#define PA10 GPIO(GPIO_PORTA, 10)		//SNS_MT0
#define PA11 GPIO(GPIO_PORTA, 11)		//SNS_RESET
#define PA15 GPIO(GPIO_PORTA, 15)		//UART_TX
#define PA14 GPIO(GPIO_PORTA, 14)		//UART_RX
#define PA16 GPIO(GPIO_PORTA, 16)		//BTLE_SDA
#define PA17 GPIO(GPIO_PORTA, 17)		//BTLE_CLK
#define PA18 GPIO(GPIO_PORTA, 18)		//SNS_Temp
#define PA19 GPIO(GPIO_PORTA, 19)		//BTLE_Pending
#define PA22 GPIO(GPIO_PORTA, 22)		//uC_Out
#define PA23 GPIO(GPIO_PORTA, 23)		//uC_Relais
#define PA24 GPIO(GPIO_PORTA, 24)		//uC_PG (Power Good from SMPS)
#define PA27 GPIO(GPIO_PORTA, 27)		//Runled

//debug pin
#define PA04 GPIO(GPIO_PORTA, 4)

#endif // ATMEL_START_PINS_H_INCLUDED
