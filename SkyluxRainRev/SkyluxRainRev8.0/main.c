/*
 * File:   Appmain.c
 * Author: S-tronics - Stijn Vermeersch
 *
 * Created on 1 May 2019
 */

/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <hal_init.h>
#include <hal_delay.h>
#include "atmel_start_pins.h"
#include "..\..\Std\I2C\StdMtch112.h"
#include "..\..\Std\Temperature\StdDS18S20.h"
#include "..\..\Std\BTLE\StdCasambi.h"
#include "..\..\Std\Timer\StdTask.h"
#include "..\..\Std\Timer\StdDelay.h"
#include "..\..\Drv\DrvFlash.h"
#include "..\..\Drv\DrvI2c.h"
#include "..\..\Drv\DrvAdc.h"
#include "..\..\Drv\DrvSci.h"
#include "..\..\Drv\DrvWdt.h"
#include "AppSensor.h"
#include "AppIO.h"
#include "AppCasambi.h"
#include "AppCasambiUpdate.h"
/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
void AppMainInitAllObjects(void);
void AppMainBackGroundLoop(void);
void AppMainStartupTask(const struct timer_task *const timer_task);
void AppMainHeartBeatTask(const struct timer_task *const timer_task);
/**********************************************************************************************************************/
/***********************************************************************************************************************
; L O C A L   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
static	TASK_HNDL	startup_task;       //Task to intialise everything before executing
static  TASK_HNDL	hb_task;		
bool	startupdone = false;
bool	configdone = false;
/***********************************************************************************************************************
; L O C A L   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void AppMainInitAllObjects()
{
	init_mcu();
	//System initialisation 
	delay_init(SysTick);
	DrvFlashInit();
	DrvI2C_0_init();
	DrvI2C_1_init();
	DrvAdc_0_init(0);			//Init ADC0
	DrvWdtInit();
	DrvWdtFeed();
	DrvWdtSetTimeout(10);									//ms
	
	//Set Runled
	gpio_set_pin_direction(PA27, GPIO_DIRECTION_OUT);
	
	StdTaskInit(200);
	StdMtch112Init(PA11, PA10);
    StdMtch112Reset();
	
	startup_task = StdTaskRegisterTask(300, AppMainStartupTask);				//Unit is ms
	hb_task = StdTaskRegisterTask(1000, AppMainHeartBeatTask);
	StdTaskStart(startup_task);
	StdDS18S20Init(PA18);
	AppIoInit();
	AppSensorInit();
	AppCasambiInit();
	AppCasambiUpdateInit();
	
	//debug
	gpio_set_pin_direction(PA04, GPIO_DIRECTION_OUT);
	
	//ResetCause
	static uint8_t debugvalue = 0;
	debugvalue = (uint8_t)hri_pm_get_RCAUSE_reg(PM, (hri_pm_rcause_reg_t) 0xff);
	AppCasambiDebug(4, &debugvalue, 1);
	
}
/**********************************************************************************************************************/
void AppMainBackGroundLoop(void)
{
	if(startupdone == true)
	{
		if(configdone == true)
		{
			StdCasambiHandler();
			AppCasambiHandler();
			if(!AppCasambiUpdateState())			//Not updating
			{
				AppSensorHandler();
				DrvWdtFeed();
			}
			else 
			{
				StdCasambiUpdateHandler();
				
			}
		}
	}
}
/**********************************************************************************************************************/
void AppMainHeartBeatTask(const struct timer_task *const timer_task)
{
	gpio_toggle_pin_level(PA27);				//Toggle Run-led
	if(configdone == false)
	{
		StdMtch112Config();
		configdone = true;
	}
}
/**********************************************************************************************************************/
void AppMainStartupTask(const struct timer_task *const timer_task)
{
	StdTaskStop(startup_task);
	StdMtch112ResetCmd();
	StdTaskStart(hb_task);
	startupdone = true;
	DrvWdtEnable();
}
/**********************************************************************************************************************/
int main(void)
{
	AppMainInitAllObjects();
	//Start Application code.
	for(;;) 
	{
		AppMainBackGroundLoop();
	}
}

/**********************************************************************************************************************/

