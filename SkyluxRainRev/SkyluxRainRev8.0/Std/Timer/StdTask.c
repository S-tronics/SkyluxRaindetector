/**********************************************************************************************************************/
/**
 * @file        timer\StdTask.c
 *
 * @author      Stijn Vermeersch
 * @date        22.12.2010
 *
 * @brief       Module for running repetitive tasks with a fixed period.
 *
 * Here, the definition of a task is like, calling a function on a specified time.\n
 * When task are registered, they are not started at that time.  Start and stop functions are seperately available.
 * The period of a registered task can be altered anytime.  Eventually, one can choose to kill a registered task, and
 * free up the RAM space used by it inside this module.  The amount of tasks which can be registered at the same time
 * is specified in AppConfig.h
 *
 *
 * \n<hr>
 * Copyright (c) 2010, TVH\n
 * All rights reserved.
 * \n<hr>\n
 */
/**********************************************************************************************************************/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; V E R I F Y    C O N F I G U R A T I O N
;---------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief  Defines the max number of tasks one wants to support
 */
//#ifndef TASK_COUNT
//    #error "TASK_COUNT not defined in AppConfig.h"
//#endif
/**********************************************************************************************************************/



/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hal_timer.h>
#include <hpl_tc_base.h>
#include <hpl_timer.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>

//DRIVER lib include section

//STANDARD lib include section
#include "StdTask.h"
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/

#define DEFAULT_TASK_PERIOD    10
#define TASK_COUNT             7
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
struct timer_descriptor				TIMER_0;
static uint16_t						stdtask_tick_period;
static uint16_t                     stdtask_count;
static struct timer_task                   stdtask_array[TASK_COUNT];
static struct timer_task*                  stdtask_first_task_ptr;
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void StdTaskInit(uint16_t tick_period_in_us)
{
    struct timer_task* task_ptr;

    stdtask_tick_period = tick_period_in_us;
    stdtask_count = 0;
	
	NVIC_SetPriority(TC0_IRQn, 1);
	
	_pm_enable_bus_clock(PM_BUS_APBC, TC0);
	_gclk_enable_channel(TC0_GCLK_ID, CONF_GCLK_TC0_SRC);

	timer_init(&TIMER_0, TC0, _tc_get_timer());

    task_ptr = &stdtask_array[0];
    while(task_ptr < &stdtask_array[TASK_COUNT])
    {
        task_ptr->interval		= DEFAULT_TASK_PERIOD;
        task_ptr->cb			= NULL;
		task_ptr->started		= false;
        task_ptr++;
    }
}
/*--------------------------------------------------------------------------------------------------------------------*/
TASK_HNDL StdTaskRegisterTask(uint32_t task_period_in_us, timer_cb_t callback)
{
    uint8_t i;
    struct timer_task* loper_task_ptr;

	
    stdtask_count++;
    loper_task_ptr = &stdtask_array[0];

    for(i = 0 ; i < TASK_COUNT ; i++)
    {
        if(stdtask_array[i].cb == NULL)
        {
            break;
        }
    }
    loper_task_ptr = &stdtask_array[i];

    loper_task_ptr->interval			= task_period_in_us;
    loper_task_ptr->cb					= callback;
	loper_task_ptr->mode				= TIMER_TASK_REPEAT;
	loper_task_ptr->started				= false;

    return i;
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool StdTaskStart(TASK_HNDL task)
{
    struct timer_task* task_ptr;
	
	timer_stop(&TIMER_0);
    if(task >= TASK_COUNT)
    {
        return false;
    }

    task_ptr = &stdtask_array[task];
    if(task_ptr->cb == NULL)
	{
		return false;
	}
	task_ptr->started = true;
    timer_add_task(&TIMER_0, task_ptr);
	timer_start(&TIMER_0);
	
    return true;
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool StdTaskStop(TASK_HNDL task)
{
    struct timer_task* task_ptr;
	
		
	if(task >= TASK_COUNT)
	{
		return false;
	}
		
	task_ptr = &stdtask_array[task];
	if(task_ptr->started == true)			
	{
		timer_stop(&TIMER_0);
		if(task_ptr->cb == NULL)
		{
			return false;
		}
	
		task_ptr->started = false;
		timer_remove_task(&TIMER_0, task_ptr);
		timer_start(&TIMER_0);
	}
    return true;
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool StdTaskStopAll(void)
{
	uint8_t i = 0;
	struct timer_task* task_ptr;
	
	timer_stop(&TIMER_0);
	
	//for(i=0; i<TASK_COUNT; i++)
	//{
		//if(stdtask_array[i].started == true)
		//{
			//task_ptr = &stdtask_array[i];
			//timer_remove_task(&TIMER_0, task_ptr);
		//}
	//}
	//timer_start(&TIMER_0);
	return true;
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool StdTaskRestartAllStarted(void)
{
	uint8_t i = 0;
	struct timer_task* task_ptr;
	
	timer_stop(&TIMER_0);
	//for(i=0; i<TASK_COUNT; i++)
	//{
		//if(stdtask_array[i].started == true)
		//{
			//task_ptr = &stdtask_array[i];
			//timer_add_task(&TIMER_0, task_ptr);
		//}
	//}
	//timer_start(&TIMER_0);
	return true;
}
/*--------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/
