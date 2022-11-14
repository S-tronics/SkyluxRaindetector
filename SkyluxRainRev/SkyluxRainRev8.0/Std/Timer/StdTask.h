/**********************************************************************************************************************/
/**
 * @file        timer\StdTask.h
 *
 * @author      Stijn Vermeersch
 * @date        22.12.2010
 *
 * @brief       Module for handling tasks on a fixed tick period.
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
#ifndef TIMER__STDTASK_H
#define TIMER__STDTASK_H
/**********************************************************************************************************************/



/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
#include <hal_timer.h>
#include <stdint.h>
#include <stdbool.h>
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   S Y M B O L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief   Hook prototype for the task control function
 */
typedef void (*TASK_CALLBACK_FUNC)(void *data_ptr);

/**
 * @brief   Task handle type definition
 */
typedef uint8_t TASK_HNDL;

/**
 * @brief   Definition of an invalid task handle
 */
#define INVALID_TASK_HNDL       255
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief   Initialise function for this module
 *
 * Inits global variables and registers itself to the module manager.  It also stores the specified tick period;
 * this is the time between two consecutive calls to \ref StdTaskTick().
 *
 * @param   tick_period_in_us     the time between two consecutive calls to \ref StdTaskTick()
 */
void StdTaskInit(uint16_t tick_period_in_us);

/**
 * @brief   Tick function for giving this task module a time base.
 *
 * This function must be called on a regular time base with a fixed interval which was specified
 * while calling \ref StdTaskInit()
 */
void StdTaskTick(void);

/**
 * @brief   Function to handle tasks which don't need to be handled directly from the \ref StdTaskTick()
 */
void StdTaskHandler(void);

/**
 * @brief   Registers a new task.
 *
 * @param   task_period_in_us     task period expressed in us
 * @param   callback              pointer to a function that will be called everytime the task is asserted;
 *                                must not be NULL
 * @param   data_ptr              VPTR pointer to a data structure that will be passed along with
 *                                the call to the callback function; this may be NULL
 * @param   priority              task priority : a value between 0 and 255\n
 *                                <b>If the priority is greater than 127 then the TASK is handled by the background
 *                                handler.  Otherwise the TASK is handled directly from \ref StdTaskTick().</b>
 *
 * @return  the handle for the newly registered task
 *
 * @remark  When the registration fails, an exception is thrown.\n
 *          When using tasks with a background priority, \ref StdTaskHandler MUST be called from the background loop
 */
TASK_HNDL StdTaskRegisterTask(uint32_t task_period_in_us, timer_cb_t callback);

/**
 * @brief   Function to change the period of the specified task
 *
 * The internal down-counter for this task is NOT altered.  If the task is running, then this new period will become
 * active on the next task period match.
 *
 * @param   task                  handle to the task whos period should be altered
 * @param   task_period_in_us     the new task period expressed in us
 *
 * @return  FALSE if the handle is not a valid task handle; TRUE otherwise
 */
bool StdTaskSetPeriod(TASK_HNDL task, uint32_t task_period_in_us);

/**
 * @brief   Starts an existing task
 *
 * The internal down-counter for this task is reset to the task period and from now on it is handled by
 * \ref StdTaskTick().
 *
 * @param   task                  handle to the task which should be started
 *
 * @return  FALSE if the handle is not a valid task handle; TRUE otherwise
 */
bool StdTaskStart(TASK_HNDL task);

/**
 * @brief   Stops an existing task
 *
 * The specified task is no longer handled by \ref StdTaskTick()
 *
 * @param   task                  handle to the task which should be stopped
 *
 * @return  FALSE if the handle is not a valid task handle; TRUE otherwise
 */
bool StdTaskStop(TASK_HNDL task);

/**
 * @brief   Stops and unregisters an existing task
 *
 * @param   task                  handle to the task which should be killed
 *
 * @return  FALSE if the handle is not a valid task handle; TRUE otherwise
 */
bool StdTaskKill(TASK_HNDL task);

/**
 * @brief   Function to check if the task is running
 *
 * @param   task : the task to check
 *
 * @return TRUE if the task is running
 */

bool StdTaskIsTaskRunning(TASK_HNDL task);

/**
 * @brief   Function to check if the task is active
 *
 * @param   task : the task to check
 *
 * @return TRUE if the task is active
 */
bool StdTaskIsTaskActive(TASK_HNDL task);

//Debug 
bool StdTaskStopAll(void);
bool StdTaskRestartAllStarted(void);
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   S T A T I C   I N L I N E   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/



#endif /* TIMER__STDTASK_H */
