/**********************************************************************************************************************/
/**
 * @file        AppSensor.h
 *
 * @author      Stijn Vermeersch
 * @date        03.07.2018
 *
 * @brief      
 *
 *
 *
 * \n<hr>\n
 * Copyright (c) 2018, S-tronics\n
 * All rights reserved.
 * \n<hr>\n
 */
/**********************************************************************************************************************/
#ifndef APP__SENSOR_H
#define APP__SENSOR_H
/**********************************************************************************************************************/



/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   S Y M B O L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/
typedef enum
{
	DET_RAIN = 0,
	DET_LIGHT = 1,
	RAIN_VALUE = 2,
	LIGHT_VALUE = 3,
	SERIAL = 4,
    RELAIS = 5
}
APP_SNS_TEST;

typedef enum
{
	MOD_IDLE,
	MOD_DRYING,
	MOD_RAIN,
	MOD_MOISTY,
	MOD_SNOW,
	MOD_SAT
}
APP_SENS_MODUS;
typedef enum
{
	TEMP_IDLE,
	TEMP_SNOW,
	TEMP_MOISTY,
	TEMP_RAIN
}
APP_TEMP_MODUS;

typedef enum
{
	MIST		= 1,
	FOG			= 2,
	DRIZZLE		= 3,
	LIGHTRAIN	= 4,
	AVGRAIN		= 5,
	HVYRAIN		= 6,
	INTSRAIN	= 7,
	SHOWER		= 8,
	SATURATED   = 19,
	IDLE		= 20
}
RAINSTATE;

typedef enum
{
	MOISTY		= 9,	
	DRYSNOW		= 10
}
SNOWSTATE;
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
extern TASK_HNDL    trend_task;
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief   Initialiser for the sensorInterface
 *
 * Initialises everything around the used sensors on SkyRain.\n
 *
 */
void AppSensorInit(void);

uint16_t AppSensorGetRainValue(void);

RAINSTATE AppSensorGetRainState(void);

uint8_t AppSensorSetTreshold(bool updown, uint8_t treshvalue);

uint8_t AppSensorGetActivity(void);

uint16_t AppSensorGetBaseValue(void);

uint32_t AppSensorGetTempValue(void);
/**
 * @brief   Handler for everything around sensor on SkyRain.\n
 *
 * Handles all the repetitive execution sequences on SkyRain. \n
 *
 */
void AppSensorHandler(void);

void AppSensorTrend(void);

/**
 * @brief   Function to receive the lightsensor value
 *
 * Function reads the lightsensor value.\n
 *
 * @return		:	The Lightsensor value in lux. AppSensorHandler should be executed first. \n
 *
 */
uint16_t AppSensorGetLightValue(void);

uint16_t AppSensorGetLightHysteresis(void);

/***********************************************************************************************************************
; E X P O R T E D   S T A T I C   I N L I N E   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/

#endif /* APP__SENSOR_H */