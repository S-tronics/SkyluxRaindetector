/*
 * File:   AppSensor.c
 * Author: stronics
 *
 * Created on 30 juni 2018, 13:11
 */

/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "..\..\Drv\DrvFlash.h"
#include "..\..\Std\Timer\StdTask.h"
#include "..\..\Std\BTLE\StdCasambi.h"
#include "atmel_start_pins.h"
#include "AppCasambi.h"
#include "AppSensor.h"
#include "AppUpdate.h"

/**********************************************************************************************************************/

/***********************************************************************************************************************
; V E R I F Y    C O N F I G U R A T I O N
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/

#define		SEND_CASAMBI
#define		DEBUG_CASAMBI
//#define		LOG_CASAMBI
//#define		SEND_SCI

#define		ADDRESS_CAS_ID			0x2FFC

#define		CAS_REG_RAINVALUE 		1
//#define		CAS_REG_RAINSTATE		1
#define		CAS_REG_RAINTRESHUP		2
#define		CAS_REG_RAINTRESHDWN	3
#define		CAS_REG_TEMPVALUE		4
#define		CAS_REG_LIGHTVALUE		5
#define		CAS_REG_ACTREG			6	

#define		SIGN		0x80

#define		CAS_TAG_RAIN			0x00
#define		CAS_TAG_TEMP			0x01
#define		CAS_TAG_LIGHT			0x02
#define		CAS_TAG_STATE			0x03		//Activity Register

#define		PARAM_MASK				0x1F

//Vendormessage
#define		CAS_DEVICECODE			49
#define		CAS_MSGTYPE				0x01
#define		B_ID					2
#define		B_RAINSTATE				3
#define		B_TEMPVALUE				4
#define		B_LIGHTVALUE			5
#define		B_ACTIVITY				7
#define		VM0_LENGTH				8
#define		VM1_LENGTH				8

//Tresholds
#define		FIXED_TRESH_UP			10
#define		FIXED_TRESH_DWN			4

//Light value
#define		LIGHT_VALUE_HYSTERESIS	100

//Firmware updates


/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/
typedef struct 
{
	uint8_t	sensor_ID;
	uint8_t rainsensorstate;
	uint8_t rainsensortresholdup;
	uint8_t rainsensortresholddown;
	uint8_t temperaturevalue;
	uint16_t lightvalue;
	uint8_t activityreg;
	uint8_t testreg;
	uint8_t idletime;
}
CAS_REG;

typedef enum
{
	RAINSENSORTRESH_UP		= 0,
	RAINSENSORTRESH_DWN		= 1,
	SENSOR_ID				= 2,
	PAR_TIME_IDLE			= 3,
	TEST					= 4
}
CAS_PARAM;

typedef enum
{
	RAINSENSOR = 0,
	TEMPERATURESENSOR = 1,
	LIGHTSENSOR = 2,
	STATUS = 3
	
}
CAS_SENSOR;
/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief   Set a parametervalue coming from Casambi Server
 *
 *	Set a parameter with data that is sent by the server (Casambi) (Server -> Client)
 *
 *	@param	tag:		tag of the parameter defined in Casambi Communication
 *	@param	data:		value of that parameter
 *
 */
void AppCasambiSetParameter(uint8_t tag, uint32_t data);
/**
 * @brief   Get a parametervalue asked by the Casambi Server
 *
 *	Get a parameter that was asked for from the server. This is the answer from the 
 *  question from the server (Casambi) (Client -> Server)
 *
 *	@param	tag:		tag of the parameter defined in Casambi Communication
 *
 *	@return parametervalue
 */
uint8_t AppCasambiGetParameter(uint8_t tag, uint8_t length);
/**
 * @brief   Get a sensorvalue asked by the Casambi Server
 *
 *	Get a sensorvalue that was asked for from the server. This is the answer from the
 *	question form the server (Casambi) (Client -> Server)
 *
 *	@param	tag:		tag of the parameter defined in Casambi Communication
 *
 */
void AppCasambiGetSensorValue(uint8_t tag);

void AppCasambiGetParameterFromServer(void);
/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
static CAS_REG cas_reg;

static TASK_HNDL	cas_task;
static uint8_t		param_mask = 0x00;
static bool			vm0_flag = false;
static bool			vm1_flag = false;
static bool			paired_flag = false;
static bool			getparam_flag = false;
static uint8_t		vendorarray0[8];
static uint8_t		vendorarray1[8];

static bool			testmode = false;

//Version update should be stored here
static uint8_t		major = 1;
static uint8_t		minor = 1;

//Casambi VM1 debugging
static bool			debugblocking = false;
/**********************************************************************************************************************/

/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void AppCasambiSetParameter(uint8_t tag, uint32_t data)
{
	static uint8_t temp = 0;
	switch((CAS_PARAM)tag)
	{
		case RAINSENSORTRESH_UP:
			AppCasambiRainsensorTresholdValueUp(false, (uint8_t)data);
			param_mask |= 1 << (uint8_t)RAINSENSORTRESH_UP;
			break;
		case RAINSENSORTRESH_DWN:
			AppCasambiRainsensorTresholdValueDown(false, (uint8_t)data);
			param_mask |= 1 << (uint8_t)RAINSENSORTRESH_DWN;
			break;
		case SENSOR_ID:
			AppCasambiSensorID(false, (uint8_t)data);
			param_mask |= 1 << (uint8_t)SENSOR_ID;
			break;
		case PAR_TIME_IDLE:
			AppCasambiRainsensorIdleTimeValue(false, (uint8_t)data + 10);
			param_mask |= 1 << (uint8_t)PAR_TIME_IDLE;
			break;
		case TEST:
			temp = (uint8_t)data;
			AppCasambiTest(false, &temp);			//Write testmode
			param_mask |= 1 << (uint8_t)TEST;
			break;
		default:
			break;
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t AppCasambiGetParameter(uint8_t tag, uint8_t length)  
{
	static uint8_t temp = 0;
	uint8_t dummy = 0;
	switch((CAS_PARAM)tag)
	{
		case RAINSENSORTRESH_UP:
			temp = AppCasambiRainsensorTresholdValueUp(true, dummy);
			break;
		case RAINSENSORTRESH_DWN:
			temp = AppCasambiRainsensorTresholdValueDown(true, dummy);
			break;
		case SENSOR_ID:
			temp = AppCasambiSensorID(true, dummy);
			break;
		case PAR_TIME_IDLE:
			AppCasambiRainsensorIdleTimeValue(true, dummy);
			break;
		case TEST:
			AppCasambiTest(true, &temp);			//Write testmode
			break;
		default:
			break;
	}
	return temp;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void AppCasambiGetSensorValue(uint8_t tag)
{
	switch((CAS_SENSOR)tag)
	{
		case RAINSENSOR:
			AppCasambiRainsensorState(true, 0);
			break;
		case TEMPERATURESENSOR:
			AppCasambiTemperatureValue(true, 0);
			break;
		case LIGHTSENSOR:
			AppCasambiLightValue(true, 0);
			break;
		case STATUS:
			AppCasambiActivity(true, 0, 0);
			break;
		default:
			break;
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void AppCasambiGetParameterFromServer(void)
{	
	StdCasambiGetParameter();

}
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void AppCasambiInit(void)
{
	//CAS_REG *cas_reg_Ptr;
	//cas_reg_Ptr = &cas_reg;
	StdCasambiInit(PA19, AppCasambiSetParameter, AppCasambiGetParameter, AppCasambiGetSensorValue);
	cas_reg.sensor_ID = 0;
	cas_reg.rainsensortresholdup = FIXED_TRESH_UP;
	cas_reg.rainsensortresholddown = FIXED_TRESH_DWN;
	cas_reg.rainsensorstate = 0;
	cas_reg.temperaturevalue = 0;
	cas_reg.lightvalue = 0;	
	cas_reg.idletime = 0;
	
	cas_task = StdTaskRegisterTask(30000, AppCasambiPush);		//Every 1 minute there is a Casambi Push
	StdTaskStart(cas_task);

	vendorarray0[0] = CAS_DEVICECODE;
	vendorarray0[1] = CAS_MSGTYPE;
#ifdef LOG_CASAMBI
	vendorarray1[0] = CAS_DEVICECODE;
	vendorarray1[1] = CAS_MSGTYPE;
#endif

	//StdCasambiGetPairedFromCasambi();
	//AppUpdateGetFirmwareVersionFromFlash(&major, &minor);
	StdSetFirmwareVersion(major, minor);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void AppCasambiPush(const struct timer_task *const timer_task)
{
#ifdef SEND_CASAMBI
	
	paired_flag = true;
	//StdCasambiGetPairedFromCasambi();
	StdTaskStop(cas_task);
	if(StdCasambiGetPaired())
	{
		getparam_flag = true;
		
		if(param_mask == PARAM_MASK)
		{
			vm0_flag = true;
			//StdCasambiSetVendorMessage(VM0_OPCODE, vendorarray0, VM0_LENGTH);
		}	
	}
	StdTaskStart(cas_task);
#else
	//AppSciWriteRainState();
	//AppSciWriteLightValue();
	//AppSciWriteTempValue();
#endif
}
/*--------------------------------------------------------------------------------------------------------------------*/
void AppCasambiHandler(void)
{
	if(paired_flag)
	{
		StdCasambiGetPairedFromCasambi();
		paired_flag = false;
	}
	if(vm0_flag)
	{
		StdCasambiSetVendorMessage(VM0_OPCODE, vendorarray0, VM0_LENGTH);
		vm0_flag = false;
	}
	if(vm1_flag)
	{
		StdCasambiSetVendorMessage(VM1_OPCODE, vendorarray1, VM1_LENGTH);
		vm1_flag = false;
	}
	if(getparam_flag)
	{
		AppCasambiGetParameterFromServer();
		getparam_flag = false;
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void AppCasambiGetVendorMessage(const uint8_t opcode, const uint8_t* bytearray, const uint8_t length)  //Server (Casambi) Sets Parameter
{
	switch(opcode)
	{
		case VM0_OPCODE:
			if(length <= 8)
			{
				memcpy(vendorarray0, bytearray, length);
			}			
			break;
		case VM1_OPCODE:
			if(length <= 8)
			{
				memcpy(vendorarray1, bytearray, length);
			}
			break;
		default:
			break;	
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void AppCasambiSetVendorMessage(uint8_t opcode, uint8_t length)
{
	if(StdCasambiGetPaired())
	{
		if(param_mask == PARAM_MASK)
		{
			switch(opcode)
			{
				case VM0_OPCODE:
#ifdef SEND_CASAMBI
					vm0_flag = true;
					//StdCasambiSetVendorMessage(opcode, vendorarray0, length);
#else

#endif
					break;
				case VM1_OPCODE:
#ifdef SEND_CASAMBI
					vm1_flag = true;
					//StdCasambiSetVendorMessage(opcode, vendorarray1, length);
#else

#endif
					break;
				default:
					break;
			}
		}
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t AppCasambiRainsensorState(bool rw, uint8_t cas_state)
{	
	if(rw)			//read
	{
		if((cas_reg.activityreg & (1 << ACTIVITY_RDY)) == (1 << ACTIVITY_RDY))
		{
#ifdef SEND_CASAMBI			
			if(StdCasambiGetPaired())
			{
				StdCasambiSetSensorValue(CAS_TAG_RAIN, 1, AppSensorGetRainState);
			}
#else
			//AppSciWriteRainState();
#endif
			return cas_reg.rainsensorstate;
		}
	}	
	else            //write to internal register
	{
		if((cas_reg.activityreg & (1 << ACTIVITY_RDY)) == (1 << ACTIVITY_RDY))
		{
			if(cas_reg.rainsensorstate != cas_state)
			{
				vendorarray0[B_RAINSTATE] = cas_state;
#ifdef LOG_CASAMBI
				vendorarray1[B_RAINSTATE] = cas_state;
#endif
				cas_reg.rainsensorstate = cas_state;
#ifdef SEND_CASAMBI	
				if(StdCasambiGetPaired())
				{
					StdCasambiSetSensorValue(CAS_TAG_RAIN, 1, AppSensorGetRainState);
					AppCasambiSetVendorMessage(VM0_OPCODE, VM0_LENGTH);
#ifdef LOG_CASAMBI
					AppCasambiSetVendorMessage(VM1_OPCODE, VM1_LENGTH);
#endif
				}
#else
				AppSciWriteRainState();
#endif
			}
			cas_reg.rainsensorstate = cas_state;
			return cas_state;
		}
	}
	
	return cas_state;
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t AppCasambiSensorID(bool rw, uint8_t cas_value)
{
	if(rw)			//Read
	{
		return cas_reg.sensor_ID;
	}
	else            //Write
	{
		if(cas_reg.sensor_ID != cas_value)
		{
			cas_reg.sensor_ID = cas_value;
			vendorarray0[B_ID] = cas_value;
#ifdef LOG_CASAMBI
			vendorarray1[B_ID] = cas_value;
#endif
			DrvFlashWrite(ADDRESS_CAS_ID, &cas_reg.sensor_ID, 1);
		}
	}
	return 0;
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t AppCasambiRainsensorTresholdValueUp(bool rw, uint8_t cas_value)
{
	uint8_t treshold = 0;
	
	if(rw)			//Read
	{
		if(StdCasambiGetPaired())
		{
			treshold = cas_reg.rainsensortresholdup;
		}
		else
		{
			treshold = FIXED_TRESH_UP;
		}
	}
	else			//Write
	{		
		cas_reg.rainsensortresholdup = cas_value;
		treshold = cas_value;
	}
	if(cas_reg.rainsensortresholdup < cas_reg.rainsensortresholddown)
	{
		AppCasambiActivity(false, ACTIVITY_ERRRAIN, true);
	}
	else
	{
		AppCasambiActivity(false, ACTIVITY_ERRRAIN, false);
		AppCasambiActivity(false, ACTIVITY_ERR, false);
		AppCasambiActivity(false, ACTIVITY_RDY, true);				//Set activity register
	}
	return treshold;
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t AppCasambiRainsensorTresholdValueDown(bool rw, uint8_t cas_value)
{
	uint8_t treshold = 0;
	if(rw)			//Read
	{
		if(StdCasambiGetPaired())
		{
			treshold = cas_reg.rainsensortresholddown;
		}
		else
		{
			treshold = FIXED_TRESH_DWN;
		}
	}
	else			//Write
	{
		cas_reg.rainsensortresholddown = cas_value;
		treshold = cas_value;
	}
	if(cas_reg.rainsensortresholddown > cas_reg.rainsensortresholdup)
	{
		AppCasambiActivity(false, ACTIVITY_ERRRAIN, true);
	}
	else
	{
		AppCasambiActivity(false, ACTIVITY_ERRRAIN, false);
		AppCasambiActivity(false, ACTIVITY_ERR, false);
		AppCasambiActivity(false, ACTIVITY_RDY, true);				//Set activity register
	}
	return treshold;
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t AppCasambiRainsensorIdleTimeValue(bool rw, uint8_t idletime)
{
	uint8_t time = 0;
	
	if(rw)			//Read
	{
		if(StdCasambiGetPaired()) time = cas_reg.idletime;		
		else time = 0;
	}				
	else			//Write
	{
		cas_reg.idletime = idletime;
	}
	return time;
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t AppCasambiTemperatureValue(bool rw, uint8_t cas_value)
{
	if(rw)				//Read
	{
		if((cas_reg.activityreg & (1 << ACTIVITY_RDY)) == (1 << ACTIVITY_RDY))
		{
#ifdef SEND_CASAMBI
			if(StdCasambiGetPaired())
			{
				StdCasambiSetSensorValue(CAS_TAG_TEMP, 1, AppSensorGetTempValue);
			}
#else
			AppSciWriteTempValue();
#endif
			return cas_reg.temperaturevalue;
		}
	}
	else               //Write
	{
		if((cas_reg.activityreg & (1 << ACTIVITY_RDY)) == (1 << ACTIVITY_RDY))
		{
			if(cas_reg.temperaturevalue != cas_value)
			{
				vendorarray0[B_TEMPVALUE] = cas_value;
#ifdef LOG_CASAMBI
				vendorarray1[B_TEMPVALUE] = cas_value;
#endif
//#ifdef SEND_CASAMBI
				//if(StdCasambiGetPaired())
				//{
					//AppCasambiSetVendorMessage(VM0_OPCODE, VM0_LENGTH);
//#ifdef LOG_CASAMBI
					//AppCasambiSetVendorMessage(VM1_OPCODE, VM1_LENGTH);
//#endif
					//StdCasambiSetSensorValue(CAS_TAG_TEMP, 1, AppSensorGetTempValue);
				//}
//#else
			//AppSciWriteTempValue();
			////AppCasambiSetVendorMessage(VM0_OPCODE, VM0_LENGTH);
//#endif
			}
			cas_reg.temperaturevalue = cas_value;
			return cas_value;
		}
	}
	
	return cas_value;
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint16_t AppCasambiLightValue(bool rw, uint16_t cas_value)
{
	if(rw)			//Read
	{
		if((cas_reg.activityreg & (1 << ACTIVITY_RDY)) == (1 << ACTIVITY_RDY))
		{
#ifdef SEND_CASAMBI
			if(StdCasambiGetPaired())
			{
				StdCasambiSetSensorValue(CAS_TAG_LIGHT, 2, AppSensorGetLightValue);
			}
#else		
			//AppSciWriteLightValue();
#endif
			return cas_reg.lightvalue;
		}
	}
	else           //Write
	{
		if((cas_reg.activityreg & (1 << ACTIVITY_RDY)) == (1 << ACTIVITY_RDY))
		{
			if((cas_value <=(cas_reg.lightvalue - AppSensorGetLightHysteresis())) || (cas_value >= (cas_reg.lightvalue + AppSensorGetLightHysteresis())))
			{
				vendorarray0[B_LIGHTVALUE] = (uint8_t)(cas_value & 0x00FF);
				vendorarray0[B_LIGHTVALUE + 1] = (uint8_t)((cas_value >> 8) & 0x00FF);
#ifdef LOG_CASAMBI
				vendorarray1[B_LIGHTVALUE] = (uint8_t)(cas_value & 0x00FF);
				vendorarray1[B_LIGHTVALUE + 1] = (uint8_t)((cas_value >> 8) & 0x00FF);
#endif				
//#ifdef SEND_CASAMBI
				//if(StdCasambiGetPaired())
				//{
					//AppCasambiSetVendorMessage(VM0_OPCODE, VM0_LENGTH);
//#ifdef LOG_CASAMBI
					//AppCasambiSetVendorMessage(VM1_OPCODE, VM1_LENGTH);
//#endif
					//StdCasambiSetSensorValue(CAS_TAG_LIGHT, 2, AppSensorGetLightValue);
				//}
//#else
			//AppSciWriteLightValue();
			////AppCasambiSetVendorMessage(VM0_OPCODE, VM0_LENGTH);
//#endif
			}
			cas_reg.lightvalue = cas_value;
			return cas_value;
		}
	}
	
	return cas_value;
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t AppCasambiActivity(bool rw, uint8_t act, bool set)
{
	static uint8_t temp = 0;
	temp = cas_reg.activityreg;
	
	if(rw)		//read
	{
#ifdef SEND_CASAMBI
			if(StdCasambiGetPaired())
			{
				StdCasambiSetSensorValue(CAS_TAG_STATE, 1, AppCasambiGetActivity);
			}
#else
			
#endif
		return cas_reg.activityreg;
	}
	else        //write
	{
		switch (act)
		{
			case ACTIVITY_ERR:
				if(set)
				{
					cas_reg.activityreg |= (1 << ACTIVITY_ERR);
					cas_reg.activityreg &= 0xF0;			//Clear strt - rdy - snow - rain
				}
				else
				{
					cas_reg.activityreg &= ~(1 << ACTIVITY_ERR);
				}
				break;
			case ACTIVITY_ERRLIGHT:
				if(set)
				{
					cas_reg.activityreg |= (1 << ACTIVITY_ERR);
					cas_reg.activityreg |= (1 << ACTIVITY_ERRLIGHT);
					cas_reg.activityreg &= 0xF0;			//Clear strt - rdy - snow - rain
				}
				else
				{
					cas_reg.activityreg &= ~(1 << ACTIVITY_ERRLIGHT);
				}
				break;
			case ACTIVITY_ERRRAIN:
				if(set)
				{
					cas_reg.activityreg |= (1 << ACTIVITY_ERR);
					cas_reg.activityreg |= (1 << ACTIVITY_ERRRAIN);
					cas_reg.activityreg &= 0xF0;			//Clear strt - rdy - snow - rain
				}
				else
				{
					cas_reg.activityreg &= ~(1 << ACTIVITY_ERRRAIN);
				}
				break;
			case ACTIVITY_ERRTEMP:
				if(set)
				{
					cas_reg.activityreg |= (1 << ACTIVITY_ERR);
					cas_reg.activityreg |= (1 << ACTIVITY_ERRTEMP);
					cas_reg.activityreg &= 0xF0;			//Clear strt - rdy - snow - rain
				}
				else
				{
					cas_reg.activityreg &= ~(1 << ACTIVITY_ERRTEMP);
				}
				break;
			case ACTIVITY_DIRT:
				if(set){
					cas_reg.activityreg |= (1 << ACTIVITY_DIRT);
				}
				else cas_reg.activityreg &= ~(1 << ACTIVITY_DIRT);
				break;
			case ACTIVITY_RAIN:
				if((cas_reg.activityreg & (1 << ACTIVITY_RDY)))
				{
					if(set)
					{
						//cas_reg.activityreg &= ~(1 << ACTIVITY_SNOW);
						cas_reg.activityreg |= (1 << ACTIVITY_RAIN);
					}
					else
					{
						cas_reg.activityreg &= ~(1 << ACTIVITY_RAIN);
					}
				}
				else
				{
					if(!set)
					{
						cas_reg.activityreg &= ~(1 << ACTIVITY_RAIN);
					}
				}
				break;
			case ACTIVITY_SNOW:
				if((cas_reg.activityreg & (1 << ACTIVITY_RDY)))
				{
					if(set)
					{	
						//cas_reg.activityreg &= ~(1 << ACTIVITY_RAIN);
						cas_reg.activityreg |= (1 << ACTIVITY_SNOW);
					}
					else
					{
						cas_reg.activityreg &= ~(1 << ACTIVITY_SNOW);
					}
				}
				else
				{
					if(!set)
					{
						cas_reg.activityreg &= ~(1 << ACTIVITY_SNOW);
					}
				}
				break;
			case ACTIVITY_RDY:
				if(set)
				{
					if((cas_reg.activityreg & (1 << ACTIVITY_ERR)) != (1 << ACTIVITY_ERR))
					{
						cas_reg.activityreg |= (1 << ACTIVITY_RDY);
						
					}
				}
				else
				{
					cas_reg.activityreg &= ~(1 << ACTIVITY_RDY);
				}
				break;
			default:
				cas_reg.activityreg |= (1 << ACTIVITY_ERR);	 
				cas_reg.activityreg &= 0xF0;			//Clear strt - rdy - snow - rain
				break;
		}
		
		vendorarray0[B_ACTIVITY] = cas_reg.activityreg;
#ifdef LOG_CASAMBI
		vendorarray1[B_ACTIVITY] = cas_reg.activityreg;
#endif
		
		if(temp != cas_reg.activityreg)
		{
			temp = cas_reg.activityreg;
			if(StdCasambiGetPaired())
			{
				StdCasambiSetSensorValue(CAS_TAG_STATE, 1, AppCasambiGetActivity);
				AppCasambiSetVendorMessage(VM0_OPCODE, VM0_LENGTH);
#ifdef LOG_CASAMBI		
				//AppCasambiSetVendorMessage(VM1_OPCODE, VM1_LENGTH);
#endif
			}
		} 
		return (act);	
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t AppCasambiGetActivity(void)
{
	return cas_reg.activityreg;
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool AppCasambiTest(bool rw, uint8_t* cas_value)
{
	if(rw)				//Read
	{
		*cas_value = cas_reg.testreg;
		return testmode;
	}
	else
	{
		if(*cas_value != 20)				//Idle value
		{
			testmode = true;
			cas_reg.testreg = *cas_value;
		}
		else
		{
			testmode = false;
			cas_reg.testreg = 20;		//Idle value
		}
	}
	return testmode;
}

/*--------------------------------------------------------------------------------------------------------------------*/
void AppCasambiDebug(uint8_t index, uint8_t* value, uint8_t length)
{
	uint8_t i = 0;
	
	if(!debugblocking)
	{
		while(i < length)
		{
			vendorarray1[index + i] = *(value + i);
			i++;
		}
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void AppCasambiSendVM1(void)
{
	debugblocking = true;
	vm1_flag = true;
	//StdCasambiSetVendorMessage(VM1_OPCODE, vendorarray1, VM1_LENGTH);	
	debugblocking = false;
}
/**********************************************************************************************************************/


