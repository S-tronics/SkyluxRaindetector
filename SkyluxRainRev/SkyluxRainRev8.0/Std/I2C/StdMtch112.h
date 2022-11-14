/**********************************************************************************************************************/
/**
 * @file        I2C/StdMtch112.h
 *
 * @author      Stijn Vermeersch
 * @date        28.06.2018
 *
 * @brief       Module for handling MTCH112 
 *
 *
 *
 *
 * \n<hr>
 * Copyright (c) 2018, S-tronics\n
 * All rights reserved.
 * \n<hr>\n
 */
/**********************************************************************************************************************/

/**********************************************************************************************************************/
#ifndef STD__MTCH112_H
#define STD__MTCH112_H

/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
#include "utils.h"
//#include "..\..\Driver\PIC18F2X-4XK22\DrvI2C.h"
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
	MTCH_SENSOR0 = 0,
	MTCH_SENSOR1 = 1
}
MTCH_SNS_NR;
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
 *	Initialising this module
 *
 */
//void StdMtch112Init(I2C_CHANNEL ch, GPIO_PORT resport, uint8_t respin, GPIO_PORT mt0port, uint8_t mt0pin);
void StdMtch112Init(const uint8_t respin, const uint8_t mt0pin);
/**
 * @brief   Function to reset the MTCH112
 *
 * Resets the sensor.
 *
 */
void StdMtch112Reset(void);
/**
 * @brief   Function to reset the MTCH112
 *
 * Resets the sensor and its registers
 *
 */
void StdMtch112ResetCmd(void);
/**
 * @brief   Function to configure the MTCH112
 *
 * Configures the sensor.
 *
 */
void StdMtch112Config(void);

/**
 * 	@brief   Function to request for calibration of the MTCH112
 *
 *	@param	sensor		: 	The sensor to calibrate inside the MTCH112
 *  @param	wfm			:	The OCD Waveform
 *
 *	@return				: 	0x00: No Timeout; 0x01: Timeout occured
 *
 */
uint8_t StdMtch112ReqCal(MTCH_SNS_NR sensor, uint8_t wfm);

/**
 *	@brief			Function to R/W Acquisition delay
 *  @param tresh		:	--> level of the treshold register
 *  @param press_prox	: 	--> pressprox= 0: controls the pressure treshold register; pressprox= 1: controls the proximity treshold register
 *  @param rw			:	--> r= 0: read acquisition delay; w= 1: write acquisition delay
 *
 */
void StdMtch112Tresh(uint8_t* tresh, uint8_t press_prox, uint8_t rw);

/**
 *	@brief	:	Function to R/W readed value
 *
 *  @param	:	sensor 		--> which sensor should be read out
 *  @param	:	reading     --> level of the reading register
 *
 */
void stdMtch112Reading(MTCH_SNS_NR sensor, uint16_t* reading);
/**
 *	@brief	:	Function to R/W baseline value
 *
 *  @param	:	sensor 		--> which sensor should be read out
 *  @param	:	baseline     --> level of the baseline register
 *
 */
void stdMtch112Baseline(MTCH_SNS_NR sensor, uint16_t* basevalue);
/**
 *	@brief	:	Function check the state of the MTCH112
 *
 *  @param	:	type 		--> which sensor should be read out
 *  @param	:	active      --> level of the reading register
 *
 *	@return	: 	0x00: No Timeout; 0x01: Timeout occured
 *
 */
uint8_t stdMtch112State(uint8_t type, uint8_t* active);
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   S T A T I C   I N L I N E   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/

#endif /* STD__MTCH112_H */