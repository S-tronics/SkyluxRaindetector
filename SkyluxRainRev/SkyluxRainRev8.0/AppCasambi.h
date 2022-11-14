/**********************************************************************************************************************/
/**
 * @file        AppCasambi.h
 *
 * @author      Stijn Vermeersch
 * @date        03.06.2019
 *
 * @brief      
 *
 *
 *
 * \n<hr>\n
 * Copyright (c) 2019, S-tronics\n
 * All rights reserved.
 * \n<hr>\n
 */
/**********************************************************************************************************************/
#ifndef APP__CASAMBI_H
#define APP__CASAMBI_H
/**********************************************************************************************************************/



/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   S Y M B O L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/
#define ACTIVITY_RAIN		0
#define ACTIVITY_SNOW		1
#define ACTIVITY_RDY		2
#define ACTIVITY_DIRT		3
#define ACTIVITY_ERRLIGHT	4
#define ACTIVITY_ERRTEMP	5
#define ACTIVITY_ERRRAIN	6
#define ACTIVITY_ERR		7

/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/


/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief   Initialiser for the casambiInterface
 *
 *
 */
void AppCasambiInit(void);

/**
 * @brief   Function to push some sensorvalues periodically. Vendormessages are sent also with this function
 *
 */
void AppCasambiPush(const struct timer_task *const timer_task);

/**
 * @brief   Function to handle Casambi I2C-messages that were enabled by interrupt.
 *
 */
void AppCasambiHandler(void);

/**
 * @brief   Function to set a vendormessage to the server (Server -> Client)
 *
 * @param	opcode:		vendormessage opcode (VM0, VM1)
 * @param	length:		the byte length of the internal array. There is only one array, because for the moment only
						one vendormessage is used.
 */
void AppCasambiSetVendorMessage(uint8_t opcode, uint8_t length);


/**
 * @brief   Function to get a vendormessage from server. (Client -> Server)
 *
 * @param	opcode:		vendormessage opcode (VM0, VM1)
 * @param	bytearray:	pointer to an array that contains the bytes of the vendormessage. This array is kept
 *						memory for future use.
 * @param	length:		the byte length of the array
 */
void AppCasambiGetVendorMessage(const uint8_t opcode, const uint8_t* bytearray, const uint8_t length);

/**
 * @brief   R/W RainsensorState
 *
 *	Read - Set RainsensorState
 *
 *	@param	rw:			true if read; false if write
 *	@param	cas_state:	bit to write to the casambi-interface
 *
 *	@return				rainsensorstate
 */
uint8_t AppCasambiRainsensorState(bool rw, uint8_t cas_state);

/**
 * @brief   R/W RainsensorTresholdValueUp
 *
 *	Read - Set Rainsensor Upper Treshold
 *
 *	@param	rw:			true if read; false if write
 *	@param	cas_value:	value to write to the casambi-interface
 *
 *	@return				tresholdvalue
 */
uint8_t AppCasambiRainsensorTresholdValueUp(bool rw, uint8_t cas_value);

/**
 * @brief   R/W RainsensorTresholdValueDown
 *
 *	Read - Set Rainsensor Lower Treshold
 *
 *	@param	rw:			true if read; false if write
 *	@param	cas_value:	value to write to the casambi-interface
 *
 *	@return				tresholdvalue
 */
uint8_t AppCasambiRainsensorTresholdValueDown(bool rw, uint8_t cas_value);

/**
 * @brief   R/W AppCasambiSensorID
 *
 *	Read - Set Rainsensor ID
 *
 *	@param	rw:			true if read; false if write
 *	@param	cas_value:	value to write to the casambi-interface
 *
 *	@return				tresholdvalue
 */
uint8_t AppCasambiSensorID(bool rw, uint8_t cas_value);
/**
 * @brief   R/W Rainsensor Idle Time
 *
 *	When the rainsensor is returning towards idle, this function decides
 *  when the rainsensor returns to its external idle state.
 *
 *	@param	rw:			true if read; false if write
 *	@param	idletime:	value to define the idletime (minutes)
 *
 *	@return				idletime
 */
uint8_t AppCasambiRainsensorIdleTimeValue(bool rw, uint8_t idletime);

/**
 *	@brief   R/W TemperatureValue
 *
 *	Read - Set Temperature to casambi-interface
 *
 *	@param	rw:			true if read; false if write
 *	@param	cas_value:	value to write to the casambi-interface
 *
 *	@return				temperaturevalue
 */
uint8_t AppCasambiTemperatureValue(bool rw, uint8_t cas_value);

/**
 *	@brief   R/W LightValue
 *
 *	Read - Set Light to casambi-interface
 *
 *	@param	rw:			true if read; false if write
 *	@param	cas_value:	value to write to the casambi-interface
 *
 *	@return				lightvalue
 */
uint16_t AppCasambiLightValue(bool rw, uint16_t cas_value);

/**
 * @brief   R/W Activity Register
 *
 *	Read - Set Activity Register
 *
 *	@param	rw:			true if read; false if write
 *	@param	cas_state:	bit to write to the casambi-interface
 *	@param	set:		value of the bit to write
 *
 *	@return				Activity bit.
 */
uint8_t AppCasambiActivity(bool rw, uint8_t act, bool set);

/**
 * @brief   Read Activity Register
 *
 *	Read - Set Activity Register without Casambi Communication
 *
 *	@return				Activity bits.
 */
uint8_t AppCasambiGetActivity(void);

/**
 * @brief   R/W Activity Register
 *
 *	Read - Set Activity Register
 *
 *  @param	rw			: Read = true;  Write = false
 *	@param	cas_value	: Value to overrule the rainstate
 *
 *	@return				Testmode.
 */
bool AppCasambiTest(bool rw, uint8_t* cas_value);

/**
 * @brief   Module to prepare update
 *
 *	Module to prepare everything arround firmware update arround Casambi
 *
 */
//void AppCasambiPrepareUpdate(void);

/**
 * @brief   Module to perform a blockupdate in flash
 *
 *	This method is used to store a block of data received by Casambi
 *
 *  @param	address			: Start address where updateblock should be written
 *	@param	*buffer			: pointer to buffer where data of the updateblock is stored
 *	@param	size			: Het aantal databytes stored in the buffer.
 *
 */
//void AppCasambiUpdate(uint32_t address, uint8_t *buffer, uint16_t size);

/**
 * @brief   Module to perform a blockupdate in flash
 *
 *	This method is used to store a block of data received by Casambi
 *
 *  @param	major			: Major type of firmware revision
 *	@param	minor			: Minor type of firmware revision
 *
 */
//void AppCasambiRegisterFirmware(uint8_t major, uint8_t minor);

/**
 * @brief   Module for debugging on Casambi Communication
 *
 *	This method is used to write some debug info over the Casambi Protocol
 *
 *  @param	index			: index of data stored in Vendormessage
 *	@param	value			: value stored @index in Vendormessage
 *	@param	length			: # of bytes stored in Vendormessage
 *
 */
void AppCasambiDebug(uint8_t index, uint8_t* value, uint8_t length);

void AppCasambiSendVM1(void);
/***********************************************************************************************************************
; E X P O R T E D   S T A T I C   I N L I N E   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/

#endif /* APP__ACTIVITY_H */