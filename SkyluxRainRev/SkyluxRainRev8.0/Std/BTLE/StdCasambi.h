/**********************************************************************************************************************/
/**
 * @file        BTLE/StdCasambi.h
 *
 * @author      Stijn Vermeersch
 * @date        29.05.19
 *
 * @brief       

 * \n<hr>
 * Copyright (c) 2019, S-tronics\n
 * All rights reserved.
 * \n<hr>\n
 */
/**********************************************************************************************************************/

/**********************************************************************************************************************/
#ifndef STD__CASAMBI_H
#define STD__CASAMBI_H


/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   S Y M B O L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/
#define MAX_ARG_LENGHT	17

#define VM0_OPCODE			20
#define VM1_OPCODE			21

#define SLV_ADR				0x10	//I2C-slave address for Casambi Interface
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/

typedef struct
{
	uint8_t length;
	uint8_t opcode;
	uint8_t arguments[MAX_ARG_LENGHT - 1];
}PACKET;

typedef uint32_t (*STDCAS_SETSENSORVAL)(void);
typedef void (*STDCAS_GETSENSORVAL)(uint8_t tag);
typedef uint32_t (*STDCAS_SETLUXVAL)(void);
typedef uint32_t (*STDCAS_GETPARAM)(uint8_t tag, uint8_t length);
typedef void (*STDCAS_SETPARAM)(uint8_t tag, uint32_t data);
typedef void (*STDCAS_SETVEND)(uint8_t opcode, uint8_t *bytearray, uint8_t length);

//update
typedef void (*STDCAS_PREPAREUPDATE)(void);
typedef void (*STDCAS_UPDATEBLOCK)(uint32_t address, uint8_t *buffer, uint16_t size);
typedef void (*STDCAS_REGISTERFIRMWARE)(uint8_t major, uint8_t minor);

//Update process

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
void StdCasambiInit(uint8_t pendingpin, STDCAS_SETPARAM setparamval, STDCAS_GETPARAM getparamval, STDCAS_GETSENSORVAL getsensorval);

/**
 * @brief   Initialise function for firmware update
 *
 *	Initialises all the hooks that should be executed
 *
 *	@param	preparehook		: hook to method that performs all the firmware update operations
 *	@param	updatehook		: hook to method that performs the update part and does the write to flash operations
 *	@param	firmwarehook	: hook to method that sets de firmware version by a write operation to flash
 *
 */
void StdCasambiUpdateInit(STDCAS_PREPAREUPDATE preparehook, STDCAS_UPDATEBLOCK updatehook, STDCAS_REGISTERFIRMWARE firmwarehook);

void StdCasambiSetReg(uint8_t opcode, uint8_t regnr, uint8_t cas_data);

bool StdCasambiGetState(void);

void StdSetFirmwareVersion(uint8_t major, uint8_t minor);

bool StdCasambiGetPaired(void);

void StdCasambiGetPairedFromCasambi(void);

void StdCasambiGetParameter(void);

void StdCasambiSetSensorValue(uint8_t tag, uint8_t bytelength, STDCAS_SETSENSORVAL setsensorval);

void StdCasambiSetParameterValue(uint8_t tag, uint8_t bytelength, uint8_t value);

void StdCasambiSetVendorMessage(uint8_t opcode, uint8_t* bytearray, uint8_t bytelength);

void StdCasambiHandler(void);

void StdCasambiUpdateHandler(void);
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   S T A T I C   I N L I N E   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/

#endif /* STD__CASAMBI_H */