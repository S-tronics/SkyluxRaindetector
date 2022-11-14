/**********************************************************************************************************************/
/**
 * @file        BTLE/StdCasambi.h
 *
 * @author      Stijn Vermeersch
 * @date        29/05/19
 *
 * @brief       
 *
 *
 * \n<hr>
 * Copyright (c) 2019, S-tronics\n
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

/**********************************************************************************************************************/



/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
//Include System Function Libraries
#include <stdint.h>
#include <string.h>
#include <hal_delay.h>
#include <hal_gpio.h>
//Include Driver Function Libraries
#include "..\..\Drv\DrvI2c.h"
//Include Standard Function Libraries
#include "..\..\Std\CRC\StdCRC.h"
#include "..\..\Std\Timer\StdTask.h"
//Include Application Function Libraries
#include "StdCasambi.h"
#include "atmel_start_pins.h"
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/

#define COM_INT				10		//µs
#define PENDING_INT			8


#define SETPAIRING			54		//S->C & C->S
#define SETPARING_LENGTH	2
#define PAIRING_ID_INDEX	0
#define SETFWVER			28		//C->S
#define SETFWVER_LENGTH		3
#define INIT_OPCODE			3		//S->C
#define INIT_LENGTH			3
#define GET_PAIRED			1		//C->S
#define GET_SENS_VAL		24		//S->C
#define GET_SENS_VAL_L		2
#define SET_SENS_VAL		25		//C->S
#define SET_SENS_VAL_L		2
#define SET_PARAM_VAL		26		//S->C & C->S
#define SET_PARAM_VAL_L		2
#define GET_PARAM_VAL		29		//S->C
#define GET_PARAM_VAL_L		2
#define PARAM_COMPLETE		28		//S->C
#define PARAM_COMPLETE_L	1
#define PARAM_COMPL			27
#define PARAM_COMPL_L		1
#define SET_LUX_SENS		44
#define SET_LUX_SENS_L		3

#define VM0_LENGTH			1
#define VM1_LENGTH			1

//update
#define FW_UPDATE_INIT			34
#define FW_UPDATE_INIT_LENGTH	6
#define FW_UPDATE_COMMIT		35
#define FW_UPDATE_COMMIT_LENGTH	6
#define FW_BLOCK_START			36
#define FW_BLOCK_START_LENGTH	14
#define FW_BLOCK_PUSH			37
#define FW_BLOCK_PUSH_LENGTH	14
#define FW_FRAGMENT				38
#define FW_SIGNAL				39
#define FW_SIGNAL_LENGTH		2

#define BLOCK_SIZE				240

/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/
typedef enum
{
	ST_LENGTH,
	ST_OPCODE,
	ST_ARG
}CAS_READ_STATE;


//Different codes to answer on Casambi Firmware update.
typedef enum
{
	CODE_ACK = 0x00,
	CODE_NACK = 0xFF,
	CODE_IO_ERR = 0x55,
	CODE_ABORT = 0xAA
}
CAS_CODE_SIGNAL;

/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
void StdCasambiSetPending(void);
static void StdCasambiTimeoutTask(const struct timer_task *const timer_task);
static void StdCasambiUpdateTask(const struct timer_task *const timer_task);
void StdCasambiWritePacket(void);
void StdCasambiSetPending(void);
void StdCasambiCom(uint8_t cas_data);
void StdCasambiPairing(void);
void StdCasambiGetVendormessage(void);
void StdCasambiLuxValue(void);
void StdCasambiCalcCRC(void);
void StdCasambiFwUpdateInit(void);
void StdCasambiFwUpdateCommit(void);
void StdCasambiFwBlockStart(void);
void StdCasambiFwBlockPush(void);
void StdCasambiFwFragment(void);
void StdCasambiFwSignalCS(CAS_CODE_SIGNAL sign);
void StdCasambiFwSignalSC(void);

/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
static uint8_t		stdcasambimajor;
static uint8_t		stdcasambiminor;
static PACKET		ar_packet;
static PACKET		ar_w_packet;
static bool			err_state = false;
static uint8_t		stdpendingpin;
static TASK_HNDL	casambitask;
static TASK_HNDL	casambiupdatetask;
static uint8_t	pairingid = 0;
static bool		paired = false;		//flag to check if device is paired.

static STDCAS_SETSENSORVAL	setsenshook;
static STDCAS_GETSENSORVAL	getsensorhook;
static STDCAS_SETLUXVAL		setluxhook;
static STDCAS_GETPARAM		getparamhook;
static STDCAS_SETPARAM		setparamhook;
static STDCAS_SETVEND		setvendorhook;	
//update
static STDCAS_PREPAREUPDATE prepareupdatehook;
static STDCAS_UPDATEBLOCK	updateblockhook;
static STDCAS_REGISTERFIRMWARE updatefirmwarehook;

//Updating
static bool updatebusy = false;
static uint8_t updateMajor = 0;
static uint8_t updateMinor = 0;
static uint16_t updatingblocks = 0;			//number of blocks
static uint16_t blockcounter = 0;
static uint32_t blockaddress = 0x3200;
static uint16_t blockindex = 0;
static uint16_t blocksize = 0;				//number of bytes in 1 block
static uint32_t digest = 0;
static uint32_t digestcalc = 0;
static uint8_t	msgcrc = 0;
static uint32_t	msgcrccalc = 0;
static bool update_send_ack = false;

static uint8_t ar_block[BLOCK_SIZE];
static uint16_t fragmentindex = 0;

/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void StdCasambiSetPending(void)					//Toggle Pending pin
{
	delay_us(COM_INT);
	gpio_set_pin_level(stdpendingpin, true);
	delay_us(PENDING_INT);
	gpio_set_pin_level(stdpendingpin, false);
	//delay_us(COM_INT);						//Before 15/09/2022; Test if data is overwritten by Casambi module
}
/*--------------------------------------------------------------------------------------------------------------------*/
static void StdCasambiTimeoutTask(const struct timer_task *const timer_task)
{
	DrvI2C_1_reinit_timeout();
	//StdTaskStop(casambitask);				//Retry
	NVIC_SystemReset();
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiWritePacket(void)
{
	int8_t i = 0;
	if(DrvI2C_1_HS())			//No TX-message pending
	{	
		StdTaskStart(casambitask);
		//DrvI2C_1_write(&ar_w_packet.length, 1);			//Before 15/09/2022; Test if data is overwritten by Casambi module
		
		StdCasambiSetPending();
		DrvI2C_1_write(&ar_w_packet.length, 1);				//Added 15/09/2022; Test if data is overwritten by Casambi module
		
		while(!DrvI2C_1_HS())
		{	}
		//DrvI2C_1_write(&ar_w_packet.opcode, 1);
		StdCasambiSetPending();
		DrvI2C_1_write(&ar_w_packet.opcode, 1);				//Added 15/09/2022; Test if data is overwritten by Casambi module
		
		while(!DrvI2C_1_HS())
		{	}
		if(ar_w_packet.length>1)					//There is an argument to send
		{
			for(i = 0; i < (ar_w_packet.length - 1); i++)
			{
				//DrvI2C_1_write(&ar_w_packet.arguments[i], 1);
				StdCasambiSetPending();
				DrvI2C_1_write(&ar_w_packet.arguments[i], 1);			//Added 15/09/2022; Test if data is overwritten by Casambi module
				while(!DrvI2C_1_HS())
				{	}
			}
		}
		StdTaskStop(casambitask);
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiPairing(void)
{
	if(ar_packet.opcode == SETPAIRING)
	{
		if(ar_packet.arguments[PAIRING_ID_INDEX] == 0)
		{
			paired = false;
		}
		else
		{
			paired = true;
			pairingid = ar_packet.arguments[PAIRING_ID_INDEX];
			ar_w_packet.length = SETFWVER_LENGTH;
			ar_w_packet.opcode = SETFWVER;
			ar_w_packet.arguments[0] = stdcasambimajor;
			ar_w_packet.arguments[1] = stdcasambiminor;
			
			//StdCasambiWritePacket();					//Niet antwoorden op pairing
		}
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiLuxValue(void)
{
	uint16_t luxvalue = 0;
	
	ar_w_packet.length = SET_LUX_SENS_L;
	ar_w_packet.opcode = SET_LUX_SENS;
	luxvalue = setluxhook();
	ar_w_packet.arguments[0] = (uint8_t)(luxvalue & 0x00FF);
	ar_w_packet.arguments[1] = (uint8_t)(luxvalue >> 8);
	StdCasambiWritePacket();
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiSetParameter(void)						//Server to client
{
	uint8_t param = 0;
	
	if(ar_packet.opcode == GET_PARAM_VAL)				//Server asks for a parameter
	{
		ar_w_packet.opcode = SET_PARAM_VAL;
		ar_w_packet.length = 3;
		param = getparamhook(ar_packet.arguments[0], 1);			//Opcode
		ar_w_packet.arguments[0] = ar_packet.arguments[0];			//Parameter Tag
		ar_w_packet.arguments[1] = param;
		StdCasambiWritePacket();
	}
	else if(ar_packet.opcode == SET_PARAM_VAL)
	{
		setparamhook(ar_packet.arguments[0], (ar_packet.arguments[1] | (ar_packet.arguments[2] << 8)));
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiGetVendormessage(void)
{
	if((ar_packet.opcode == VM0_OPCODE) || (ar_packet.opcode == VM1_OPCODE))
	{
		setvendorhook(ar_packet.opcode, ar_packet.arguments, ar_packet.length - 1);
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiCalcCRC(void)
{
	//msgcrccalc = StdCRC32Table(msgcrccalc, &ar_packet.length, 1);
	msgcrccalc = StdCRC32Table(msgcrccalc, &ar_packet.opcode, 1);
	msgcrccalc = StdCRC32Table(msgcrccalc, ar_packet.arguments, ar_packet.length - 1);
	
}
/*--------------------------------------------------------------------------------------------------------------------*/
static void StdCasambiUpdateTask(const struct timer_task *const timer_task)
{
	update_send_ack = true;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiFwUpdateInit(void)						//Start update
{
	if(ar_packet.opcode == FW_UPDATE_INIT)
	{
		//gpio_set_pin_level(PA04, true);
		//StdTaskStopAll();
		updatebusy = true;
		updateMajor = ar_packet.arguments[0];
		updateMinor = ar_packet.arguments[1];
		updatingblocks = 0;
		updatingblocks |= (uint16_t)ar_packet.arguments[3] << 8;
		updatingblocks |= ar_packet.arguments[2];
		blockcounter = 0;
		msgcrc = ar_packet.arguments[4];
		msgcrccalc = 0;
		prepareupdatehook();
		StdCasambiFwSignalCS(CODE_ACK);
		//StdCasambiCalcCRC();
		StdTaskStart(casambiupdatetask);
		//StdSetFirmwareVersion(updateMajor, updateMinor);
		//gpio_set_pin_level(PA04, false);
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiFwUpdateCommit(void)						//End update
{
	//gpio_set_pin_level(PA04, true);
	if(ar_packet.opcode == FW_UPDATE_COMMIT)
	{
		if(ar_packet.arguments[0] == updateMajor)
		{
			if(ar_packet.arguments[1] == updateMinor)
			{
				if((uint16_t)(((uint16_t)ar_packet.arguments[3] << 8) | ar_packet.arguments[2]) == updatingblocks)
				{
					if(blockcounter == updatingblocks)
					{
						StdTaskStop(casambiupdatetask);
						StdCasambiFwSignalCS(CODE_ACK);
						//StdCasambiCalcCRC();
						updatefirmwarehook(updateMajor, updateMinor);			//Write correct firmware to the correct flash location. (specified by the hook)
					}
				}
			}
		} 
		else
		{
			StdCasambiFwSignalCS(CODE_ABORT);
		}
	}
	//gpio_set_pin_level(PA04, false);
	blockcounter = 0;
	msgcrccalc = 0;
	//StdTaskRestartAllStarted();
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiFwBlockStart(void)						//Start block update
{
	if(ar_packet.opcode == FW_BLOCK_START)
	{
		gpio_set_pin_level(PA04, true);
		blockaddress = 0;
		blockaddress |= ((uint32_t)ar_packet.arguments[3] << 24) & 0xFF000000;
		blockaddress |= ((uint32_t)ar_packet.arguments[2] << 16) & 0x00FF0000;
		blockaddress |= ((uint32_t)ar_packet.arguments[1] << 8) & 0x0000FF00;
		blockaddress |= ((uint32_t)ar_packet.arguments[0]) & 0x000000FF;
		blockindex = 0;
		blockindex |= ((uint16_t)ar_packet.arguments[5] << 8) & 0xFF00;
		blockindex |=((uint16_t)ar_packet.arguments[4]) & 0x00FF;
		blocksize = 0;
		blocksize |= ((uint16_t)ar_packet.arguments[7] << 8 ) & 0xFF00;
		blocksize |= ((uint16_t)ar_packet.arguments[6]) & 0x00FF;
		digest = 0;
		digest |= ((uint32_t)ar_packet.arguments[11] << 24) & 0xFF000000;
		digest |= ((uint32_t)ar_packet.arguments[10] << 16) & 0x00FF0000;
		digest |= ((uint32_t)ar_packet.arguments[9] << 8) & 0x0000FF00;
		digest |= ((uint32_t)ar_packet.arguments[8]) & 0x000000FF;
		msgcrc = ar_packet.arguments[12];
		fragmentindex = 0;
		StdCasambiFwSignalCS(CODE_ACK);	
		//StdCasambiCalcCRC();
		//gpio_set_pin_level(PA04, false);
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiFwBlockPush(void)					//push block of memory to the correct flash section
{
	uint32_t tempblockaddress = 0x00000000;
	//gpio_set_pin_level(PA04, true);
	if(ar_packet.opcode == FW_BLOCK_PUSH)
	{
		tempblockaddress |= ((uint32_t)ar_packet.arguments[3] << 24) & 0xFF000000;
		tempblockaddress |= ((uint32_t)ar_packet.arguments[2] << 16) & 0x00FF0000;
		tempblockaddress |= ((uint32_t)ar_packet.arguments[1] << 8) & 0x0000FF00;
		tempblockaddress |= ((uint32_t)ar_packet.arguments[0]) & 0x000000FF; 
		digest = 0;
		digest |= ((uint32_t)ar_packet.arguments[11] << 24) & 0xFF000000;
		digest |= ((uint32_t)ar_packet.arguments[10] << 16) & 0x00FF0000;
		digest |= ((uint32_t)ar_packet.arguments[9] << 8) & 0x0000FF00;
		digest |= ((uint32_t)ar_packet.arguments[8]) & 0x000000FF;
		digestcalc = 0;
		digestcalc = StdCRC32Table(digestcalc, ar_block, blocksize);
		
		msgcrc = ar_packet.arguments[12];
		
		//msgcrccalc = StdCRC32Table(msgcrccalc, ar_block, blocksize);
		
		if((blockaddress != tempblockaddress) || (fragmentindex < blocksize) || (digest != digestcalc))
		{

			StdCasambiFwSignalCS(CODE_NACK);	
		}
		else
		{
			StdCasambiFwSignalCS(CODE_ACK);
			updateblockhook(blockaddress, ar_block, blocksize);
			//StdCasambiCalcCRC();
		}
		blockcounter++;
		fragmentindex = 0;
	}
	gpio_set_pin_level(PA04, false);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiFwFragment(void)
{
	int8_t i = 0;
	if(ar_packet.opcode == FW_FRAGMENT)
	{
		//gpio_set_pin_level(PA04, true);
		for(i = 0; i < (ar_packet.length - 1); i++)
		{
			if((fragmentindex < (uint16_t)BLOCK_SIZE) && (fragmentindex < blocksize))
			{
				ar_block[fragmentindex] = ar_packet.arguments[i];
				fragmentindex++;
				//StdCasambiCalcCRC();
			}
			else 
			{
				StdCasambiFwSignalCS(CODE_ABORT);
			}
		}
		StdCasambiFwSignalCS(CODE_ACK);
		//gpio_set_pin_level(PA04, false);
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiFwSignalCS(CAS_CODE_SIGNAL sign)		//Client to Server
{
	ar_w_packet.opcode = FW_SIGNAL;
	ar_w_packet.length = FW_SIGNAL_LENGTH;
	ar_w_packet.arguments[0] = sign;
	switch(sign)
	{
		case CODE_ACK:
			StdCasambiWritePacket();
			break;
		case CODE_NACK:
			StdCasambiWritePacket();
			NVIC_SystemReset();
			break;
		case CODE_IO_ERR:
			StdCasambiWritePacket();
			NVIC_SystemReset();
			break;
		case CODE_ABORT:
			StdCasambiWritePacket();
			NVIC_SystemReset();
			break;
	}
	
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiFwSignalSC(void)			//Server to Client
{
	if(ar_packet.opcode == FW_SIGNAL)
	{	
		switch((CAS_CODE_SIGNAL)ar_packet.arguments[0])
		{
			case CODE_ACK:
				break;
			case CODE_NACK:
				NVIC_SystemReset();
				break;
			case CODE_IO_ERR:
				break;
			case CODE_ABORT:
				NVIC_SystemReset();
				break;
		}
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiCom(uint8_t cas_data)
{
	static uint8_t	arg_cntr = 1;				//First time when cas_stat = ST_ARG we read already 1 argument
	
	static CAS_READ_STATE cas_state = ST_LENGTH;
	
	switch(cas_state)
	{
		case ST_LENGTH:
			if((cas_data == 0)||(cas_data > 17))
			{
				cas_state = ST_LENGTH;
			}
			else
			{
				ar_packet.length = cas_data;
				cas_state = ST_OPCODE;
			}
			break;
		case ST_OPCODE:
			ar_packet.opcode = cas_data;
			if(ar_packet.length == 1)		cas_state = ST_LENGTH;
			else if(ar_packet.length > 1)	cas_state = ST_ARG;
			switch(ar_packet.opcode)
			{
				case INIT_OPCODE:
				case SETPAIRING:
					break;
				case GET_SENS_VAL:
					break;
				case SET_PARAM_VAL:						//Server (Casambi) or Client wants to set a value in the sensor
					break;
				case GET_PARAM_VAL:						//Server (Casambi) wants to set a parameter to the sensor
					break;
				case PARAM_COMPLETE:
					break;
				case VM0_OPCODE:						//Server (Casambi) wants to send a vendormessage (VM0) to the sensor
					break;
				case VM1_OPCODE:						//Server (Casambi) wants to send a vendormessage (VM1) to the sensor
					break;
				case FW_UPDATE_INIT:
					break;
				case FW_UPDATE_COMMIT:
					break;
				case FW_BLOCK_START:
					break;
				case FW_BLOCK_PUSH:
					break;
				case FW_FRAGMENT:
					break;
				case FW_SIGNAL:
					break;
				default:
					//cas_state = ST_ARG;
					break;
			}			
			break;
		case ST_ARG:
			ar_packet.arguments[arg_cntr-1] = cas_data;
			arg_cntr++;
			if(arg_cntr > (ar_packet.length - 1))
			{
				switch(ar_packet.opcode)
				{	
					case INIT_OPCODE:
					case SETPAIRING:
						StdCasambiPairing();
						break;
					case GET_SENS_VAL:
						getsensorhook(ar_packet.arguments[0]);
						break;
					case GET_PARAM_VAL:
						StdCasambiSetParameter();
						break;
					case SET_PARAM_VAL:
						StdCasambiSetParameter();					
						break;
					case VM0_OPCODE:
						break;
					case VM1_OPCODE:
						break;
					case FW_UPDATE_INIT:
						StdCasambiFwUpdateInit();
						break;
					case FW_UPDATE_COMMIT:
						StdCasambiFwUpdateCommit();
						break;
					case FW_BLOCK_START:
						StdCasambiFwBlockStart();
						break;
					case FW_BLOCK_PUSH:
						StdCasambiFwBlockPush();
						break;
					case FW_FRAGMENT:
						StdCasambiFwFragment();
						break;
					case FW_SIGNAL:
						StdCasambiFwSignalSC();
						break;
					default:
						break;
				}
				cas_state = ST_LENGTH;
				arg_cntr = 1;
			}
			break;
		default:
			err_state = true;
			break;
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void StdCasambiInit(uint8_t pendingpin, STDCAS_SETPARAM setparamval, STDCAS_GETPARAM getparamval, STDCAS_GETSENSORVAL getsensorval)
{
	stdpendingpin = pendingpin;

	gpio_set_pin_direction(stdpendingpin, GPIO_DIRECTION_OUT);				//Initialise output for pending
	
	err_state = false;
	
	setparamhook = setparamval;
	getparamhook = getparamval;
	getsensorhook = getsensorval;
	
	casambitask = StdTaskRegisterTask(1000, StdCasambiTimeoutTask);
	casambiupdatetask = StdTaskRegisterTask(80, StdCasambiUpdateTask);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiUpdateInit(STDCAS_PREPAREUPDATE preparehook, STDCAS_UPDATEBLOCK updatehook, STDCAS_REGISTERFIRMWARE firmwarehook)
{
	prepareupdatehook = preparehook;
	updateblockhook = updatehook;
	updatefirmwarehook = firmwarehook;
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool StdCasambiGetPaired(void)
{
	return paired;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiGetPairedFromCasambi(void)
{
	ar_w_packet.length = GET_PAIRED;
	ar_w_packet.opcode = SETPAIRING;
	
	StdCasambiWritePacket();
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiGetParameter(void)						//Client to Server
{
	ar_w_packet.length = 0x01;
	ar_w_packet.opcode = GET_PARAM_VAL;
	StdCasambiWritePacket();
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool StdCasambiGetState(void)
{
	return err_state;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdSetFirmwareVersion(uint8_t major, uint8_t minor)						//Client to server
{
	ar_w_packet.opcode = SETFWVER;
	ar_w_packet.length = SETFWVER_LENGTH;
	ar_w_packet.arguments[0] = major;
	ar_w_packet.arguments[1] = minor;
	stdcasambimajor = major;
	stdcasambiminor = minor;
	
	StdCasambiWritePacket();
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiSetSensorValue(uint8_t tag, uint8_t bytelength, STDCAS_SETSENSORVAL setsensorval)
{
	uint32_t sensorvalue = 0;
	int8_t i = 0;
	
	setsenshook = setsensorval;
	ar_w_packet.length = SET_SENS_VAL_L + bytelength;
	ar_w_packet.opcode = SET_SENS_VAL;
	ar_w_packet.arguments[0] = tag;
	sensorvalue = setsenshook();
	if((bytelength >= 1)&&(bytelength <=3))
	{
		for(i=0; i<bytelength; i++)
		{
			ar_w_packet.arguments[i+1] = (uint8_t)(sensorvalue >> (i*8));
		}
		StdCasambiWritePacket();
	}
	else
	{
		err_state = true;
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiSetParameterValue(uint8_t tag, uint8_t bytelength, uint8_t value)
{
	ar_w_packet.length = SET_PARAM_VAL_L + bytelength;
	ar_w_packet.opcode = SET_PARAM_VAL;
	ar_w_packet.arguments[0] = tag;
	if((bytelength >= 1)&&(bytelength <=3))
	{
		ar_w_packet.arguments[1] = value;
		StdCasambiWritePacket();
	}
	else
	{
		err_state = true;
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiSetVendorMessage(uint8_t opcode, uint8_t* bytearray, uint8_t bytelength)
{
	ar_w_packet.length = VM1_LENGTH + bytelength;
	switch(opcode)
	{
		case VM0_OPCODE:
			ar_w_packet.length = VM1_LENGTH + bytelength;
			ar_w_packet.opcode = VM0_OPCODE;
			break;
		case VM1_OPCODE:
			ar_w_packet.length = VM1_LENGTH + bytelength;
			ar_w_packet.opcode = VM1_OPCODE;
			break;
		default:
			break;	
	}
	if(bytelength <= 8)
	{
		memcpy(ar_w_packet.arguments, bytearray, bytelength);
	}
	StdCasambiWritePacket();
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiHandler(void)
{
	static uint8_t data = 0;
	
	if(DrvI2C_1_read(&data, 1))
	{
		StdCasambiCom(data);
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdCasambiUpdateHandler(void)
{
	if(update_send_ack == true)
	{
		//gpio_set_pin_level(PA04, true);
		//StdCasambiFwSignalCS(CODE_ACK);
		update_send_ack = false;
		//gpio_set_pin_level(PA04, false);
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/
