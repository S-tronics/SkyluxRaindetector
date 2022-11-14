/*
 * File:   AppCasmabiUpdate.c
 * Author: stronics
 *
 * Created on 28 september 2021, 13:11
 */

/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "..\..\Drv\DrvFlash.h"
#include "..\..\Drv\DrvWdt.h"
#include "..\..\Std\Timer\StdTask.h"
#include "..\..\Std\BTLE\StdCasambi.h"
#include "atmel_start_pins.h"
#include "AppCasambi.h"
#include "AppCasambiUpdate.h"
#include "AppSensor.h"

/**********************************************************************************************************************/

/***********************************************************************************************************************
; V E R I F Y    C O N F I G U R A T I O N
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/
#define		ADDR_CTRL			0x2800
#define		ADDR_APP1			0x3000
#define		ADDR_APP2			0x10000

#define		ADDR_APP1_FW_MAJ	0x2801
#define		ADDR_APP1_FW_MIN	0x2802
#define		ADDR_APP2_FW_MAJ	0x2803
#define		ADDR_APP2_FW_MIN	0x2804

/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief   Initialiser for Casambi Updates
 *
 *
 */

/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
static uint32_t		addr = 0x0000000;
static uint32_t		addr2 = 0x0000000;
static uint8_t		addrdata = 0x00;
static uint32_t		firstupdateaddr = 0x00000000;
static bool			updating = false;

/**********************************************************************************************************************/

/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void AppCasambiUpdateInit(void)
{
	StdCasambiUpdateInit(AppCasambiPrepareUpdate, AppCasambiUpdate, AppCasambiRegisterFirmware);
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool AppCasambiUpdateState(void)
{
	return updating;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void AppCasambiPrepareUpdate(void)
{
	addr = ADDR_CTRL;
	
	updating = true;				//Disable all background handlers
	DrvWdtDeInit();					//Stop Watchdogtimer
	StdTaskStop(trend_task);
	
	DrvFlashRead(addr, &addrdata, 1);
	
	if((addrdata & 0x55) == 0x55)				//App. 1 is running
	{
		firstupdateaddr = ADDR_APP2;			//Data should be stored in memory reserved for App2
	}
	else if((addrdata & 0xAA) == 0xAA)			//App. 2 is running
	{
		firstupdateaddr = ADDR_APP1;			//Data should be stored in memory reserved for App1
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void AppCasambiUpdate(uint32_t address, uint8_t *buffer, uint16_t size)
{
	DrvFlashWrite(firstupdateaddr + address, buffer, size);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void AppCasambiRegisterFirmware(uint8_t major, uint8_t minor)
{
	addr = ADDR_CTRL;
	
	DrvFlashRead(addr, &addrdata, 1);
	
	if((addrdata & 0x55) == 0x55)				//App. 1 is running; App. 2 is updated and should be started in bootloader
	{
		addr = ADDR_APP2_FW_MAJ;
		DrvFlashWrite(addr, &major, 1);
		addr = ADDR_APP2_FW_MIN;
		DrvFlashWrite(addr, &minor, 1);
		addr = ADDR_CTRL;
		addrdata = 0xAA;						//App. 2 should start in bootloader.
		DrvFlashWrite(addr, &addrdata, 1);
	}
	else if((addrdata & 0xAA) == 0xAA)			//App. 2 is running; App. 1 is updated and should be started in bootloader
	{
		addr = ADDR_APP1_FW_MAJ;
		DrvFlashWrite(addr, &major, 1);
		addr = ADDR_APP1_FW_MIN;
		DrvFlashWrite(addr, &minor, 1);
		addr = ADDR_CTRL;
		addrdata = 0x55;						//App. 1 should start in bootloader.
		DrvFlashWrite(addr, &addrdata, 1);
	}
	StdSetFirmwareVersion(major, minor);
	
	/* Jump to bootloader */
	//addr = 0x00000000;								//General reset vector
	//addr2 = *(uint32_t*)(addr + 4);
	///* Re-base the Stack Pointer */ 
	//__set_MSP(*(uint32_t *)addr);
	///* Re-base the vector table addres */
	//SCB->VTOR = ((uint32_t)addr & SCB_VTOR_TBLOFF_Msk);
	///* Jump to application Reset Handler in the application */
	//asm("bx %0"::"r"(addr2));
	NVIC_SystemReset();
	
}
/**********************************************************************************************************************/


