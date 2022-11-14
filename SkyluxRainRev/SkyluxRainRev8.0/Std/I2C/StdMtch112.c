/**********************************************************************************************************************/
/**
 * @file        I2C/StdMtch112.h
 *
 * @author      Stijn Vermeersch
 * @date        28/06/2018
 *
 * @brief       Module to control the device MTCH112
 *
 * The functions defined below are there to control the device MTCH112
 *
 * \n<hr>
 * Copyright (c) 2018, S-tronics\n
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
//#include <stdio.h>
#include <hal_delay.h>
#include <hal_io.h>
#include <stdint.h>
#include <hal_gpio.h>
#include "utils.h"

#include "..\..\Drv\DrvI2c.h"
#include "StdMtch112.h"
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/
#define ADDRESS		0x73


//I2C Register
#define MTCH112_OUTCON      0x01    //OUTCON-register
#define MTCH112_CALCON0     0x02    
#define MTCH112_CALCON1     0x03
#define MTCH112_ADACQ0      0x04
#define MTCH112_ADACQ1      0x05
#define MTCH112_LPCON		0x06
#define MTCH112_PR_TRESH    0x07
#define MTCH112_PX_TRESH    0x08
#define MTCH112_TIMEOUT_L   0x09
#define MTCH112_TIMEOUT_H   0x0A
#define MTCH112_STATE       0x80
#define MTCH112_READING0L   0x81
#define MTCH112_READING0H   0x82
#define MTCH112_READING1L   0x83
#define MTCH112_READING1H   0x84
#define MTCH112_BASE0L      0x85
#define MTCH112_BASE0H      0x86
#define MTCH112_BASE1L      0x87
#define MTCH112_BASE1H      0x88

//I2C Data
#define OUTCON_DATA         0x01    //Use as touch button

#define NRM_TCH_WFM         0x00
#define DBL_TCH_WFM         0x01
#define HLF_TCH_WFM         0x02

#define MTCH112_DelayUs(dly_us)          __delay_us(dly_us)
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
/**
 * 	@brief   Function to write a command to the MTCH112
 *
 *	@param	reg		:	Addres of the Eeprom data
 *	@param	data	:	Data for the Eeprom
 *
 */
void StdMtch112WriteCmd(uint8_t* reg, uint8_t* data);
/**
 * 	@brief   Function to read from MTCH112
 *
 *	@param	reg		: 	Register of the MTCH112
 *  @param	data	:	Data read from that register
 *
 */
void StdMtch112ReadCmd(uint8_t* reg, uint8_t* data);


/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/

//static DRVI2C_DEV_HNDL 	hndl;
static uint8_t 		resetpin;
static uint8_t		mt0pin;

static uint8_t		cmdreg;
static uint8_t		cmdreg16;
static uint8_t 		cmddata;
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/
void StdMtch112WriteCmd(uint8_t* reg, uint8_t* data)
{
	static uint8_t wdata[5];
	static uint8_t checksum;
	
	//DrvI2CBeginTransmission(hndl,I2C_W);		//Startbit + write address
	wdata[0] = 0x55;
	wdata[1] = 0xAA;
	wdata[2] = *reg;
	wdata[3] = *data;
	checksum = 0x55;
	checksum ^= 0xAA;
	checksum ^= *reg;
	checksum ^= *data;
	wdata[4] = checksum;
	DrvI2C_0_write(wdata, 5);
	//DrvI2CStop(hndl);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdMtch112ReadCmd(uint8_t* reg, uint8_t* data)
{
	static uint8_t wdata[5];
	static uint8_t rdata[5];
	
	wdata[0] = *reg;
	DrvI2C_0_write(wdata, 1);
	DrvI2C_0_read(rdata, 1);
    *data = rdata[0];
}
/*--------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void StdMtch112Init(const uint8_t respin, const uint8_t mt0pin)
{
	 DrvI2C0_SetSlaveAddress(ADDRESS);
	 //Initialise Reset-pin
	 resetpin = respin;
	 gpio_set_pin_direction(respin, GPIO_DIRECTION_OUT);
	 gpio_set_pin_level(respin, true);
	 
	 //Initialise MT0-pin
	 gpio_set_pin_direction(mt0pin, GPIO_DIRECTION_IN);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdMtch112Reset(void)
{
	//Set Reset-pin low
	gpio_set_pin_level(resetpin, false);
	//Set Reset-pin high
    delay_us(400);
	gpio_set_pin_level(resetpin, true);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdMtch112ResetCmd(void)
{
	cmdreg = 0x00;
	cmddata = 0xFF;
	StdMtch112WriteCmd(&cmdreg, &cmddata);

}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdMtch112Config(void)
{
    cmdreg = MTCH112_OUTCON;
    cmddata = 0x00;
    StdMtch112WriteCmd(&cmdreg, &cmddata);
    cmdreg = MTCH112_CALCON1;
    cmddata = 0x00;
    StdMtch112WriteCmd(&cmdreg, &cmddata);
	cmdreg = MTCH112_LPCON;
	cmddata = 0x02;			//Sleep = 2ms typical sleep duration; CLKSEL = 0 ==> 16MHz (Sampleclock) Normally lower power consumption
	StdMtch112WriteCmd(&cmdreg, &cmddata);
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t StdMtch112ReqCal(MTCH_SNS_NR sensor, uint8_t wfm)
{
	uint8_t timeout = 0x00;
    
	switch(sensor)
    {
      case MTCH_SENSOR0:
        cmdreg = MTCH112_CALCON0;
        break;
      case MTCH_SENSOR1:
        cmdreg = MTCH112_CALCON1;
        break;
      default:
        break;
    }
	
	cmddata |= (wfm << 6);		//Set correct CVD Waveform
	cmddata &= 0xFD;			//Set calibration request bit
	StdMtch112WriteCmd(&cmdreg, &cmddata);
    
	while(timeout < 50)
	{
		//Introduce a waittime here
		StdMtch112ReadCmd(&cmdreg, &cmddata);
		if((cmddata & 0x02)!=0x02)
		{
			timeout++;
		}
		else
		{
			return 0x00;
		}
	}
	return 0x01;			//Timeout occured
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdMtch112Tresh(uint8_t* tresh, uint8_t press_prox, uint8_t rw)
{
	if(press_prox == 0)
	{
		cmdreg = MTCH112_PR_TRESH;
	}
	else
	{
		cmdreg = MTCH112_PX_TRESH;
	}
	
	if(rw == 0)			//Read Treshhold
	{
		StdMtch112ReadCmd(&cmdreg, &cmddata); 
	}
	else				//Write Treshhold
	{
		StdMtch112WriteCmd(&cmdreg, tresh); 
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void stdMtch112Reading(MTCH_SNS_NR sensor, uint16_t* reading)
{
	switch(sensor)
    {
		case MTCH_SENSOR0:
			cmdreg      = MTCH112_READING0L;
			cmdreg16    = MTCH112_READING0H;
			break;
		case MTCH_SENSOR1:
			cmdreg      = MTCH112_READING1L;
			cmdreg16 	= MTCH112_READING1H;
			break;
		default: 
			break;
	}	
	
	StdMtch112ReadCmd(&cmdreg, &cmddata);
	*reading = cmddata;
	StdMtch112ReadCmd(&cmdreg16, &cmddata);
	*reading |= ((cmddata & 0x1F) << 8);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void stdMtch112Baseline(MTCH_SNS_NR sensor, uint16_t* baseline)
{
	switch(sensor)
    {
		case MTCH_SENSOR0:
			cmdreg 		= MTCH112_BASE0L;
			cmdreg16 	= MTCH112_BASE0H;
			break;
		case MTCH_SENSOR1:
			cmdreg 		= MTCH112_BASE1L;
			cmdreg16 	= MTCH112_BASE1H;
			break;
		default: 
			break;
	}
	StdMtch112ReadCmd(&cmdreg, &cmddata);
	*baseline = cmddata;
	StdMtch112ReadCmd(&cmdreg16, &cmddata);
	*baseline |= ((cmddata & 0x1F) << 8);
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t stdMtch112State(uint8_t type, uint8_t* active)
{
	cmdreg = MTCH112_STATE;
	StdMtch112ReadCmd(&cmdreg, &cmddata);
	*active = (cmddata >> type) & 0x01;
	return (cmddata >> 4) & 0x07;
}
/*--------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/
