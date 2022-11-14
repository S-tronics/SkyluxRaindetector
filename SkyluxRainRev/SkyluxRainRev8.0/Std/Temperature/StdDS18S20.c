/**********************************************************************************************************************/
/**
 * @file        Temperature/DS18S20.h
 *
 * @author      Stijn Vermeersch
 * @date        03/07/2018
 *
 * @brief       Module to control the temperature sensor DS18S20Z
 *
 * The functions defined below are there to control the device DS18S20Z
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
//Include System Function Libraries
#include <hal_delay.h>
#include <stdint.h>
#include <hal_gpio.h>
#include <hri_systick_d20.h>
//Include Driver Function Libraries
//Include Standard Function Libraries
#include "..\..\Std\Timer\StdDelay.h"

//Include Application Function Libraries
#include "StdDS18S20.h"
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/
#define		READROM				0x33
#define		SKIPROM				0xCC
#define		CONVERT_T			0x44
#define		CPY_SCRATCH			0x48
#define		WRT_SCRATCH			0x4E
#define		REA_POWER			0xB4
#define		RECALL_EE			0xB8
#define		READSCRATCH			0xBE
#define		ALARM_SEARCH		0xEC	

 
#define     TRST				480
#define     TPRESD				60              //Presence delay
#define     MSTRBITSTRT			2
#define     MSTRBITWDELAY		80
#define     MSTRRECOV			2
#define     MSTRBITRDELAY		5

#define     REGTEMPLSB			0
#define     REGTEMPMSB			1
#define     SCRATCHLENGTH		9


/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/
typedef enum {
	DS18_IDLE = 0,
	DS18_CONVERT = 1,
	DS18_PIN = 2,
	DS18_AVAILABLE = 3
}
DS18_STATE;
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
void StdDS18S20WriteBit(uint8_t wrdata);
void StdDS18S20WriteByte(uint8_t wrdata);
bool StdDS18S20ReadBit(void);
uint8_t StdDS18S20ReadByte(void);
void StdDS18S20HighZ(void);
void StdDS18S20Reset(void);
void StdDS18S20SkipRom(void);
void StdDS18S20ConvertT(void);
void StdDS18S20AlarmSearch(void);
void StdDS18S20WriteScratch(void);
void StdDS18S20ReadScratch(void);

void DisableInt(void);
void EnableInt(void);
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
static uint8_t	snsr_present = 0;
static uint8_t	bit_shift = 0;
static uint16_t	shifteddata = 0;

static int			regtemph = 120;			//High Temperature  Limit 120°C
static int			regtempl = -20;			//Low Temperature Limit -20°C
static DS18_STATE	state = DS18_IDLE;
static int			temperature = 0;

static uint8_t drvsenspin;

static uint8_t   ascratchpath[SCRATCHLENGTH];
static uint32_t temp_irq_reg;
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/

void StdDS18S20WriteBit(uint8_t wrdata)
{
	
	gpio_set_pin_direction(drvsenspin, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(drvsenspin, false);
	
	asm("nop");
	asm("nop");
	
    if(wrdata != 0x00)
    {
		gpio_set_pin_level(drvsenspin, true);
    }
	delay_us(MSTRBITWDELAY);
	gpio_set_pin_direction(drvsenspin, GPIO_DIRECTION_IN);
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdDS18S20WriteByte(uint8_t wrdata)
{
    uint8_t i = 0;
    uint8_t temp = 0;
	DisableInt();
    CRITICAL_SECTION_ENTER();
    for(i=0; i < 8; i++)
    {
        temp = wrdata >> i;
        temp &= 0x01;
        StdDS18S20WriteBit(temp);
    }
	CRITICAL_SECTION_LEAVE();
	EnableInt();
	delay_us(40);
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool StdDS18S20ReadBit(void)
{
    bool bitflag = false;
    
	gpio_set_pin_direction(drvsenspin, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(drvsenspin, false);

    asm("nop");
	asm("nop");
    StdDS18S20HighZ();
    asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	bitflag = gpio_get_pin_level(drvsenspin);
    delay_us(40);
	
    return bitflag;
}
/*--------------------------------------------------------------------------------------------------------------------*/
uint8_t StdDS18S20ReadByte(void)
{
    uint8_t i = 0;
    uint8_t value = 0;
	DisableInt();
    CRITICAL_SECTION_ENTER();
    for(i = 0; i < 8; i++)
    {
        if(StdDS18S20ReadBit())
        {
            value |= 1<<i;
        }
    }
	CRITICAL_SECTION_LEAVE();
	EnableInt();
    return value;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdDS18S20HighZ(void)
{
	gpio_set_pin_direction(drvsenspin, GPIO_DIRECTION_IN);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdDS18S20Reset(void)
{
	DisableInt();
    gpio_set_pin_direction(drvsenspin, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(drvsenspin, false);
    delay_us(TRST);
	gpio_set_pin_direction(drvsenspin, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(drvsenspin, true);
    StdDS18S20HighZ();
    delay_us(TPRESD);
    StdDS18S20HighZ();

	delay_us(480);
    EnableInt();
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdDS18S20SkipRom(void)
{
    StdDS18S20WriteByte(SKIPROM);
	
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdDS18S20ConvertT(void)
{
	StdDS18S20Reset();
    //StdDS18S20Reset();
	StdDS18S20SkipRom();
    StdDS18S20WriteByte(CONVERT_T);
    //while(!StdDS18S20ReadBit())
    //{
        ////Timeout toevoegen van > 750ms
    //}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdDS18S20AlarmSearch(void)
{
	
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdDS18S20ReadScratch(void)
{
	uint8_t i = 0;
	
	StdDS18S20Reset();
    StdDS18S20SkipRom();
    StdDS18S20WriteByte(READSCRATCH);
    //BYTE0: Temperature LSB
    //BYTE1: Temperature MSB
    //BYTE2: Th register or user byte 1
    //BYTE3: Tl register or user byte 2
    //BYTE4: Reserved
    //BYTE5: Reserved
    //BYTE6: Count Remain
    //BYTE7: Count per �C
    //BYTE8: CRC
    for(i = 0; i < SCRATCHLENGTH; i++)
    {
        ascratchpath[i] =  StdDS18S20ReadByte();
    }
    i = 0;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DisableInt(void)
{
	uint8_t i = 0;
	for(i = 0; i < 26; i++)
	{
		if(NVIC_GetEnableIRQ((IRQn_Type) i) == 1)
		{
			NVIC_DisableIRQ((IRQn_Type) i);
			temp_irq_reg |= 1 << i;
		}
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void EnableInt(void)
{
	uint8_t i = 0;
	for(i = 0; i < 26; i++)
	{
		if(((temp_irq_reg >> i) & 0x01) == 0x01)
		{
			NVIC_EnableIRQ((IRQn_Type) i);
		}
	}
	temp_irq_reg = 0;
}

/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void StdDS18S20Init(uint8_t senspin)
{
	uint8_t i = 0;
    drvsenspin = senspin;
	for(i=0; i < SCRATCHLENGTH; i++)
	{
		ascratchpath[i] = 0;
	}
    //Output pin of temperature sensor
    StdDS18S20HighZ();
    StdDS18S20ConvertT();
    StdDS18S20ReadScratch();
	state = DS18_CONVERT;
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool StdDS18S20Handler(void)
{	
	bool temp_ok = false;
	switch(state)
	{
		case DS18_IDLE:
			temp_ok = false;
			break;
		case DS18_CONVERT:
			StdDS18S20ConvertT();
			state = DS18_PIN;
			temp_ok = false;
			break;
		case DS18_PIN:
			if(StdDS18S20ReadBit()) state = DS18_AVAILABLE;
			temp_ok = false;
			break;
		case DS18_AVAILABLE:
			StdDS18S20ReadScratch();
			state = DS18_CONVERT;
			temp_ok = true;
			break;
		default:
			temp_ok = false;
			state = DS18_CONVERT;
			break;
	}
	return temp_ok;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void StdDS18S20SetAlarm(int temph, int templ)
{
	regtemph = temph;
	regtempl = templ;
}
/*--------------------------------------------------------------------------------------------------------------------*/
int16_t StdDS18S20GetTemperature(void)
{
    //StdDS18S20ReadScratch(); 
    return ((ascratchpath[REGTEMPMSB] << 8) | (ascratchpath[REGTEMPLSB]));
}
/*--------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/
