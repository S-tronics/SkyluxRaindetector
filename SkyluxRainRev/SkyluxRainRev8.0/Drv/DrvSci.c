/*
 * DrvAdc.c
 *
 * Created: 01/05/2019 09:54:22
 *  Author: Vermeersch Stijn
 */ 
/**********************************************************************************************************************/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; V E R I F Y    C O N F I G U R A T I O N
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/



/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>
#include <hal_adc_sync.h>
#include "utils.h"

#include "atmel_start_pins.h"
#include "DrvSci.h"
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/
#define		RX_QUEUE_LENGTH		128			//Has to be a power of 2
#define		TX_QUEUE_LENGTH		64			//Has to be a power of 2
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
void DrvSciInitRxQueue(void);
void DrvSciInitTxQueue(void);
bool DrvSciFillRxQueue(uint8_t* queuedata);
bool DrvSciFillTxQueue(uint8_t* queuedata);
bool DrvSciReadRxQueue(uint8_t* queuedata);
bool DrvSciReadTxQueue(uint8_t* queuedata);
void DrvSciIntRx(void);
static void tx_cb_transfercompleted(const struct usart_async_descriptor *const io_descr);
/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
static struct usart_async_descriptor USART_0;
static struct io_descriptor *io;

static uint8_t	rxhead = 0;
static uint8_t	rxtail = 0;
static bool		rxfull = false;
static uint8_t	txhead = 0;
static uint8_t	txtail = 0;
static bool		txfull = false;
static uint8_t	a_rx_data[RX_QUEUE_LENGTH];
static uint8_t	a_tx_data[TX_QUEUE_LENGTH];
/**********************************************************************************************************************/

/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void DrvSciInitRxQueue(void)
{
	rxhead = 0;
	rxtail = 0;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvSciInitTxQueue(void)
{
	txhead = 0;
	txtail = 0;
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool DrvSciFillRxQueue(uint8_t* queuedata)
{
	int16_t diff = 0;
	int16_t length = 0;
	
	length = RX_QUEUE_LENGTH;
	
	if(rxfull == true)
	{
		return false;
	}
	diff = rxhead - rxtail;
	if(diff < 0)
	{
		diff += length;
	}
	if(diff >= length)
	{
		return false;
	}
	a_rx_data[rxhead] = *queuedata;
	rxhead++;
	
	if(rxhead == length)
	{
		rxhead = 0;
	}
	if(rxhead == rxtail)
	{
		rxfull = true;
	}
	return true;
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool DrvSciFillTxQueue(uint8_t* queuedata)
{
	int16_t diff = 0;
	int16_t length = 0;
	
	length = TX_QUEUE_LENGTH;
	
	if(txfull == true)
	{
		return false;
	}
	diff = txhead - txtail;
	if(diff < 0)
	{
		diff += length;
	}
	if(diff >= length)
	{
		return false;
	}
	a_tx_data[txhead] = *queuedata;
	txhead++;
	
	if(txhead == length)
	{
		txhead = 0;
	}
	if(txhead == txtail)
	{
		txfull = true;
	}
	return true;
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool DrvSciReadRxQueue(uint8_t* queuedata)
{
	uint16_t length = 0;
	
	length = RX_QUEUE_LENGTH;
	
	if((rxhead == rxtail) && !rxfull)
	{
		return false;
	}
	*queuedata = a_rx_data[rxtail];
	rxtail++;
	if(rxtail == length)
	{
		rxtail = 0;
	}
	return true;
	
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool DrvSciReadTxQueue(uint8_t* queuedata)
{
	uint16_t length = 0;
	
	length = TX_QUEUE_LENGTH;
	
	if((txhead == txtail) && !txfull)
	{
		return false;
	}
	*queuedata = a_tx_data[txtail];
	txtail++;
	if(txtail == length)
	{
		txtail = 0;
	}
	return true;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvSciIntRx(void)
{
	//DrvSciFillRxQueue();
}
/*--------------------------------------------------------------------------------------------------------------------*/
static void tx_cb_transfercompleted(const struct usart_async_descriptor *const io_descr)
{
	
	
}
/*--------------------------------------------------------------------------------------------------------------------*/
static void rx_cb_datareceived(const struct usart_async_descriptor *const io_descr)
{
	
	
}
/**********************************************************************************************************************/

/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void DrvSciInit()
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM2);
	_gclk_enable_channel(SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC);
	
	
	usart_async_set_mode(&USART_0, USART_MODE_ASYNCHRONOUS);				//Asynchronous Communication
	usart_async_set_parity(&USART_0, USART_PARITY_NONE);					//No parity check
	usart_async_set_stopbits(&USART_0, USART_STOP_BITS_ONE);
	usart_async_set_character_size(&USART_0, USART_CHARACTER_SIZE_8BITS);
	usart_async_set_baud_rate(&USART_0, 9600);
	usart_async_init(&USART_0, SERCOM2, a_rx_data, RX_QUEUE_LENGTH, (void*)NULL);
	usart_async_set_data_order(&USART_0, USART_DATA_ORDER_LSB);			//MSB first
	gpio_set_pin_function(PA14, PINMUX_PA14C_SERCOM2_PAD2);
	gpio_set_pin_function(PA15, PINMUX_PA15C_SERCOM2_PAD3);
	
	//usart_async_register_callback(&USART_0, USART_ASYNC_TXC_CB, tx_cb_transfercompleted);
	
	usart_async_get_io_descriptor(&USART_0, &io);
	usart_async_enable(&USART_0);
	
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvSciWrite(unsigned char* buffer, uint8_t length)
{
	uint8_t i = 0;
	for(i = 0; i < length; i++)
	{
		DrvSciFillTxQueue(buffer);
		buffer++;
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvTxHandler(void)
{
	static uint8_t txdata;
	
	if(usart_async_is_tx_empty(&USART_0) == 1)		//Transmitter is empty
	{
		if(DrvSciReadTxQueue(&txdata))
		{
		io_write(io, &txdata, 1);
		}
	}
	
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool DrvSciRead(unsigned char* buffer, uint8_t length)
{
	//usart_async_read(io, buffer, length);
	if(io_read(io, buffer, length) > 0)
	{
		return true;
	}
	
	return false;
}
/**********************************************************************************************************************/