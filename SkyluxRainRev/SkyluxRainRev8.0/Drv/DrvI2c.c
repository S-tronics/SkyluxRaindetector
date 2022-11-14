/*
 * DrvI2c.c
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
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>
#include <hal_gpio.h>
#include <hal_i2c_m_sync.h>
#include "..\..\hal\include\hal_i2c_s_async.h"
#include "utils.h"

#include "atmel_start_pins.h"
#include "DrvI2c.h"
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/
#define		I2CRX_QUEUE_LENGTH	256
#define		I2CTX_QUEUE_LENGTH	32

/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   T Y P E D E F S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   F U N C T I O N   P R O T O T Y P E S
;---------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_0_PORT_init(void);
void DrvI2C_0_CLOCK_init(void);
void DrvI2C_1_PORT_init(void);
void DrvI2C_1_CLOCK_init(void);
static void I2C_1_rx_complete(const struct i2c_s_async_descriptor *const descr);
static void I2C_1_tx_complete(const struct i2c_s_async_descriptor *const descr);
/**********************************************************************************************************************/



/***********************************************************************************************************************
; L O C A L   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
static struct i2c_m_sync_desc I2C_0;
static struct io_descriptor *I2C_0_io;
static struct i2c_s_async_descriptor I2C_1;
static struct io_descriptor *I2C_1_io;

static bool tx_complete = true;

static uint8_t	a_I2Crx_data[I2CRX_QUEUE_LENGTH];
static uint8_t	a_I2Ctx_data[I2CTX_QUEUE_LENGTH];
/**********************************************************************************************************************/

/***********************************************************************************************************************
; E X P O R T E D   V A R I A B L E S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/

/***********************************************************************************************************************
; L O C A L   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(PA08,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_OFF);

	gpio_set_pin_function(PA08, PINMUX_PA08C_SERCOM0_PAD0);

	gpio_set_pin_pull_mode(PA09,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_OFF);

	gpio_set_pin_function(PA09, PINMUX_PA09C_SERCOM0_PAD1);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_1_PORT_init(void)
{
	gpio_set_pin_pull_mode(PA16,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_OFF);

	gpio_set_pin_function(PA16, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_pull_mode(PA17,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_OFF);

	gpio_set_pin_function(PA17, PINMUX_PA17C_SERCOM1_PAD1);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
	_gclk_enable_channel(SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC);
	_gclk_enable_channel(SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_1_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM1);
	_gclk_enable_channel(SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC);
	_gclk_enable_channel(SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC);
}
/*--------------------------------------------------------------------------------------------------------------------*/
static void I2C_1_rx_complete(const struct i2c_s_async_descriptor *const descr)
{
	uint8_t c;

	io_read(I2C_1_io, &c, 1);
}
/*--------------------------------------------------------------------------------------------------------------------*/
static void I2C_1_tx_complete(const struct i2c_s_async_descriptor *const descr)
{
	tx_complete = true;
}
/**********************************************************************************************************************/

/***********************************************************************************************************************
; E X P O R T E D   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_0_init(void)
{
	DrvI2C_0_CLOCK_init();
	i2c_m_sync_init(&I2C_0, SERCOM0);
	DrvI2C_0_PORT_init();
	i2c_m_sync_get_io_descriptor(&I2C_0, &I2C_0_io);
	//i2c_s_async_register_callback(&I2C_1, I2C_S_RX_COMPLETE, I2C_1_rx_complete);
	i2c_m_sync_enable(&I2C_0);
	
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvI2C0_SetSlaveAddress(uint8_t address)
{
	i2c_m_sync_set_slaveaddr(&I2C_0, address, I2C_M_SEVEN);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_0_write(uint8_t* buffer, uint8_t length)
{
	io_write(I2C_0_io, buffer, length);
	
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_0_read(uint8_t* buffer, uint8_t length)
{
	io_read(I2C_0_io, buffer, length);
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_1_init(void)
{
	DrvI2C_1_CLOCK_init();
	i2c_s_async_init(&I2C_1, SERCOM1, a_I2Crx_data, I2CRX_QUEUE_LENGTH);
	DrvI2C_1_PORT_init();
	i2c_s_async_get_io_descriptor(&I2C_1, &I2C_1_io);
	i2c_s_async_register_callback(&I2C_1, I2C_S_TX_COMPLETE, I2C_1_tx_complete);
	i2c_s_async_set_addr(&I2C_1, 0x10);				//Here is the slaveaddress seen by the Casambi Device
	i2c_s_async_enable(&I2C_1);
	i2c_s_async_flush_rx_buffer(&I2C_1);
	
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_1_flush_rx_buf(void)
{
	i2c_s_async_flush_rx_buffer(&I2C_1);
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool DrvI2C_1_read(unsigned char* buffer, uint8_t lenght)
{
	if(io_read(I2C_1_io, buffer, lenght))
	{
		return true;
	}
	return false;
}
/*--------------------------------------------------------------------------------------------------------------------*/
bool DrvI2C_1_HS(void)
{
	return tx_complete;
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_1_write(uint8_t* buffer, uint8_t length)
{
	if(tx_complete == true)					//Never perform a write to I2C bus if transmitting is still pending from an earlier message.
	{
		tx_complete = false;
		io_write(I2C_1_io, buffer, length);	
	}
}
/*--------------------------------------------------------------------------------------------------------------------*/
void DrvI2C_1_reinit_timeout(void)
{
	i2c_s_async_abort_tx(&I2C_1);
	i2c_s_async_deinit(&I2C_1);
	DrvI2C_1_init();
	tx_complete = true;
	
}

/*--------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/