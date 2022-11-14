/* Auto-generated config file hpl_sercom_config.h */
#ifndef HPL_SERCOM_CONFIG_H
#define HPL_SERCOM_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

#include <peripheral_clk_config.h>

#ifndef SERCOM_I2CM_CTRLA_MODE_I2C_MASTER
#define SERCOM_I2CM_CTRLA_MODE_I2C_MASTER (5 << 2)
#endif

#ifndef CONF_SERCOM_0_I2CM_ENABLE
#define CONF_SERCOM_0_I2CM_ENABLE 1
#endif

// <h> Basic

// <o> I2C Bus clock speed (Hz) <1-400000>
// <i> I2C Bus clock (SCL) speed measured in Hz
// <id> i2c_master_baud_rate
#ifndef CONF_SERCOM_0_I2CM_BAUD
#define CONF_SERCOM_0_I2CM_BAUD 100000
#endif

// </h>

// <e> Advanced
// <id> i2c_master_advanced
#ifndef CONF_SERCOM_0_I2CM_ADVANCED_CONFIG
#define CONF_SERCOM_0_I2CM_ADVANCED_CONFIG 0
#endif

// <o> TRise (ns) <0-300>
// <i> Determined by the bus impedance, check electric characteristics in the datasheet
// <i> Standard Fast Mode: typical 215ns, max 300ns
// <id> i2c_master_arch_trise

#ifndef CONF_SERCOM_0_I2CM_TRISE
#define CONF_SERCOM_0_I2CM_TRISE 215
#endif

// <q> SCL Low Time-Out (LOWTOUT)
// <i> Enables SCL low time-out. If SCL is held low for 25ms-35ms, the master will release it's clock hold
// <id> i2c_master_arch_lowtout
#ifndef CONF_SERCOM_0_I2CM_LOWTOUT
#define CONF_SERCOM_0_I2CM_LOWTOUT 0
#endif

// <o> Inactive Time-Out (INACTOUT)
// <0x0=>Disabled
// <0x1=>5-6 SCL cycle time-out(50-60us)
// <0x2=>10-11 SCL cycle time-out(100-110us)
// <0x3=>20-21 SCL cycle time-out(200-210us)
// <i> Defines if inactivity time-out should be enabled, and how long the time-out should be
// <id> i2c_master_arch_inactout
#ifndef CONF_SERCOM_0_I2CM_INACTOUT
#define CONF_SERCOM_0_I2CM_INACTOUT 0x0
#endif

// <o> SDA Hold Time (SDAHOLD)
// <0=>Disabled
// <1=>50-100ns hold time
// <2=>300-600ns hold time
// <3=>400-800ns hold time
// <i> Defines the SDA hold time with respect to the negative edge of SCL
// <id> i2c_master_arch_sdahold
#ifndef CONF_SERCOM_0_I2CM_SDAHOLD
#define CONF_SERCOM_0_I2CM_SDAHOLD 0x2
#endif

// <q> Run in stand-by
// <i> Determine if the module shall run in standby sleep mode
// <id> i2c_master_arch_runstdby
#ifndef CONF_SERCOM_0_I2CM_RUNSTDBY
#define CONF_SERCOM_0_I2CM_RUNSTDBY 0
#endif

// <o> Debug Stop Mode
// <i> Behavior of the baud-rate generator when CPU is halted by external debugger.
// <0=>Keep running
// <1=>Halt
// <id> i2c_master_arch_dbgstop
#ifndef CONF_SERCOM_0_I2CM_DEBUG_STOP_MODE
#define CONF_SERCOM_0_I2CM_DEBUG_STOP_MODE 0
#endif

// </e>

#if CONF_SERCOM_0_I2CM_TRISE < 215 || CONF_SERCOM_0_I2CM_TRISE > 300
#warning Bad I2C Rise time for Standard/Fast mode, reset to 215ns
#undef CONF_SERCOM_0_I2CM_TRISE
#define CONF_SERCOM_0_I2CM_TRISE 215U
#endif

//                  gclk_freq - (i2c_scl_freq * 10) - (gclk_freq * i2c_scl_freq * Trise)
// BAUD + BAUDLOW = --------------------------------------------------------------------
//                  i2c_scl_freq
// BAUD:    register value low  [7:0]
// BAUDLOW: register value high [15:8], only used for odd BAUD + BAUDLOW
#define CONF_SERCOM_0_I2CM_BAUD_BAUDLOW                                                                                \
	(((CONF_GCLK_SERCOM0_CORE_FREQUENCY - (CONF_SERCOM_0_I2CM_BAUD * 10U)                                              \
	   - (CONF_SERCOM_0_I2CM_TRISE * (CONF_SERCOM_0_I2CM_BAUD / 100U) * (CONF_GCLK_SERCOM0_CORE_FREQUENCY / 1000U)     \
	      / 10000U))                                                                                                   \
	      * 10U                                                                                                        \
	  + 5U)                                                                                                            \
	 / (CONF_SERCOM_0_I2CM_BAUD * 10U))
#ifndef CONF_SERCOM_0_I2CM_BAUD_RATE
#if (CONF_SERCOM_0_I2CM_BAUD_BAUDLOW > (0xFF * 2) || CONF_SERCOM_0_I2CM_BAUD_BAUDLOW <= 1)
#warning Failed to generate requested I2C baudrate, please check your settings
#define CONF_SERCOM_0_I2CM_BAUD_RATE 0xFF
#else
#define CONF_SERCOM_0_I2CM_BAUD_RATE                                                                                   \
	((CONF_SERCOM_0_I2CM_BAUD_BAUDLOW & 0x1)                                                                           \
	     ? (CONF_SERCOM_0_I2CM_BAUD_BAUDLOW / 2) + ((CONF_SERCOM_0_I2CM_BAUD_BAUDLOW / 2 + 1) << 8)                    \
	     : (CONF_SERCOM_0_I2CM_BAUD_BAUDLOW / 2))
#endif
#endif

#include <peripheral_clk_config.h>

#ifndef SERCOM_I2CM_CTRLA_MODE_I2C_SLAVE
#define SERCOM_I2CM_CTRLA_MODE_I2C_SLAVE (4 << 2)
#endif

#ifndef CONF_SERCOM_1_I2CS_ENABLE
#define CONF_SERCOM_1_I2CS_ENABLE 1
#endif

// <h> Basic Configuration

// <o> Address <0x0-0x3FF>
// <i> In 7-bit addressing mode the range is 0x00 to 0x7F, in 10-bit addressing mode the range is 0x000 to 0x3FF
// <id> i2c_slave_address
#ifndef CONF_SERCOM_1_I2CS_ADDRESS
#define CONF_SERCOM_1_I2CS_ADDRESS 0x10
#endif

// </h>

// <e> Advanced Configuration
// <id> i2c_slave_advanced
#ifndef CONF_SERCOM_1_I2CS_ADVANCED_CONFIG
#define CONF_SERCOM_1_I2CS_ADVANCED_CONFIG 1
#endif

// <q> Run in stand-by
// <i> Determine if the module shall run in standby sleep mode
// <id> i2c_slave_runstdby
#ifndef CONF_SERCOM_1_I2CS_RUNSTDBY
#define CONF_SERCOM_1_I2CS_RUNSTDBY 0
#endif

// <o> SDA Hold Time (SDAHOLD)
// <0=>Disabled
// <1=>50-100ns hold time
// <2=>300-600ns hold time
// <3=>400-800ns hold time
// <i> Defines the SDA hold time with respect to the negative edge of SCL
// <id> i2c_slave_sdahold
#ifndef CONF_SERCOM_1_I2CS_SDAHOLD
#define CONF_SERCOM_1_I2CS_SDAHOLD 0x2
#endif

// <q> SCL Low Time-Out (LOWTOUT)
// <i> Enables SCL low time-out. If SCL is held low for 25ms-35ms, the master will release it's clock hold
// <id> i2c_slave_lowtout
#ifndef CONF_SERCOM_1_I2CS_LOWTOUT
#define CONF_SERCOM_1_I2CS_LOWTOUT 0
#endif

// <q> General call addressing (GENCEN)
// <i> Enables general call addressing
// <id> i2c_slave_gencen
#ifndef CONF_SERCOM_1_I2CS_GENCEN
#define CONF_SERCOM_1_I2CS_GENCEN 0
#endif

// <o> Address mode (AMODE)
// <0=>Mask
// <1=>Two addresses
// <2=>Range
// <i> Defines the address mode of a slave device
// <id> i2c_slave_amode
#ifndef CONF_SERCOM_1_I2CS_AMODE
#define CONF_SERCOM_1_I2CS_AMODE 0x0
#endif

// <o> Address mask<0x0-0x3FF>
// <i> This mask acts as second address match register, an address mask register or the lower limit of an address range
// <i> When acts as mask, bit value 1 means that the corresponding address bit is ignored
// <id> i2c_slave_address_mask
#ifndef CONF_SERCOM_1_I2CS_ADDRESS_MASK
#define CONF_SERCOM_1_I2CS_ADDRESS_MASK 0x0
#endif
// </e>

#ifndef CONF_SERCOM_1_I2CS_SPEED
#define CONF_SERCOM_1_I2CS_SPEED 0x00 // Speed: Standard/Fast mode
#endif

// <<< end of configuration section >>>

#endif // HPL_SERCOM_CONFIG_H
