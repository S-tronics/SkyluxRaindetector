/**********************************************************************************************************************/
/**
 * @file        AppIO.h
 *
 * @author      Stijn Vermeersch
 * @date        03.07.2018
 *
 * @brief      
 *
 *
 *
 * \n<hr>\n
 * Copyright (c) 2018, S-tronics\n
 * All rights reserved.
 * \n<hr>\n
 */
/**********************************************************************************************************************/

/**********************************************************************************************************************/



/***********************************************************************************************************************
; I N C L U D E S
;---------------------------------------------------------------------------------------------------------------------*/
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   S Y M B O L   D E F I N I T I O N S   A N D   M A C R O S
;---------------------------------------------------------------------------------------------------------------------*/

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
void AppIoInit(void);

/**
 * @brief   Function to control the relais output of SkyRain
 *
 * This function sets/clears the relais 
 *
 * @param   onoff:	on = 1 --> set Relais; off = 0 --> clear Relais
 *
 */
void AppIORelais(uint8_t onoff);

/**
 * @brief   Function to control the rain detector output of SkyRain
 *
 * This function sets/clears the rain detector output (sets open drain) 
 *
 * @param   onoff:	on = 1 --> set Open Drain; off = 0 --> clear Open Drain
 *
 */
void AppIOOut(uint8_t onoff);

void AppIODetRain(uint8_t onoff);
/**********************************************************************************************************************/



/***********************************************************************************************************************
; E X P O R T E D   S T A T I C   I N L I N E   F U N C T I O N S
;---------------------------------------------------------------------------------------------------------------------*/

/**********************************************************************************************************************/

