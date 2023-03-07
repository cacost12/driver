/*******************************************************************************
*
* FILE:
* 		valve.c
*
* DESCRIPTION:
* 		Servo valve actuation API	
*
*******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes                                                              
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "sdr_pin_defines_L0005.h"
#include "stm32h7xx_hal.h"
#include "main.h"
#include "valve.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
volatile static int32_t  lox_valve_pos       = 0;  /* LOX Valve Encoder count  */
volatile static bool     lox_channelA_state  = ENCODER_LOW; /* Voltage on channel 
                                                             A pin */
volatile static bool     lox_channelB_state  = ENCODER_LOW; /* Voltage on Channel 
                                                             B pin */
volatile static int32_t  fuel_valve_pos      = 0;  /* Fuel Valve Encoder count */
volatile static bool     fuel_channelA_state = ENCODER_LOW; /* Voltage on channel 
                                                             A pin */
volatile static bool     fuel_channelB_state = ENCODER_LOW; /* Voltage on Channel 
                                                             B pin */


/*------------------------------------------------------------------------------
 Internal Function Prototypes 
------------------------------------------------------------------------------*/



/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

#ifdef WIP
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_open_ox_valve                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Open the main oxidizer valve                                           *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_open_ox_valve
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
return VALVE_OK;
} /* valve_open_ox_valve */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_open_fuel_valve                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Open the main fuel valve                                               *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_open_fuel_valve
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Initializations
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
return VALVE_OK;
} /* valve_open_fuel_valve */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_close_ox_valve                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Close the main oxidizer valve                                          *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_close_ox_valve
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Initializations
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
return VALVE_OK;
} /* valve_close_ox_valve */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_close_fuel_valve                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Close the main fuel valve                                              *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_close_fuel_valve
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Initializations
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
return VALVE_OK;
} /* valve_close_fuel_valve */
#endif /* #ifdef WIP */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_get_ox_valve_pos                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Get the position of the main oxidizer valve                            *
*                                                                              *
*******************************************************************************/
int32_t valve_get_ox_valve_pos
	(
	void
	)
{
return lox_valve_pos*360/1000;
} /* valve_get_ox_valve_pos */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_get_fuel_valve_pos                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Get the position of the main fuel valve                                *
*                                                                              *
*******************************************************************************/
int32_t valve_get_fuel_valve_pos
	(
	void
	)
{
return fuel_valve_pos*360/1000;
} /* valve_get_fuel_valve_pos */

#ifdef WIP
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_get_ox_valve_state                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Get the state of the main oxidizer valve                               *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_get_ox_valve_state
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Initializations
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
return VALVE_OK;
} /* valve_get_ox_valve_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_get_main_valve_state                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Get the status of the main fuel valve                                  *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_get_main_valve_state
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Initializations
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
return VALVE_OK;
} /* valve_get_main_valve_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		valve_calibrate_valves                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calibrate initial valve positions                                      *
*                                                                              *
*******************************************************************************/
VALVE_STATUS valve_calibrate_valves
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Initializations
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
return VALVE_OK;
} /* valve_calibrate_valves */
#endif /* #ifdef WIP */

/*------------------------------------------------------------------------------
 Interrupt Service Routines 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lox_channelA_ISR                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       LOX Main Valve Encoder Channel A Interrupt                             *
*                                                                              *
*******************************************************************************/
void lox_channelA_ISR
	(
	void
	)
{
/* Low to High Transition */
if ( HAL_GPIO_ReadPin( LOX_ENC_GPIO_PORT, LOX_ENC_A_PIN ) )
	{
	lox_channelA_state = ENCODER_HIGH;
	if ( !lox_channelB_state )
		{
		lox_valve_pos -= 1;
		}
	}
/* High to Low Transition */
else
	{
	lox_channelA_state = ENCODER_LOW;
	}
} /* lox_channelA_ISR */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lox_channelB_ISR                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       LOX Main Valve Encoder Channel B Interrupt                             *
*                                                                              *
*******************************************************************************/
void lox_channelB_ISR
	(
	void
	)
{
/* Low to High Transition */
if ( HAL_GPIO_ReadPin( LOX_ENC_GPIO_PORT, LOX_ENC_B_PIN ) )
	{
	lox_channelB_state = ENCODER_HIGH;
	if ( !lox_channelA_state )
		{
		lox_valve_pos += 1;
		}
	}
/* High to Low Transition */
else
	{
	lox_channelB_state = ENCODER_LOW;
	}

} /* lox_channelB_ISR */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		fuel_channelA_ISR                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Fuel Main Valve Encoder Channel A Interrupt                            *
*                                                                              *
*******************************************************************************/
void fuel_channelA_ISR
	(
	void
	)
{
/* Low to High Transition */
if ( HAL_GPIO_ReadPin( KER_ENC_A_GPIO_PORT, KER_ENC_A_PIN ) )
	{
	fuel_channelA_state = ENCODER_HIGH;
	if ( !fuel_channelB_state )
		{
		fuel_valve_pos -= 1;
		}
	}
/* High to Low Transition */
else
	{
	fuel_channelA_state = ENCODER_LOW;
	}
} /* fuel_channelA_ISR */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		fuel_channelB_ISR                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Fuel Main Valve Encoder Channel B Interrupt                            *
*                                                                              *
*******************************************************************************/
void fuel_channelB_ISR
	(
	void
	)
{
/* Low to High Transition */
if ( HAL_GPIO_ReadPin( KER_ENC_B_GPIO_PORT, KER_ENC_B_PIN ) )
	{
	fuel_channelB_state = ENCODER_HIGH;
	if ( !fuel_channelA_state )
		{
		fuel_valve_pos += 1;
		}
	}
/* High to Low Transition */
else
	{
	fuel_channelB_state = ENCODER_LOW;
	}

} /* fuel_channelB_ISR */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/