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

/* Encoder variables */
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

/* Stepper Driver States */
static STEPPER_DRIVER_STATE lox_driver_state;
static STEPPER_DRIVER_STATE fuel_driver_state;


/*------------------------------------------------------------------------------
 Internal Function Prototypes 
------------------------------------------------------------------------------*/

/* Enable the lox stepper motor driver */
static void lox_driver_enable
	(
	void
	);

/* Enable the fuel stepper motor driver */
static void fuel_driver_enable
	(
	void
	);

/* Disable the lox stepper motor driver */
static void lox_driver_disable
	(
	void
	);

/* Disable the fuel stepper motor driver */
static void fuel_driver_disable
	(
	void
	);

/* Set the lox stepper motor direction */
static VALVE_STATUS lox_driver_set_direction
	(
	STEPPER_DRIVER_DIR_STATE direction
	);

/* Set the fuel stepper motor direction */
static VALVE_STATUS fuel_driver_set_direction
	(
	STEPPER_DRIVER_DIR_STATE direction
	);


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

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
VALVE_STATUS valve_status; /* Status return codes from valve API */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
valve_status = VALVE_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Enable the driver   */
//lox_driver_enable();

/* Set the direction   */
//valve_status = lox_driver_set_direction( STEPPER_DRIVER_CCW );
//if ( valve_status != VALVE_OK )
//	{
	//return valve_status;
	//}

/* Actuate the valve   */
//HAL_TIM_PWM_Start( &( VALVE_TIM ), LOX_TIM_CHANNEL );
//HAL_Delay( 1000 );
//HAL_TIM_PWM_Stop ( &( VALVE_TIM ), LOX_TIM_CHANNEL );
while (1)
	{
	HAL_GPIO_TogglePin( LOX_DIR_GPIO_PORT, LOX_DIR_PIN );
	HAL_Delay( 1 );
	}

/* Turn off the driver */
//lox_driver_disable();

return VALVE_OK;
} /* valve_open_ox_valve */


#ifdef WIP
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
*                                                                              *
* PROCEDURE:                                                                   *
* 		lox_driver_enable                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Enable the lox stepper motor driver                                    *
*                                                                              *
*******************************************************************************/
static void lox_driver_enable
	(
	void
	)
{
/* Set GPIO State      */
HAL_GPIO_WritePin( LOX_EN_GPIO_PORT, LOX_EN_PIN, GPIO_PIN_SET );
while(1)
	{
	HAL_GPIO_TogglePin( LOX_DIR_GPIO_PORT, LOX_DIR_PIN );
	HAL_Delay(1);
	}

/* Update global state */
lox_driver_state.enable = STEPPER_DRIVER_ENABLED;

} /* lox_driver_enable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		fuel_driver_enable                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Enable the fuel stepper motor driver                                   *
*                                                                              *
*******************************************************************************/
static void fuel_driver_enable
	(
	void
	)
{
/* Set GPIO level           */
HAL_GPIO_WritePin( KER_EN_GPIO_PORT, KER_EN_PIN, GPIO_PIN_SET );

/* Update the globale state */
fuel_driver_state.enable = STEPPER_DRIVER_ENABLED;

} /* fuel_driver_enable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lox_driver_disable                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Disable the lox stepper motor driver                                   *
*                                                                              *
*******************************************************************************/
static void lox_driver_disable
	(
	void
	)
{
/* Set GPIO Level          */
HAL_GPIO_WritePin( LOX_EN_GPIO_PORT, LOX_EN_PIN, GPIO_PIN_RESET );

/* Update the global state */
lox_driver_state.enable = STEPPER_DRIVER_DISABLED;

} /* lox_driver_disable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		fuel_driver_disable                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Disable the fuel stepper motor driver                                  *
*                                                                              *
*******************************************************************************/
static void fuel_driver_disable
	(
	void
	)
{
/* Set GPIO level          */
HAL_GPIO_WritePin( KER_EN_GPIO_PORT, KER_EN_PIN, GPIO_PIN_RESET );

/* Update the global state */
fuel_driver_state.enable = STEPPER_DRIVER_DISABLED;

} /* fuel_driver_disable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lox_driver_set_direction                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set the lox stepper motor direction                                    *
*                                                                              *
*******************************************************************************/
static VALVE_STATUS lox_driver_set_direction
	(
	STEPPER_DRIVER_DIR_STATE direction
	)
{
/* Set the GPIO level      */
if      ( direction == STEPPER_DRIVER_CW  )
	{
	HAL_GPIO_WritePin( LOX_DIR_GPIO_PORT, LOX_DIR_PIN, GPIO_PIN_SET );
	}
else if ( direction == STEPPER_DRIVER_CCW )
	{
	HAL_GPIO_WritePin( LOX_DIR_GPIO_PORT, LOX_DIR_PIN, GPIO_PIN_RESET );
	}
else
	{
	/* Invalid direction */
	return VALVE_INVALID_DIR;
	}

/* Update the global state */
lox_driver_state.direction = direction;
return VALVE_OK;
} /* lox_driver_set_direction */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		fuel_driver_set_direction                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set the fuel stepper motor direction                                   *
*                                                                              *
*******************************************************************************/
static VALVE_STATUS fuel_driver_set_direction
	(
	STEPPER_DRIVER_DIR_STATE direction
	)
{
/* Set the GPIO level      */
if      ( direction == STEPPER_DRIVER_CW  )
	{
	HAL_GPIO_WritePin( KER_DIR_GPIO_PORT, KER_DIR_PIN, GPIO_PIN_SET );
	}
else if ( direction == STEPPER_DRIVER_CCW )
	{
	HAL_GPIO_WritePin( KER_DIR_GPIO_PORT, KER_DIR_PIN, GPIO_PIN_RESET );
	}
else
	{
	return VALVE_INVALID_DIR;
	}

/* Update the global state */
fuel_driver_state.direction = direction;
return VALVE_OK;
} /* fuel_driver_set_direction */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/