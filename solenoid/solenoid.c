/*******************************************************************************
*
* FILE:
* 		solenoid.c
*
* DESCRIPTION:
* 		Basic solenoid actuation API
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "solenoid.h"
#include "stm32h7xx_hal.h"


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_map                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Maps a solenoid number to a microcontroller port and pin number        *
*                                                                              *
*******************************************************************************/
void solenoid_map
	(
	struct sol_GPIO_handle* psol_GPIO_handle, /* Pointer to GPIO port and pin
                                                 configuration for target
                                                 solenoid                     */
	uint8_t solenoid_num /* Solenoid number to actuate */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
uint8_t solenoid_pin_map[] =        {2, 3, 4, 0, 1, 2};    /* Solenoid bit
                                                             shifts          */
GPIO_TypeDef* solenoid_port_map[] = {SOL1_PORT, SOL2_PORT, /* Solenoid GPIO  */
                                     SOL3_PORT, SOL4_PORT,
                                     SOL5_PORT, SOL6_PORT};


/*------------------------------------------------------------------------------
 Mapping
------------------------------------------------------------------------------*/
/* Port Setting Mapping */
psol_GPIO_handle -> GPIOx    = solenoid_port_map[solenoid_num-1];

/* Pin Setting Mapping */
psol_GPIO_handle -> GPIO_pin = 1 << solenoid_pin_map[solenoid_num-1];

} /* solenoid_map */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_cmd_execute                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Executes a solenoid function based on a subcommand code from a PC      *
*                                                                              *
*******************************************************************************/
void solenoid_cmd_execute
	(
	uint8_t solenoid_cmd_opcode  /* Solenoid actuation code */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
uint8_t solenoid_bitmask = 0x07; /* Solenoid bits: 0000 0111 */
uint8_t solenoid_base_code; /* Subcommand base code */
uint8_t solenoid_number; /* Solenoid number to actuate */

/*------------------------------------------------------------------------------
 Command Input Processing
------------------------------------------------------------------------------*/
// Extract Solenoid base code
solenoid_base_code = (~solenoid_bitmask) & solenoid_cmd_opcode;

// Extract Solenoid number
solenoid_number = solenoid_bitmask & solenoid_cmd_opcode;

/*------------------------------------------------------------------------------
 Call Solenoid API Function
------------------------------------------------------------------------------*/
switch(solenoid_base_code)
	{
	/* Solenoid On */
	case SOL_ON_BASE_CODE:
		solenoid_on(solenoid_number);
		break;

	/* Solenoid Off */
	case SOL_OFF_BASE_CODE:
		solenoid_off(solenoid_number);
		break;

	/* Solenoid Toggle */
	case SOL_TOGGLE_BASE_CODE:
		solenoid_toggle(solenoid_number);
		break;

	/* Solenoid Reset */
	case SOL_RESET_CODE:
		solenoid_reset();
		break;

	default:
		/* Do nothing */
      break;
	}
} /* solenoid_cmd_execute */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_on                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		 Applies power to a specified solenoid                                 *
*                                                                              *
*******************************************************************************/
void solenoid_on
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
struct sol_GPIO_handle solenoid_on_GPIO_handle;

/*------------------------------------------------------------------------------
 Actuation
------------------------------------------------------------------------------*/
/* Solenoid number to GPIO port/pin mapping */
solenoid_map(&solenoid_on_GPIO_handle, solenoid_num);

/* HAL GPIO Driver Call */
HAL_GPIO_WritePin(solenoid_on_GPIO_handle.GPIOx,
                       solenoid_on_GPIO_handle.GPIO_pin,
                       GPIO_PIN_SET);
} /* solenoid_on */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_off                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		 Removes power from a specified solenoid                               *
*                                                                              *
*******************************************************************************/
void solenoid_off
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
struct sol_GPIO_handle solenoid_on_GPIO_handle;

/*------------------------------------------------------------------------------
 Actuation
------------------------------------------------------------------------------*/
/* Solenoid number to GPIO port/pin mapping */
solenoid_map(&solenoid_on_GPIO_handle, solenoid_num);

/* HAL GPIO Driver Call */
HAL_GPIO_WritePin(solenoid_on_GPIO_handle.GPIOx,
                       solenoid_on_GPIO_handle.GPIO_pin,
                       GPIO_PIN_RESET);
} /* solenoid_off */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_toggle                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		 Toggles the actuation state of a specified solenoid                   *
*                                                                              *
*******************************************************************************/
void solenoid_toggle
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
struct sol_GPIO_handle solenoid_on_GPIO_handle;

/*------------------------------------------------------------------------------
 Actuation
------------------------------------------------------------------------------*/
/* Solenoid number to GPIO port/pin mapping */
solenoid_map(&solenoid_on_GPIO_handle, solenoid_num);

/* HAL GPIO Driver Call */
HAL_GPIO_TogglePin(solenoid_on_GPIO_handle.GPIOx,
                       solenoid_on_GPIO_handle.GPIO_pin);
} /* solenoid_toggle */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		solenoid_reset                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		 Resets all solenoids to their default state                           *
*                                                                              *
*******************************************************************************/
void solenoid_reset
	(
	void
	)
{
// Call HAL_GPIO_WritePin for all pins
HAL_GPIO_WritePin(SOL1_PORT, SOL1_PIN|SOL2_PIN|SOL3_PIN, GPIO_PIN_RESET);
HAL_GPIO_WritePin(SOL4_PORT, SOL4_PIN|SOL5_PIN|SOL6_PIN, GPIO_PIN_RESET);
} /* solenoid_reset */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
