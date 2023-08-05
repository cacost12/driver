/*******************************************************************************
*
* FILE: 
* 		ignition.c
*
* DESCRIPTION: 
* 		Contains API function to the engine controller ignition system and 
*       contintuity readings
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( BASE_FLIGHT_COMPUTER        )
	#include "zav_pin_defines_A0001.h"
#elif defined( FULL_FLIGHT_COMPUTER        )
	#include "zav_pin_defines_A0002.h"
#elif defined( LEGACY_FLIGHT_COMPUTER      )
	#include "zav_pin_defines_A0003.h"
#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
	#include "zav_pin_defines_A0004.h"
#else
	#error "No ignition compatible device specified in Makefile"
#endif

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "ignition.h"


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_get_cont_info                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Polls each continuity pin and sets the continuity bits in the          *
*       response code                                                          *   
*                                                                              *
*******************************************************************************/
IGN_CONT_STATUS ign_get_cont_info
	(
    void
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
IGN_CONT_STATUS cont_status = 0; /* Status code to be returned */


/*------------------------------------------------------------------------------
 Call API functions 
------------------------------------------------------------------------------*/

/* Poll the switch, main parachute, and drogue parachute continuity pins */
if ( ign_switch_cont() )
	{
    cont_status |= IGN_SWITCH_MASK;
    }
if ( ign_main_cont()   )
	{
    cont_status |= IGN_MAIN_CONT_MASK;
    }
if ( ign_drogue_cont() )
	{
    cont_status |= IGN_DROGUE_CONT_MASK;
    }
return cont_status;

} /* ign_get_cont_info */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_deploy_main                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Asserts the ignition signal to ignite the main parachute deployment    *
*       ematch. Returns a response code indicating if the ignition occured     *
*       succesfully                                                            *
*                                                                              *
*******************************************************************************/
IGN_STATUS ign_deploy_main 
    (
	void
    )
{
/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check continuities before deploying*/
if      ( !ign_switch_cont() )
	{
	return IGN_SWITCH_FAIL;
	}
else if ( !ign_main_cont()   )
	{
    return IGN_CONT_FAIL; 
    }
else /* Continuity is good for main */
	{
	/* Assert ignition signal for 10 ms */
	HAL_GPIO_WritePin( MAIN_GPIO_PORT, MAIN_PIN, GPIO_PIN_SET );
	HAL_Delay( IGN_BURN_DELAY );
	HAL_GPIO_WritePin( MAIN_GPIO_PORT, MAIN_PIN, GPIO_PIN_RESET );
	}

/* Check ematch continuity to check that ematch was lit */
if ( !ign_main_cont() )
	{
	return IGN_SUCCESS;
	}
else /* Ignition unsuccessful */
	{
	return IGN_FAILED_TO_IGNITE;
	}

} /* ign_deploy_main */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_deploy_drogue                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Asserts the ignition signal to ignite the drogue parachute deployment  *
*       ematch. Returns a response code indicating if the ignition occured     *
*       succesfully                                                            *
*                                                                              *
*******************************************************************************/
IGN_STATUS ign_deploy_drogue 
    (
	void
    )
{
/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check continuities before deploying*/
if      ( !ign_switch_cont() )
	{
	return IGN_SWITCH_FAIL;
	}
else if ( !ign_drogue_cont()   )
	{
    return IGN_CONT_FAIL; 
    }
else /* Continuity is good for drogue */
	{
	/* Assert ignition signal for 10 ms */
	HAL_GPIO_WritePin( DROGUE_GPIO_PORT, DROGUE_PIN, GPIO_PIN_SET   );
	HAL_Delay( IGN_BURN_DELAY );
	HAL_GPIO_WritePin( DROGUE_GPIO_PORT, DROGUE_PIN, GPIO_PIN_RESET );
	}

/* Check ematch continuity to check that ematch was lit */
if ( !ign_drogue_cont() )
	{
	return IGN_SUCCESS;
	}
else /* Ignition unsuccessful */
	{
	return IGN_FAILED_TO_IGNITE;
	}

} /* ign_deploy_drogue */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_main_cont                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the main parachute          *
*       deployment ematch                                                      *
*                                                                              *
*******************************************************************************/
bool ign_main_cont
	(
	void
	)
{

/* Check MCU GPIO State */
uint8_t main_cont_pinstate = HAL_GPIO_ReadPin( MAIN_CONT_GPIO_PORT, MAIN_CONT_PIN );

/* Return true if GPIO state is low */
if ( main_cont_pinstate == 0 )
	{
    return false;
	}
else
	{
    return true;
    }

} /* ign_main_cont */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_drogue_cont                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the drogue parachute        * 
*       deployment ematch                                                      *
*                                                                              *
*******************************************************************************/
bool ign_drogue_cont
	(
	void
	)
{

/* Check MCU GPIO State */
uint8_t drogue_cont_pinstate = HAL_GPIO_ReadPin( DROGUE_CONT_GPIO_PORT, 
                                                 DROGUE_CONT_PIN );

/* Return true if GPIO state is low */
if ( drogue_cont_pinstate == 0 )
	{
    return false;
	}
else
	{
    return true;
    }

} /* drogue_cont */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_switch_cont                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the switch screw terminals  * 
*                                                                              *
*******************************************************************************/
bool ign_switch_cont
	(
	void
	)
{
/* Check MCU GPIO State */
uint8_t switch_cont_pinstate = HAL_GPIO_ReadPin(SWITCH_GPIO_PORT, SWITCH_PIN);

/* Return true if GPIO state is low */
if ( switch_cont_pinstate == 0 )
	{
    return false;
	}
else
	{
    return true;
    }

} /* switch_cont */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/