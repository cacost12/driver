/*******************************************************************************
*
* FILE: 
* 		solenoid.h
*
* DESCRIPTION: 
* 		Basic solenoid actuation API
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SOLENOID_H
#define SOLENOID_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Solenoid Subcommand Codes */
#define SOL_ON_BASE_CODE	    0x00
#define SOL_OFF_BASE_CODE	    0x08
#define SOL_TOGGLE_BASE_CODE	0x10
#define SOL_RESET_CODE	        0x18


/*------------------------------------------------------------------------------
 Types                                                                     
------------------------------------------------------------------------------*/
struct sol_GPIO_handle {
	GPIO_TypeDef* GPIOx;    /* GPIO Port */
	uint16_t      GPIO_pin; /* Solenoid GPIO Pin */
};

/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
void solenoid_map
	(
	struct sol_GPIO_handle* sol_GPIO_config, /* Pointer to GPIO port and pin 
                                                configuration for target 
                                                solenoid                      */
	uint8_t solenoid_num /* Solenoid number to actuate */ 
	); /* solenoid_map */

void solenoid_cmd_execute
	(
	uint8_t solenoid_cmd_opcode  /* Solenoid actuation code */
	); /* solenoid_cmd_execute */

void solenoid_on
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	); /* solenoid_on */

void solenoid_off
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	); /* solenoid_off */

void solenoid_toggle
	(
	uint8_t solenoid_num  /* Solenoid number to actuate */
	); /* solenoid_toggle */

void solenoid_reset
	(
	void
	); /* solenoid_reset */

#ifdef __cplusplus
}
#endif

#endif /* SOLENOID_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/