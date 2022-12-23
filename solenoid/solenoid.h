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
#define SOL_OP     0x51 /* solenoid command line opcode */

/* Microcontroller Pin Assignments */
#define SOL1_PIN   GPIO_PIN_2
#define SOL2_PIN   GPIO_PIN_3
#define SOL3_PIN   GPIO_PIN_4
#define SOL4_PIN   GPIO_PIN_0
#define SOL5_PIN   GPIO_PIN_1
#define SOL6_PIN   GPIO_PIN_2

/* Microcontroller Pin Assignments */
#define SOL1_PORT   GPIOE
#define SOL2_PORT   GPIOE
#define SOL3_PORT   GPIOE
#define SOL4_PORT   GPIOA
#define SOL5_PORT   GPIOA
#define SOL6_PORT   GPIOA

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

#endif /* SOLENOID_H */
