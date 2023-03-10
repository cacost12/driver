/*******************************************************************************
*
* FILE: 
* 		valve.h
*
* DESCRIPTION: 
* 		Servo valve actuation API	
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VALVE_H 
#define VALVE_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Encoder states */
#define ENCODER_HIGH              true
#define ENCODER_LOW               false


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Return Codes */
typedef enum _VALVE_STATUS
	{
	VALVE_OK         ,
	VALVE_INVALID_DIR, /* Invalid motor direction */
	VALVE_ERROR
	} VALVE_STATUS;

/* Stepper driver enable states */
typedef enum _STEPPER_DRIVER_EN_STATE
	{
	STEPPER_DRIVER_ENABLED,
	STEPPER_DRIVER_DISABLED
	} STEPPER_DRIVER_EN_STATE;

/* Stepper driver rotation directions */
typedef enum _STEPPER_DRIVER_DIR_STATE
	{
	STEPPER_DRIVER_CW,  /* Clockwise         */
	STEPPER_DRIVER_CCW  /* Counter clockwise */
	} STEPPER_DRIVER_DIR_STATE;

/* Stepper Motor driver state */
typedef struct _STEPPER_DRIVER_STATE
	{
	STEPPER_DRIVER_EN_STATE  enable;    /* Motor enabled      */
	STEPPER_DRIVER_DIR_STATE direction; /* Rotation Direction */
	} STEPPER_DRIVER_STATE; 


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

#ifdef WIP
/* Open the main oxidizer valve */
VALVE_STATUS valve_open_ox_valve
	(
	void
	);

/* Open the main fuel valve */
VALVE_STATUS valve_open_fuel_valve
	(
	void
	);

/* Close the main oxidizer valve */
VALVE_STATUS valve_close_ox_valve
	(
	void
	);

/* Close the main fuel valve */
VALVE_STATUS valve_close_fuel_valve
	(
	void
	);
#endif

/* LOX Main Valve Encoder Channel A Interrupt */
void lox_channelA_ISR
	(
	void
	);

/* LOX Main Valve Encoder Channel B Interrupt */
void lox_channelB_ISR
	(
	void
	);

/* Fuel Main Valve Encoder Channel A Interrupt */
void fuel_channelA_ISR
	(
	void
	);

/* Fuel Main Valve Encoder Channel B Interrupt */
void fuel_channelB_ISR
	(
	void
	);

/* Get the position of the main oxidizer valve */
int32_t valve_get_ox_valve_pos
	(
	void
	);

/* Get the position of the main fuel valve */
int32_t valve_get_fuel_valve_pos
	(
	void
	);

#ifdef WIP
/* Get the position of the main fuel valve */
VALVE_STATUS valve_get_fuel_valve_pos
	(
	void
	);

/* Get the state of the main oxidizer valve */
VALVE_STATUS valve_get_ox_valve_state
	(
	void
	);

/* Get the status of the main fuel valve */
VALVE_STATUS valve_get_main_valve_state
	(
	void
	);

/* Calibrate initial valve positions */
VALVE_STATUS valve_calibrate_valves
	(
	void
	);
#endif

#ifdef __cplusplus
}
#endif

#endif /* VALVE_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/