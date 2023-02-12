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
#define ENCODER_HIGH    true
#define ENCODER_LOW     false


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Return Codes */
typedef enum _VALVE_STATUS
	{
	VALVE_OK,
	VALVE_ERROR
	} VALVE_STATUS;


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

/* Get the position of the main oxidizer valve */
uint32_t valve_get_ox_valve_pos
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

#endif /* VALVE_H */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/