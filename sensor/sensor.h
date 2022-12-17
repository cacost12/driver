/*******************************************************************************
*
* FILE: 
* 		sensor.h
*
* DESCRIPTION: 
* 		Contains functions to interface between sdec terminal commands and SDR
*       sensor APIs
*
*******************************************************************************/

// /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_H
#define SENSOR_H

#include "stm32h7xx_hal.h"
#include "imu.h"

/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/

/* GCC requires stdint.h for uint_t types */
#ifdef UNIT_TEST
	#include <stdint.h>
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Sensor subcommand codes */
#define SENSOR_DUMP_CODE    ( 0x01 )
#define SENSOR_POLL_CODE    ( 0x02 )

#if   defined( FLIGHT_COMPUTER   )
	/* General */
	#define NUM_SENSORS         ( 1   )
	#define IMU_DATA_SIZE       ( 20  )
	#define SENSOR_DATA_SIZE	( 28  )
#elif defined( ENGINE_CONTROLLER )
	/* General */
	#define NUM_SENSORS         ( 10  )
#else
	#error Board is not compatible with SENSOR module
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/
typedef enum 
	{
    SENSOR_OK = 0         ,
	SENSOR_UNRECOGNIZED_OP,
	SENSOR_UNSUPPORTED_OP ,
	SENSOR_IMU_FAIL       ,
	SENSOR_ACCEL_ERROR    ,
    SENSOR_GRYO_ERROR     ,
	SENSOR_MAG_ERROR      ,
	SENSOR_BARO_ERROR     ,
    SENSOR_FAIL
    } SENSOR_STATUS;

#if defined( FLIGHT_COMPUTER )
typedef struct SENSOR_DATA 
	{
	IMU_DATA imu_data;
	uint32_t baro_pressure;
	uint32_t baro_temp;	
	} SENSOR_DATA;
#endif /* #if defined( FLIGHT_COMPUTER ) */

/*------------------------------------------------------------------------------
 Public Function Prototypes 
------------------------------------------------------------------------------*/

#if ( defined( TERMINAL ) && defined( FLIGHT_COMPUTER ) )
/* Execute a sensor subcommand */
SENSOR_STATUS sensor_cmd_execute
	(
	uint8_t subcommand
    );
#endif /* #if ( defined( TERMINAL ) && defined( FLIGHT_COMPUTER ) ) */

#if ( defined( TERMINAL ) && defined( ENGINE_CONTROLLER ) )
/* Execute a sensor subcommand */
uint8_t sensor_cmd_execute
	(
	uint8_t subcommand
    );
#endif

#if defined( FLIGHT_COMPUTER )
/* Dump all sensor readings to console */
SENSOR_STATUS sensor_dump
	(
    SENSOR_DATA* sensor_data_ptr 
    );
#endif

#if defined( ENGINE_CONTROLLER )
/* Dump all sensor readings to console */
SENSOR_STATUS sensor_dump
	(
    uint32_t* pSensor_buffer 
    );
#endif


#endif /* SENSOR_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
