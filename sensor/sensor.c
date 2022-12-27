/*******************************************************************************
*
* FILE: 
* 		sensor.c
*
* DESCRIPTION: 
* 		Contains functions to interface between sdec terminal commands and SDR
*       sensor APIs
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER   )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER )
	#include "sdr_pin_defines_L0002.h"
#endif 


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#if defined( FLIGHT_COMPUTER )
	#include "imu.h"
	#include "baro.h"
#endif
#include "usb.h"
#include "sensor.h"
#if defined( ENGINE_CONTROLLER )
	#include "pressure.h"
#endif


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Public procedures 
------------------------------------------------------------------------------*/

#if ( defined( TERMINAL ) && defined( FLIGHT_COMPUTER ) )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		sensor_cmd_execute                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Executes a sensor subcommand                                           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_cmd_execute 
	(
    uint8_t subcommand 
    )
{

/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
SENSOR_STATUS sensor_subcmd_status;                 /* Status indicating if 
                                                       subcommand function 
                                                       returned properly      */
USB_STATUS    usb_status;                            /* USB return codes      */
SENSOR_DATA   sensor_data;                           /* Struct with all sensor 
                                                        data                  */
uint8_t       sensor_data_bytes[ SENSOR_DATA_SIZE ]; /* Byte array with sensor 
                                                       readouts               */
uint8_t       num_sensor_bytes = SENSOR_DATA_SIZE;   /* Size of data in bytes */
uint8_t       num_sensors;                           /* Number of sensors to 
                                                        use for polling       */
uint8_t       poll_sensors[ SENSOR_MAX_NUM_POLL ];   /* Codes for sensors to
                                                        be polled             */

/*------------------------------------------------------------------------------
 Initializations  
------------------------------------------------------------------------------*/
usb_status  = USB_OK;
num_sensors = 0;
memset( &sensor_data_bytes[0], 0, sizeof( sensor_data_bytes ) );
memset( &sensor_data         , 0, sizeof( sensor_data       ) );
memset( &poll_sensors[0]     , 0, sizeof( poll_sensors      ) );


/*------------------------------------------------------------------------------
 Execute Sensor Subcommand 
------------------------------------------------------------------------------*/
switch ( subcommand )
	{

	/* Poll Sensors continuously */
    case SENSOR_POLL_CODE:
		{
		/* Determine the number of sensors to poll */
		usb_status = usb_receive( &num_sensors, 
		                          sizeof( num_sensors ), 
								  HAL_DEFAULT_TIMEOUT );
		if ( usb_status != USB_OK )
			{
			return SENSOR_USB_FAIL;
			}

		/* Determine which sensors to poll */
		usb_status = usb_receive( &poll_sensors[0],
		                          num_sensors     , 
                                  HAL_SENSOR_TIMEOUT );
		if ( usb_status != USB_OK )
			{
			return SENSOR_USB_FAIL;
			}

		/* Start Polling sensors */
		
		return ( SENSOR_OK );
        } /* SENSOR_POLL_CODE */ 

	/* Poll sensors once and dump data on terminal */
	case SENSOR_DUMP_CODE: 
		{
		/* Tell the PC how many bytes to expect */
		usb_transmit( &num_sensor_bytes,
                      sizeof( num_sensor_bytes ), 
                      HAL_DEFAULT_TIMEOUT );

		/* Get the sensor readings */
	    sensor_subcmd_status = sensor_dump( &sensor_data );	

		/* Convert to byte array */
		memcpy( &(sensor_data_bytes[0]), &sensor_data, sizeof( sensor_data ) );

		/* Transmit sensor readings to PC */
		if ( sensor_subcmd_status == SENSOR_OK )
			{
			// readings_to_bytes( &sensor_readings_bytes[0], 
            //                    &sensor_readings[0] );
			usb_transmit( &sensor_data_bytes[0]      , 
                          sizeof( sensor_data_bytes ), 
                          HAL_SENSOR_TIMEOUT );
			return ( sensor_subcmd_status );
            }
		else
			{
			/* Sensor readings not recieved */
			return( SENSOR_FAIL );
            }
        } /* SENSOR_DUMP_CODE */

	/* Subcommand not recognized */
	default:
		{
		return ( SENSOR_UNRECOGNIZED_OP );
        }
    }

} /* sensor_cmd_execute */


#endif /* #if ( defined( TERMINAL ) && defined( FLIGHT_COMPUTER ) ) */


#if ( defined( TERMINAL ) && defined( ENGINE_CONTROLLER ) )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_cmd_execute                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Executes a sensor subcommand                                           *
*                                                                              *
*******************************************************************************/
uint8_t sensor_cmd_execute 
	(
    uint8_t subcommand 
    )
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
SENSOR_STATUS sensor_subcmd_status;           /* Status indicating if 
                                                 subcommand function returned 
                                                 properly                     */
uint32_t      sensor_readings[ NUM_SENSORS ]; /* Readings obtained from each 
                                                 sensor                       */
uint8_t       sensor_readings_bytes[ 4*NUM_SENSORS ];
uint8_t       num_sensor_bytes = 4*NUM_SENSORS; /* Number of bytes to be 
                                                   transmitted back to PC     */

/*------------------------------------------------------------------------------
 Initializations  
------------------------------------------------------------------------------*/

/* Set sensor readings to zero */
memset( &sensor_readings[0]      , 0, sizeof( sensor_readings       ) );
memset( &sensor_readings_bytes[0], 0, sizeof( sensor_readings_bytes ) );


/*------------------------------------------------------------------------------
 Execute Sensor Subcommand 
------------------------------------------------------------------------------*/
switch ( subcommand )
	{

	/* Poll Sensors continuously */
    case SENSOR_POLL_CODE:
		{
		// TODO: Implement sensor poll function 
		return ( SENSOR_UNSUPPORTED_OP );
        } /* SENSOR_POLL_CODE */ 

	/* Poll sensors once and dump data on terminal */
	case SENSOR_DUMP_CODE: 
		{
		/* Tell the PC how many bytes to expect */
		usb_transmit( &num_sensor_bytes         ,
		              sizeof( num_sensor_bytes ),
					  HAL_DEFAULT_TIMEOUT );

		/* Get the sensor readings */
	    sensor_subcmd_status = sensor_dump( &sensor_readings[0] );	

		/* Transmit sensor readings to PC */
		if ( sensor_subcmd_status == SENSOR_OK )
			{
			memcpy( &sensor_readings_bytes[0], 
			        &sensor_readings[0]      , 
					sizeof( sensor_readings ) );
			usb_transmit( &sensor_readings_bytes[0],
			              sizeof( sensor_readings_bytes ),
						  HAL_SENSOR_TIMEOUT );
			return ( sensor_subcmd_status );
            }
		else
			{
			/* Sensor readings not recieved */
			return( SENSOR_FAIL );
            }
        } /* SENSOR_DUMP_CODE */

	/* Subcommand not recognized */
	default:
		{
		return ( SENSOR_UNRECOGNIZED_OP );
        }
    }
} /* sensor_cmd_execute */
#endif /* #if ( defined( TERMINAL ) && defined( ENGINE_CONTROLLER ) )*/


#if defined( FLIGHT_COMPUTER )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		sensor_dump                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       reads from all sensors and fill in the sensor data structure           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_dump 
	(
    SENSOR_DATA*        sensor_data_ptr /* Pointer to the sensor data struct should 
                                        be written */ 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
IMU_STATUS      accel_status;           /* IMU sensor status codes     */       
IMU_STATUS      gyro_status;
IMU_STATUS      mag_status; 
BARO_STATUS     press_status;           /* Baro Sensor status codes    */
BARO_STATUS     temp_status;


/*------------------------------------------------------------------------------
 Call sensor API functions 
------------------------------------------------------------------------------*/

/* Poll the IMU sensors */
accel_status = imu_get_accel_xyz( &(sensor_data_ptr->imu_data) ); 
gyro_status  = imu_get_gyro_xyz ( &(sensor_data_ptr->imu_data) );
mag_status   = imu_get_mag_xyz  ( &(sensor_data_ptr->imu_data) );
sensor_data_ptr -> imu_data.temp = 0;     // Figure out what to do with this 
                                          // readout, temporarily being used 
                                          // as struct padding

/* Poll the Baro sensors */
press_status = baro_get_pressure( &(sensor_data_ptr -> baro_pressure ) );
temp_status  = baro_get_temp    ( &(sensor_data_ptr -> baro_temp     ) );


/*------------------------------------------------------------------------------
 Set command status from sensor API returns 
------------------------------------------------------------------------------*/
if      ( accel_status != IMU_OK )
	{
	return SENSOR_ACCEL_ERROR;
	}
else if ( gyro_status  != IMU_OK )
	{
	return SENSOR_GRYO_ERROR;
	}
else if ( mag_status   != IMU_OK )
	{
	return SENSOR_MAG_ERROR;	
	}
else if ( press_status != BARO_OK ||
          temp_status  != BARO_OK  )
	{
	return SENSOR_BARO_ERROR;
	}
else
	{
	return SENSOR_OK;
	}
} /* sensor_dump */
#endif /* #if defined( FLIGHT_COMPUTER ) */

#if defined( ENGINE_CONTROLLER )
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		sensor_dump                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       reads from all sensors and transmits data back to host PC              *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_dump 
	(
    uint32_t* pSensor_buffer /* Pointer to buffer where sensor data should 
                                be written */ 
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
PRESSURE_STATUS pt_status; /* Status of pressure transducers */


/*------------------------------------------------------------------------------
 Call sensor API functions 
------------------------------------------------------------------------------*/

pt_status = pressure_poll_pts( pSensor_buffer ); /* Pressure transducers */
/* Poll load cell            */
// TODO
/* Poll thermocouple         */
// TODO


/*------------------------------------------------------------------------------
 Set command status from sensor API returns 
------------------------------------------------------------------------------*/
if ( pt_status != PRESSURE_OK )
	{
	return ( SENSOR_PT_FAIL );
    }
else
	{
	return ( SENSOR_OK );
    }

} /* sensor_dump */
#endif /* #if defined( ENGINE_CONTROLLER ) */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_poll                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Poll specific sensors on the board                                     *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_poll
	(
	uint8_t**  sensor_data_ptrs, /* Array of pointers to export data */
	SENSOR_ID* sensor_ids_ptr  , /* Array containing sensor IDS      */
	uint8_t    num_sensors       /* Number of sensors to poll        */
	)
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
uint8_t*   sensor_data_ptr;  /* Target location of next readout     */
uint8_t**  sensor_data_pptr; /* Pointer to next target location     */
SENSOR_ID  sensor_id;        /* ID of sensor currently being polled */
SENSOR_ID* sensor_id_ptr;    /* Pointer to sensor id                */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
sensor_data_pptr = sensor_data_ptrs;
sensor_id_ptr    = sensor_ids_ptr;
sensor_data_ptr  = *(sensor_data_pptr);
sensor_id        = *(sensor_id_ptr   );


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Iterate over each sensor readout */
for ( int i = 0; i < num_sensors; ++i )
	{
	
	/* Poll sensor */
	switch ( sensor_id )
		{
		#if defined( FLIGHT_COMPUTER )
			case SENSOR_ACCX:
				{
				break;
				}

			case SENSOR_ACCY:
				{
				break;
				}

			case SENSOR_ACCZ:
				{
				break;
				}

			case SENSOR_GYROX:
				{
				break;
				}

			case SENSOR_GYROY:
				{
				break;
				}

			case SENSOR_GYROZ:
				{
				break;
				}

			case SENSOR_MAGX:
				{
				break;
				}

			case SENSOR_MAGY:
				{
				break;
				}

			case SENSOR_MAGZ:
				{
				break;
				}

			case SENSOR_IMUT:
				{
				break;
				}

			case SENSOR_PRES:
				{
				break;
				}

			case SENSOR_TEMP:
				{
				break;
				}

		#elif defined( ENGINE_CONTROLLER )
			case SENSOR_PT0:
				{
				break;
				}

			case SENSOR_PT1:
				{
				break;
				}

			case SENSOR_PT2:
				{
				break;
				}

			case SENSOR_PT3:
				{
				break;
				}

			case SENSOR_PT4:
				{
				break;
				}

			case SENSOR_PT5:
				{
				break;
				}

			case SENSOR_PT6:
				{
				break;
				}

			case SENSOR_PT7:
				{
				break;
				}

			case SENSOR_TC:
				{
				break;
				}

			case SENSOR_LC:
				{
				break;
				}

		#endif /* #elif defined( ENGINE_CONTROLLER ) */

		default:
			{
			/* Unrecognized sensor id */
			return SENSOR_UNRECOGNIZED_SENSOR_ID; 
			}
		} /* switch( sensor_id ) */

		/* Go to next sensor */
		sensor_data_pptr++;
		sensor_id_ptr++;
		sensor_data_ptr  = *(sensor_data_pptr);
		sensor_id        = *(sensor_id_ptr   );

	} /*  while( i < num_sensors ) */

return SENSOR_OK;
} /* sensor_poll */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/