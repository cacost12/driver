/*******************************************************************************
*
* FILE: 
* 		baro.c
*
* DESCRIPTION: 
* 		Contains API functions for the barometric pressure sensor
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_A0002.h"
#include "baro.h"


/*------------------------------------------------------------------------------
Global Variables  
------------------------------------------------------------------------------*/

/* Current Baro Sensor configuration */
static BARO_CONFIG baro_configuration;

/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       baro_init                                                              *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Intialize the barometric pressure sensor                               *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_init
	(
	BARO_CONFIG* config_ptr
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
BARO_STATUS       baro_status;    /* Status code from Baro API calls   */
uint8_t           baro_device_id; /* Baro device id                    */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
baro_status    = BARO_OK;
baro_device_id = 0;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Verify functional I2C connection to sensor */
baro_status = baro_get_device_id( &baro_device_id );
if      ( baro_status   != BARO_OK         )
	{
	return BARO_I2C_ERROR;
	}
else if ( baro_device_id != BARO_DEVICE_ID )
	{
	return BARO_UNRECOGNIZED_CHIP_ID;
	}

/* Configure the sensor */
baro_status = baro_config( config_ptr );
return baro_status;

} /* baro_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       baro_config                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Configure the barometric pressure sensor                               *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_config
	(
	BARO_CONFIG* config_ptr
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;     /* Status codes from HAL API         */
uint8_t           pwr_ctrl;       /* Contents of the PWR_CTRL register */
uint8_t           osr;            /* Contents of the OSR register      */
uint8_t           odr;            /* Contents of the ODR register      */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status     = HAL_OK;
pwr_ctrl       = 0;
osr            = 0;
odr            = 0;


/*------------------------------------------------------------------------------
 Extract Baro settings from config struct  
------------------------------------------------------------------------------*/

/* Set register contents */
pwr_ctrl |= config_ptr -> enable;                /* Enable sensors           */
pwr_ctrl |= config_ptr -> mode << 4;             /* Set operating mode       */
osr      |= config_ptr -> press_OSR_setting;     /* Pressure oversampling    */
osr      |= config_ptr -> temp_OSR_setting << 3; /* Temperature oversampling */
odr      |= config_ptr -> ODR_setting;           /* Sampling Frequency       */

/* Set global baro configuration */
baro_configuration.enable            = config_ptr -> enable;
baro_configuration.mode              = config_ptr -> mode;
baro_configuration.press_OSR_setting = config_ptr -> press_OSR_setting;
baro_configuration.temp_OSR_setting  = config_ptr -> temp_OSR_setting;
baro_configuration.ODR_setting       = config_ptr -> ODR_setting;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Write to the PWR_CTRL register -> Operating mode and enable sensors */
hal_status = HAL_I2C_Mem_Write( &( BARO_I2C ), 
                                BARO_I2C_ADDR       , 
								BARO_REG_PWR_CTRL   ,
				                I2C_MEMADD_SIZE_8BIT,
							    &pwr_ctrl           ,
								sizeof( pwr_ctrl )  ,
								HAL_DEFAULT_TIMEOUT );

if ( hal_status != HAL_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Write to the OSR register -> set the oversampling rate */
hal_status = HAL_I2C_Mem_Write( &( BARO_I2C ),
                                BARO_I2C_ADDR, 
                                BARO_REG_OSR ,
                                I2C_MEMADD_SIZE_8BIT,
                                &osr                ,
                                sizeof( osr )       ,
                                HAL_DEFAULT_TIMEOUT );
if ( hal_status != HAL_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Write to the ODR register -> set the sampling frequency */
hal_status = HAL_I2C_Mem_Write( &( BARO_I2C ),
                                BARO_I2C_ADDR, 
                                BARO_REG_ODR ,
                                I2C_MEMADD_SIZE_8BIT,
                                &odr,
                                sizeof( odr )       ,
                                HAL_DEFAULT_TIMEOUT );
if ( hal_status != HAL_OK )
	{
	return BARO_I2C_ERROR;
	}
else
	{
	return BARO_OK;
	}

} /* baro_config */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       BARO_Read_Register                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Read one register for BARO                                             *
*                                                                              *
*******************************************************************************/
BARO_STATUS BARO_Read_Register
	(
	uint8_t reg_addr, 
	uint8_t *pData
	)
{   

/*------------------------------------------------------------------------------
Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;


/*------------------------------------------------------------------------------
API function implementation 
------------------------------------------------------------------------------*/

/* Read I2C register*/
hal_status = HAL_I2C_Mem_Read( &( BARO_I2C ),
				               BARO_I2C_ADDR,
				               reg_addr,
				               I2C_MEMADD_SIZE_8BIT,
				               pData,
				               sizeof( uint8_t ),
				               HAL_DEFAULT_TIMEOUT
				             );

if (hal_status != HAL_TIMEOUT)
	{
	return BARO_OK;
	}
else
	{
	return BARO_TIMEOUT;
	}
} /* BARO_Read_Register */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       BARO_Read_Registers                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Read the specific numbers of registers at one time for BARO            *
*                                                                              *
*******************************************************************************/
BARO_STATUS BARO_Read_Registers
	(
	uint8_t reg_addr,
	uint8_t *pData,
	uint8_t num_registers
	)
{
/*------------------------------------------------------------------------------
Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;


/*------------------------------------------------------------------------------
API function implementation 
------------------------------------------------------------------------------*/

/*Read I2C register*/
hal_status = HAL_I2C_Mem_Read( &( BARO_I2C ),
                               BARO_I2C_ADDR,
                               reg_addr,
                               I2C_MEMADD_SIZE_8BIT,
                               pData,
                               num_registers,
                               HAL_DEFAULT_TIMEOUT );

if (hal_status != HAL_TIMEOUT)
	{
	return BARO_OK;
	}
else
	{
	return BARO_TIMEOUT;
	}
} /* BARO_Read_Registers */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_device_id                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Gets the device ID of the barometric pressure sensor, primarily used   *
*       to verify that the sensor can be accessed by the MCU                   *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_device_id
	(
   	uint8_t* baro_id_ptr /* reference to memory where id is returned */ 
	)
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;


/*------------------------------------------------------------------------------
 API Function implementation 
------------------------------------------------------------------------------*/

/* Read baro register with I2C */
hal_status = HAL_I2C_Mem_Read (
                               &( BARO_I2C ),
                               BARO_I2C_ADDR       ,
                               BARO_REG_CHIP_ID    ,
                               I2C_MEMADD_SIZE_8BIT,
							   baro_id_ptr         ,
							   sizeof( uint8_t )   ,
                               HAL_DEFAULT_TIMEOUT
                              );

/* Check HAL Status and return data if okay */
switch ( hal_status )
	{
	case HAL_OK: 
		return BARO_OK;
		break;

	case HAL_TIMEOUT:
		return BARO_TIMEOUT;
		break;

	case HAL_ERROR:
		return BARO_I2C_ERROR;
		break;

	default:
		return BARO_UNRECOGNIZED_HAL_STATUS;
		break;
	}

} /* baro_get_device_id */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_pressure                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		retrieves a pressure reading from the sensor                           *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_pressure
	(
    uint32_t* pressure_ptr 
	)
{
/*------------------------------------------------------------------------------
Local variables 
------------------------------------------------------------------------------*/
uint8_t     pressure_bytes[3]; /* Pressure raw readout bytes, LSB first   */
uint32_t    raw_pressure;      /* Pressure raw readout in uint32_t format */
BARO_STATUS baro_status;       /* Return codes for baro API calls         */
uint8_t     osr_bitshift;      /* Number of XLS bits due to OSR setting   */


/*------------------------------------------------------------------------------
Initializations
------------------------------------------------------------------------------*/
raw_pressure = 0;
baro_status  = BARO_OK;
osr_bitshift = baro_configuration.press_OSR_setting;
memset( &pressure_bytes[0], 0, sizeof( pressure_bytes ) );

/*------------------------------------------------------------------------------
API function implementation 
------------------------------------------------------------------------------*/

/* Read 3 consecutive pressure data registers */
baro_status = BARO_Read_Registers( BARO_REG_PRESS_DATA,
                                   &pressure_bytes[0] ,
                                   3 );

/* Check for HAL BARO error */
if ( baro_status == BARO_TIMEOUT )
	{
	return BARO_TIMEOUT;
	}

/* Combine all bytes value to 24 bit value */
raw_pressure = ( ( (uint32_t) pressure_bytes[2] << (8 + osr_bitshift) ) |
                 ( (uint32_t) pressure_bytes[1] <<      osr_bitshift  ) |
				 ( (uint32_t) pressure_bytes[0]                ) );

/* Export data */
*pressure_ptr = raw_pressure;

return BARO_OK;
} /* baro_get_pressure */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_temp                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		retrieves a temperature reading from the sensor                        *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_temp
	(
    uint32_t* temp_ptr 
	)
{
/*------------------------------------------------------------------------------
Local variables 
------------------------------------------------------------------------------*/
uint8_t     temp_bytes[3]; /* Raw temperature readout bytes, LSB first */
uint32_t    raw_temp;      /* Raw temperature readout in uint32_t format */
BARO_STATUS baro_status;   /* Status codes returned from baro API calls  */
uint8_t     osr_bitshift;  /* Bitshift due to XLSB bits from OSR setting */


/*------------------------------------------------------------------------------
Initializations
------------------------------------------------------------------------------*/
raw_temp     = 0;
baro_status  = BARO_OK;
osr_bitshift = baro_configuration.temp_OSR_setting;
memset( &temp_bytes[0], 0, sizeof( temp_bytes ) );


/*------------------------------------------------------------------------------
API function implementation 
------------------------------------------------------------------------------*/

/* Read 3 consecutive temperature data registers */
baro_status = BARO_Read_Registers( BARO_REG_TEMP_DATA,
                                   &temp_bytes[0]    ,
                                   3 );

/* Check for HAL BARO error */
if ( baro_status == BARO_TIMEOUT )
	{
	return BARO_TIMEOUT;
	}

/* Combine all bytes value to 24 bit value */
raw_temp = ( ( (uint32_t) temp_bytes[2] << (8 + osr_bitshift) ) |
             ( (uint32_t) temp_bytes[1] <<      osr_bitshift  ) |
			 ( (uint32_t) temp_bytes[0]                       ) ); 

/* Export data */
*temp_ptr = raw_temp;

return BARO_OK;

} /* baro_get_temp */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_altitude                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		gets the altitude of the rocket from the sensor readouts               *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_altitude
	(
    void
	)
{
return BARO_OK;
} /* baro_get_altitude */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/