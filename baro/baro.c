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
 Internal function prototypes 
------------------------------------------------------------------------------*/

/* Read from the baro's registers at a specified address  */
static BARO_STATUS baro_read_regs
	(
	uint8_t  reg_addr, /* In:  Register address            */
	uint8_t  num_regs, /* In:  Number of registers to read */
	uint8_t* pData     /* Out: Register contents           */
	);


/* Write to one of the baro's registers at a specified address */
static BARO_STATUS baro_write_reg
	(
	uint8_t  reg_addr, /* In: Register address            */
	uint8_t  data      /* In: Register contents           */
	);


/*------------------------------------------------------------------------------
 API Functions 
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
uint8_t           baro_err_reg;   /* Contents of error register        */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
baro_status    = BARO_OK;
baro_device_id = 0;
baro_err_reg   = 0xFF;


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

/* Check the Baro error register */
baro_status = baro_read_regs( BARO_REG_ERR_REG      , 
                              sizeof( baro_err_reg ), 
							  &baro_err_reg );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}
else if ( baro_err_reg )
	{
	return BARO_ERROR;
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
BARO_STATUS   baro_status; /* Baro API call return codes                  */
uint8_t       pwr_ctrl;    /* Contents of PWR_CTRL register               */
uint8_t       osr;         /* Contents of OSR register                    */
uint8_t       odr;         /* Contents of ODR register                    */
uint8_t       iir;         /* Contents of CONFIG register (IIR Filter)    */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
baro_status = BARO_OK;
pwr_ctrl    = 0;
osr         = 0;
odr         = 0;
iir         = 0;


/*------------------------------------------------------------------------------
 Extract Baro settings from config struct  
------------------------------------------------------------------------------*/

/* Set register contents */
pwr_ctrl |= config_ptr -> enable;                /* Enable sensors           */
pwr_ctrl |= config_ptr -> mode << 4;             /* Set operating mode       */
osr      |= config_ptr -> press_OSR_setting;     /* Pressure oversampling    */
osr      |= config_ptr -> temp_OSR_setting << 3; /* Temperature oversampling */
odr      |= config_ptr -> ODR_setting;           /* Sampling Frequency       */
iir      |= config_ptr -> IIR_setting;           /* IIR Filter Selection     */

/* Set global baro configuration */
baro_configuration.enable            = config_ptr -> enable;
baro_configuration.mode              = config_ptr -> mode;
baro_configuration.press_OSR_setting = config_ptr -> press_OSR_setting;
baro_configuration.temp_OSR_setting  = config_ptr -> temp_OSR_setting;
baro_configuration.ODR_setting       = config_ptr -> ODR_setting;
baro_configuration.IIR_setting       = config_ptr -> IIR_setting;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Write to the PWR_CTRL register -> Operating mode and enable sensors */
baro_status = baro_write_reg( BARO_REG_PWR_CTRL, pwr_ctrl );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Write to the OSR register -> set the oversampling rate */
baro_status = baro_write_reg( BARO_REG_OSR, osr );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Write to the ODR register -> set the sampling frequency */
baro_status = baro_write_reg( BARO_REG_ODR, odr );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Write to the CONFIG register -> Configure the IIR filter */
baro_status = baro_write_reg( BARO_REG_CONFIG, iir );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
	}

/* Configuration complete */
return BARO_OK;

} /* baro_config */


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
   	uint8_t* baro_id_ptr /* Out: Baro device id */ 
	)
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
BARO_STATUS baro_status; /* Status codes returned from baro API calls */


/*------------------------------------------------------------------------------
 API Function implementation 
------------------------------------------------------------------------------*/

/* Read baro register with I2C */
baro_status = baro_read_regs( BARO_REG_CHIP_ID, 
                              sizeof( uint8_t ), 
							  baro_id_ptr );
return baro_status;
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
    uint32_t* pressure_ptr  /* Out: Baro pressure */
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
baro_status = baro_read_regs( BARO_REG_PRESS_DATA, 
                              sizeof( pressure_bytes ), 
							  &pressure_bytes[0] );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
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
baro_status = baro_read_regs( BARO_REG_TEMP_DATA  , 
                              sizeof( temp_bytes ), 
							  &temp_bytes[0] );
if ( baro_status != BARO_OK )
	{
	return BARO_I2C_ERROR;
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


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       baro_read_regs                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Read from the baro's registers at a specified address                  *
*                                                                              *
*******************************************************************************/
static BARO_STATUS baro_read_regs
	(
	uint8_t  reg_addr, /* In:  Register address            */
	uint8_t  num_regs, /* In:  Number of registers to read */
	uint8_t* pData     /* Out: Register contents           */
	)
{   
/*------------------------------------------------------------------------------
Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status; /* HAL API Return codes */


/*------------------------------------------------------------------------------
API function implementation 
------------------------------------------------------------------------------*/

/* Read I2C register*/
hal_status = HAL_I2C_Mem_Read( &( BARO_I2C )       ,
				               BARO_I2C_ADDR       ,
				               reg_addr            ,
				               I2C_MEMADD_SIZE_8BIT,
				               pData               ,
				               num_regs            , 
				               HAL_DEFAULT_TIMEOUT );
if ( hal_status != HAL_OK )
	{
	return BARO_I2C_ERROR;
	}
else
	{
	return BARO_OK;
	}

} /* baro_read_reg */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       baro_write_reg                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Write to one of the baro's registers at a specified address            *
*                                                                              *
*******************************************************************************/
static BARO_STATUS baro_write_reg
	(
	uint8_t  reg_addr, /* In: Register address            */
	uint8_t  data      /* In: Register contents           */
	)
{   
/*------------------------------------------------------------------------------
Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status; /* HAL API Return codes */


/*------------------------------------------------------------------------------
API function implementation 
------------------------------------------------------------------------------*/

/* Write to register with I2C */
hal_status = HAL_I2C_Mem_Write( &( BARO_I2C )       ,
				                BARO_I2C_ADDR       ,
				                reg_addr            ,
				                I2C_MEMADD_SIZE_8BIT,
				                &data               ,
				                sizeof( uint8_t )   , 
				                HAL_DEFAULT_TIMEOUT );
if ( hal_status != HAL_OK )
	{
	return BARO_I2C_ERROR;
	}
else
	{
	return BARO_OK;
	}

} /* baro_read_reg */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/