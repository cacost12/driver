/*******************************************************************************
*
* FILE: 
* 		temp.c
*
* DESCRIPTION: 
* 		Contains API functions for reading data from the engine's thermocouple 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_L0002.h"
#include "temp.h"

/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

/* Write to a thermocouple register */
static THERMO_STATUS write_reg
    (
    uint8_t reg_id,  /* Thermocouple register id  */
    uint8_t reg_data /* Data to write to register */
    );

/* Read a specified number of bytes from a thermocouple register */
static THERMO_STATUS read_reg
    (
    uint8_t  reg_id,       /* Thermocouple register id  */
    uint8_t* reg_data_ptr, /* Pointer to output         */
    uint8_t  num_bytes     /* Number of bytes to read   */
    );


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		temp_init                                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initialize the thermocouple cold junction compensation chip            *
*                                                                              *
*******************************************************************************/
THERMO_STATUS temp_init
    (
    THERMO_CONFIG* thermo_config_ptr /* Pointer to thermocouple settings */
    )
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
THERMO_STATUS thermo_status;     /* Return codes from temp functions       */
uint8_t       sensor_config_reg; /* Contents of thermocouple sensor config 
                                    register                               */
uint8_t       dev_config_reg;    /* Contents of thermocouple device config 
                                    register                               */
uint8_t       device_id;         /* Chip identification code               */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
thermo_status     = THERMO_OK;
sensor_config_reg = 0;
dev_config_reg    = 0;
device_id         = 0;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Shift configuration bits into correct position */
sensor_config_reg |= thermo_config_ptr -> type                 << 4;
sensor_config_reg |= thermo_config_ptr -> filter_coeff;
dev_config_reg    |= thermo_config_ptr -> cold_junc_resolution << 7;
dev_config_reg    |= thermo_config_ptr -> adc_resolution       << 5;

/* Check that device can be reached */
thermo_status = temp_get_device_id( &device_id );
if      ( thermo_status != THERMO_OK )
    {
    return thermo_status;
    }
else if ( device_id != THERMO_DEV_ID )
    {
    return THERMO_UNRECOGNIZED_ID;
    }

/* Write to the configuration registers */
thermo_status = write_reg( THERMO_SENSOR_CONFIG_REG_ID, sensor_config_reg );
if ( thermo_status != THERMO_OK )
    {
    return thermo_status;
    }
thermo_status = write_reg( THERMO_DEV_CONFIG_REG_ID, dev_config_reg );
if ( thermo_status != THERMO_OK )
    {
    return thermo_status;
    }

/* Get the device status */
thermo_status = temp_get_status( &( thermo_config_ptr -> status ) );
if ( thermo_status != THERMO_OK )
    {
    return thermo_status;
    }

return THERMO_OK;
} /* temp_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		temp_get_status                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Read the status of the thermocouple cold junction compensation chip    *
*                                                                              *
*******************************************************************************/
THERMO_STATUS temp_get_status
    (
    uint8_t* status_ptr /* Pointer to output data */
    )
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
THERMO_STATUS thermo_status; /* Return codes from temp functions */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
thermo_status = THERMO_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
thermo_status = read_reg( THERMO_STATUS_REG_ID, 
                          status_ptr          ,
                          sizeof( uint8_t ) );
if ( thermo_status != THERMO_OK )
    {
    return thermo_status;
    }
else
    {
    return THERMO_OK;
    }

} /* temp_get_status */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		temp_get_device_id                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Get the device id of the thermcouple                                   *
*                                                                              *
*******************************************************************************/
THERMO_STATUS temp_get_device_id 
	(
    uint8_t* device_id_ptr
    )
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
THERMO_STATUS thermo_status; /* Return codes from temp API calls */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
thermo_status = THERMO_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
thermo_status = read_reg( THERMO_DEV_ID_REG_ID, 
                          device_id_ptr       ,
                          sizeof( uint8_t ) );
if ( thermo_status != THERMO_OK )
    {
    return thermo_status;
    }
else
    {
    return THERMO_OK;
    }

} /* temp_get_device_id */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		write_reg                                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Write to a thermocouple register                                       *
*                                                                              *
*******************************************************************************/
static THERMO_STATUS write_reg
    (
    uint8_t reg_id,  /* Thermocouple register id  */
    uint8_t reg_data /* Data to write to register */
    )
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status; /* Return codes from I2C HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Load the thermocouple pointer register */
hal_status = HAL_I2C_Master_Transmit( &( THERMO_I2C ) , 
                                      THERMO_I2C_ADDR , 
                                      &reg_id         ,
                                      sizeof( reg_id ), 
                                      HAL_DEFAULT_TIMEOUT );
if ( hal_status != HAL_OK )
    {
    return THERMO_I2C_ERROR;
    }

/* Send the data */
hal_status = HAL_I2C_Master_Transmit( &( THERMO_I2C )   , 
                                      THERMO_I2C_ADDR   , 
                                      &reg_data         ,
                                      sizeof( reg_data ), 
                                      HAL_DEFAULT_TIMEOUT );
if ( hal_status != HAL_OK )
    {
    return THERMO_I2C_ERROR;
    }
else
    {
    return THERMO_OK;
    }

} /* write_reg */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		read_reg                                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Read a specified number of bytes from a thermocouple register          *
*                                                                              *
*******************************************************************************/
static THERMO_STATUS read_reg
    (
    uint8_t  reg_id,       /* Thermocouple register id  */
    uint8_t* reg_data_ptr, /* Pointer to output         */
    uint8_t  num_bytes     /* Number of bytes to read   */
    )
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status; /* Return codes from I2C HAL     */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Load the thermocouple pointer register */
hal_status = HAL_I2C_Master_Transmit( &( THERMO_I2C ) , 
                                     THERMO_I2C_ADDR , 
                                     &reg_id         ,
                                     sizeof( reg_id ), 
                                     HAL_DEFAULT_TIMEOUT );
if ( hal_status != HAL_OK )
    {
    return THERMO_I2C_ERROR;
    }

/* Load the data */
hal_status = HAL_I2C_Master_Receive( &( THERMO_I2C )   , 
                                     THERMO_I2C_ADDR   , 
                                     reg_data_ptr      ,
                                     num_bytes         , 
                                     HAL_DEFAULT_TIMEOUT );
if ( hal_status != HAL_OK )
    {
    return THERMO_I2C_ERROR;
    }
else
    {
    return THERMO_OK;
    }
return THERMO_OK;
} /* read_reg */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/