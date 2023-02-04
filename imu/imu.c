/*******************************************************************************
*
* FILE: 
* 		imu.c
*
* DESCRIPTION: 
* 		Contains API functions to access data from the IMU MPU9250
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_A0002.h"
#include "imu.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

/* BMI270 Initialization File */
#ifdef A0002_REV2
    const uint8_t bmi270_init_file[] = {
        #include "bmi270_init_file.tbin"
    };
#endif

/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

/* Read IMU registers */
static IMU_STATUS read_imu_reg
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint8_t  num_regs  /* Number of registers */
    ); 

/* Write to a specified IMU register */
static IMU_STATUS write_imu_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    );

/* Write IMU registers */
static IMU_STATUS write_imu_regs 
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint8_t  num_regs  /* Number of registers */
    ); 

/* Read Magnetometer registers */
static IMU_STATUS read_mag_regs
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint8_t  num_regs  /* Number of registers */
    ); 

/* Write to a specified magnetometer register */
static IMU_STATUS write_mag_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    ); 

/* Write Magnetometer registers */
static IMU_STATUS write_mag_regs 
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint8_t  num_regs  /* Number of registers */
    ); 


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		imu_init                                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initialize the IMU                                                     *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_init 
	(
    IMU_CONFIG* imu_config_ptr /* IMU Configuration */ 
	)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
IMU_STATUS imu_status;          /* IMU API call return codes       */
uint8_t    imu_dev_id;          /* IMU identification code         */
uint8_t    imu_status_reg;      /* Contents of IMU status register */
uint8_t    imu_acc_conf;        /* IMU ACC_CONF Register contents  */
uint8_t    imu_gyr_conf;        /* IMU GYR_CONF Register contents  */
uint8_t    imu_sensor_data[12]; /* IMU Sensor Data                 */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
imu_status     = IMU_OK;
imu_dev_id     = 0;
imu_status_reg = 0;
imu_acc_conf   = ( imu_config_ptr -> acc_odr         ) |
                 ( imu_config_ptr -> acc_filter      ) |
                 ( imu_config_ptr -> acc_filter_mode );
imu_gyr_conf   = ( imu_config_ptr -> gyro_odr        ) |
                 ( imu_config_ptr -> gyro_filter     ) |
                 ( 1 << 7 );
memset( &imu_sensor_data[0], 0, sizeof( imu_sensor_data ) );


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Read IMU ID and verify correct ID */
imu_status = imu_get_device_id( &imu_dev_id );
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* BMI270 Initialization Sequence */
#if defined( A0002_REV2 )
    /* Disable advanced power save */
    imu_status = write_imu_reg( IMU_REG_PWR_CONF, 0x00 );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }
    HAL_Delay( 1 );

    /* Prepare Config Load */
    imu_status = write_imu_reg( IMU_REG_INIT_CTRL, 0x00 );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }
    
    /* Load the initialization data */
    imu_status = write_imu_regs( IMU_REG_INIT_DATA, 
                                 &bmi270_init_file, 
                                 sizeof( bmi270_init_file ) );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }

    /* Complete config load */
    imu_status = write_imu_reg( IMU_REG_INIT_CTRL, 0x01 );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }

    /* Check if Initialization was Successful */
    HAL_Delay( 150 );
    imu_status = read_imu_regs( IMU_REG_INTERNAL_STATUS, 
                                &imu_status_reg        ,
                                sizeof( imu_status_reg ) );
    if ( imu_status != IMU_OK || !( imu_status_reg & 0b00000001 ) )
        {
        return IMU_INIT_FAIL;
        }
#endif /* #if defined( A0002_REV2 ) */

/* Initial IMU Configuration */
#if defined( A0002_REV2 )
    /* Enable Sensors */
    imu_status = imu_write_reg( IMU_REG_PWR_CTRL, 
                                imu_config_ptr -> sensor_enable );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }

    /* Configure the Accelerometer */
    imu_status = imu_write_reg( IMU_REG_ACC_CONF, imu_acc_conf );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }
    imu_status = imu_write_reg( IMU_REG_ACC_RANGE,
                                imu_config_ptr -> acc_range );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }
    
    /* Configure the Gyroscope */
    imu_status = imu_write_reg( IMU_REG_GYR_CONF, imu_gyr_conf );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }
    imu_status = imu_write_reg( IMU_REG_GYR_RANGE, 
                                imu_config_ptr -> gyro_range );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }

    /* Disable Advanced Power Save */
    imu_status = imu_write_reg( IMU_REG_PWR_CONF, 0x02 );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }

    /* Readout sensor registers */
    imu_status = imu_read_regs( IMU_REG_DATA_8     , 
                                &imu_sensor_data[0], 
                                sizeof( imu_sensor_data ) );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL; 
        }
#endif /* #if defined( A0002_REV2 ) */

/* IMU Inititialization Successful */
return IMU_OK;
} /* imu_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_accel_xyz                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the                       * 
*       x,y,z acceleration values from the IMU                                 *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_get_accel_xyz
    (
    IMU_DATA *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t       regAccelX[2];    /* Bytes from accelerometer registers */
uint8_t       regAccelY[2];
uint8_t       regAccelZ[2];
uint16_t      accel_x_raw ;    /* Raw sensor readouts                */    
uint16_t      accel_y_raw ;  
uint16_t      accel_z_raw ; 
IMU_STATUS    imu_status_x;    /* IMU status codes                   */
IMU_STATUS    imu_status_y;
IMU_STATUS    imu_status_z;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read ACCEL_X, ACCEL_Y, ACCEL_Z high byte and low byte registers */
imu_status_x = IMU_Read_Registers( ACCEL_XOUT_H, &regAccelX[0], sizeof( regAccelX ) );
imu_status_y = IMU_Read_Registers( ACCEL_YOUT_H, &regAccelY[0], sizeof( regAccelY ) );
imu_status_z = IMU_Read_Registers( ACCEL_ZOUT_H, &regAccelZ[0], sizeof( regAccelZ ) );

/* Check for HAL IMU error */
if ( imu_status_x == IMU_TIMEOUT || 
     imu_status_y == IMU_TIMEOUT || 
     imu_status_z == IMU_TIMEOUT )
	{
	return IMU_TIMEOUT;
	}

/* Combine high byte and low byte to 16 bit data */ 
accel_x_raw    = ( (uint16_t) regAccelX[0] ) << 8  | regAccelX[1];
accel_y_raw    = ( (uint16_t) regAccelY[0] ) << 8  | regAccelY[1];
accel_z_raw    = ( (uint16_t) regAccelZ[0] ) << 8  | regAccelZ[1]; 

/* Export data to IMU sstruct */
pIMU->accel_x = accel_x_raw;
pIMU->accel_y = accel_y_raw;
pIMU->accel_z = accel_z_raw;

return IMU_OK;
} /* imu_get_accel_xyz */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_gryo_xyz                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the x,y,z gyro values     * 
*       from the IMU                                                           *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_get_gyro_xyz
    (
    IMU_DATA *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t     regGyroX[2] ;    /* Bytes from gyro registers */
uint8_t     regGyroY[2] ;
uint8_t     regGyroZ[2] ;
uint16_t    gyro_x_raw  ;    /* Raw gyro sensor readouts  */
uint16_t    gyro_y_raw  ; 
uint16_t    gyro_z_raw  ; 
IMU_STATUS  imu_status_x;    /* IMU status return codes   */
IMU_STATUS  imu_status_y;
IMU_STATUS  imu_status_z;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read GYRO_X, GYRO_Y, GYRO_Z high byte and low byte registers */
imu_status_x = IMU_Read_Registers( GYRO_XOUT_H, &regGyroX[0], sizeof( regGyroX ) );
imu_status_y = IMU_Read_Registers( GYRO_YOUT_H, &regGyroY[0], sizeof( regGyroY ) );
imu_status_z = IMU_Read_Registers( GYRO_ZOUT_H, &regGyroZ[0], sizeof( regGyroZ ) );
 
/* Check for HAL IMU error */
if (imu_status_x == IMU_TIMEOUT || 
    imu_status_y == IMU_TIMEOUT || 
    imu_status_z == IMU_TIMEOUT )
	{
	return IMU_TIMEOUT;
	}

/* Combine high byte and low byte to 16 bit data  */
gyro_x_raw = ( (uint16_t) regGyroX[0] ) << 8 | regGyroX[1];
gyro_y_raw = ( (uint16_t) regGyroY[0] ) << 8 | regGyroY[1];
gyro_z_raw = ( (uint16_t) regGyroZ[0] ) << 8 | regGyroZ[1];

/* Export Sensor Readouts */
pIMU->gyro_x = gyro_x_raw;
pIMU->gyro_y = gyro_y_raw;
pIMU->gyro_z = gyro_z_raw; 

return IMU_OK;
} /* imu_get_gyro_xyz */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_mag_xyz                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the x,y,z magnetometer    *
*       values from the IMU                                                    *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_get_mag_xyz
    (
    IMU_DATA *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t     regMagX[2]  ;    /* Magnetometer register bytes      */
uint8_t     regMagY[2]  ;
uint8_t     regMagZ[2]  ;
uint16_t    mag_x_raw   ;    /* Raw magnetometer sensor readouts */ 
uint16_t    mag_y_raw   ; 
uint16_t    mag_z_raw   ; 
IMU_STATUS  imu_status_x;    /* IMU status return codes          */
IMU_STATUS  imu_status_y;
IMU_STATUS  imu_status_z;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read MAG_X, MAG_Y, MAG_Z high byte and low byte registers */
imu_status_x = IMU_MAG_Read_Registers( MAG_XOUT_H, &regMagX[0], sizeof( regMagX ) );
imu_status_y = IMU_MAG_Read_Registers( MAG_YOUT_H, &regMagY[0], sizeof( regMagY ) );
imu_status_z = IMU_MAG_Read_Registers( MAG_ZOUT_H, &regMagZ[0], sizeof( regMagZ ) );

/* Check for HAL IMU error */
if ( imu_status_x == IMU_TIMEOUT ||
     imu_status_y == IMU_TIMEOUT || 
     imu_status_z == IMU_TIMEOUT )
	{
	return IMU_TIMEOUT;
	}

// Combine high byte and low byte to 16 bit data 
mag_x_raw  = ( (uint16_t) regMagX[0] ) << 8 | regMagX[1];
mag_y_raw  = ( (uint16_t) regMagY[0] ) << 8 | regMagY[1];
mag_z_raw  = ( (uint16_t) regMagZ[0] ) << 8 | regMagZ[1];

/* Export sensor data */
pIMU->mag_x = mag_x_raw;
pIMU->mag_y = mag_y_raw;
pIMU->mag_z = mag_z_raw;

return IMU_OK;
} /* imu_get_mag_xyz */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_device_id                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		return the device ID of the IMU to verify that the                     *
*       IMU registers are accessible                                           *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_get_device_id
    (
    uint8_t* pdevice_id 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
IMU_STATUS  imu_status;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read Device ID register */
#if defined( A0002_REV1 )
    imu_status = IMU_Read_Register( WHO_AM_I       , pdevice_id );
#elif defined( A0002_REV2 )
    imu_status = IMU_Read_Register( IMU_REG_CHIP_ID, pdevice_id );
#endif

if ( *pdevice_id != IMU_ID )
    {
    imu_status = IMU_UNRECOGNIZED_OP;
    }

return imu_status;
} /* imu_get_device_id */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		read_mag_regs                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read the specific numbers of registers at one time from magnetometer   *
*       module in the IMU                                                      *
*                                                                              *
*******************************************************************************/
static IMU_STATUS read_mag_regs 
    (
    uint8_t  reg_addr,
    uint8_t* data_ptr, 
    uint8_t  num_regs
    )
{
    
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;     /* Status return code of I2C HAL */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read I2C registers */
hal_status = HAL_I2C_Mem_Read( &( IMU_I2C )        , 
                               IMU_MAG_ADDR        , 
                               reg_addr            , 
                               I2C_MEMADD_SIZE_8BIT, 
                               data_ptr            , 
                               num_regs            , 
                               HAL_DEFAULT_TIMEOUT );

/* Return status code of I2C HAL */
if ( hal_status != HAL_OK ) 
	{
	return IMU_MAG_ERROR;
	}
else 
	{
	return IMU_OK;
	}

} /* read_mag_regs */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		read_imu_regs                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Read the specific numbers of registers at one time from acceleration   *
*       and gyroscope module in the IMU                                        *
*                                                                              *
*******************************************************************************/
static IMU_STATUS read_imu_regs 
    (
    uint8_t  reg_addr, /* Register address            */
    uint8_t* data_ptr, /* Register data               */ 
    uint8_t  num_regs  /* Number of registers to read */
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Read I2C register */
hal_status = HAL_I2C_Mem_Read( &( IMU_I2C )        , 
                               IMU_ADDR            , 
                               reg_addr            , 
                               I2C_MEMADD_SIZE_8BIT, 
                               data_ptr            , 
                               num_regs            , 
                               HAL_MAX_DELAY );

if ( hal_status != HAL_OK )
	{
	return IMU_ERROR;
	}
else
	{
	return IMU_OK;
	}
} /* read_imu_regs */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		write_imu_reg                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       write to a specified IMU register                                      *
*                                                                              *
*******************************************************************************/
static IMU_STATUS write_imu_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    ) 
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write( &( IMU_I2C )        , 
                                IMU_ADDR            , 
                                reg_addr            , 
                                I2C_MEMADD_SIZE_8BIT, 
                                &data               , 
                                sizeof( uint8_t )   , 
                                HAL_MAX_DELAY );
if ( hal_status != HAL_OK )
    {
    return IMU_I2C_ERROR;
    }
else
    {
    return IMU_OK;
    }
} /* write_imu_regs */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		write_imu_regs                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
*       write to specified IMU registers                                       *
*                                                                              *
*******************************************************************************/
static IMU_STATUS write_imu_regs 
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint8_t  num_regs  /* Number of registers */
    ) 
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write( &( IMU_I2C ), 
                                IMU_ADDR            , 
                                reg_addr            , 
                                I2C_MEMADD_SIZE_8BIT, 
                                data_ptr            , 
                                num_regs            , 
                                HAL_MAX_DELAY );
if ( hal_status != HAL_OK )
    {
    return IMU_I2C_ERROR;
    }
else
    {
    return IMU_OK;
    }
} /* write_imu_regs */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		write_mag_reg                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       write to a specified magnetometer register                             *
*                                                                              *
*******************************************************************************/
static IMU_STATUS write_mag_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    ) 
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write( &( IMU_I2C )        , 
                                IMU_MAG_ADDR        , 
                                reg_addr            , 
                                I2C_MEMADD_SIZE_8BIT, 
                                &data               , 
                                sizeof( uint8_t )   , 
                                HAL_MAX_DELAY );
if ( hal_status != HAL_OK )
    {
    return IMU_I2C_ERROR;
    }
else
    {
    return IMU_OK;
    }
} /* write_mag_regs */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		write_mag_regs                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
*       write to specified magnetometer registers                              *
*                                                                              *
*******************************************************************************/
static IMU_STATUS write_mag_regs 
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint8_t  num_regs  /* Number of registers */
    ) 
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write( &( IMU_I2C )        , 
                                IMU_MAG_ADDR        , 
                                reg_addr            , 
                                I2C_MEMADD_SIZE_8BIT, 
                                data_ptr            , 
                                num_regs            , 
                                HAL_MAX_DELAY );
if ( hal_status != HAL_OK )
    {
    return IMU_I2C_ERROR;
    }
else
    {
    return IMU_OK;
    }
} /* write_mag_regs */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/