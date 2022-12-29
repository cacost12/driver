/*******************************************************************************
*
* FILE: 
* 		baro.h
*
* DESCRIPTION: 
* 		Contains API functions for the barometric pressure sensor
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BARO_H 
#define BARO_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* I2C Device Params */
#define BARO_I2C_ADDR	    ( 0x76 << 1 )	/* 1110110 -> 0x76 */


/* Barometric Pressure Sensor register addresses */
#define BARO_REG_CHIP_ID        ( 0x00 )
#define BARO_REG_PRESS_DATA     ( 0x04 )  
#define BARO_REG_TEMP_DATA      ( 0x07 ) 
#define BARO_REG_PWR_CTRL       ( 0x1B )
#define BARO_REG_OSR            ( 0x1C ) 
#define BARO_REG_ODR            ( 0x1D )

/* Baro device id */
#define BARO_DEVICE_ID          ( 0x60 )


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Return codes for API functions */
typedef enum _BARO_STATUS
	{
	BARO_OK                     ,
	BARO_FAIL                   , 
	BARO_TIMEOUT                ,
	BARO_UNRECOGNIZED_HAL_STATUS,
	BARO_UNSUPPORTED_CONFIG     ,
	BARO_UNRECOGNIZED_CHIP_ID   ,
	BARO_I2C_ERROR
	} BARO_STATUS;

/* Sensor enable encodings */
typedef enum _BARO_SENSOR_ENABLES
	{
	BARO_DISABLED           = 0,
    BARO_PRESS_ENABLED      = 1,
	BARO_TEMP_ENABLED       = 2,
	BARO_PRESS_TEMP_ENABLED = 3
	} BARO_SENSOR_ENABLES;

/* Operating mode of baro sensor */
typedef enum _BARO_MODE
	{
	BARO_SLEEP_MODE  = 0,
	BARO_FORCED_MODE = 1,
	BARO_NORMAL_MODE = 3
	} BARO_MODE;

/* Pressure Sensor Oversampling Settings */
typedef enum _BARO_PRESS_OSR_SETTING
	{
	BARO_PRESS_OSR_X1 = 0,
    BARO_PRESS_OSR_X2    ,
	BARO_PRESS_OSR_X4    ,
	BARO_PRESS_OSR_X8    ,
	BARO_PRESS_OSR_X16   ,
	BARO_PRESS_OSR_X32
	} BARO_PRESS_OSR_SETTING;

/* Temperature Sensor Oversampling Settings */
typedef enum _BARO_TEMP_OSR_SETTING
	{
	BARO_TEMP_OSR_X1 = 0,
    BARO_TEMP_OSR_X2    ,
	BARO_TEMP_OSR_X4    ,
	BARO_TEMP_OSR_X8    ,
	BARO_TEMP_OSR_X16   ,
	BARO_TEMP_OSR_X32
	} BARO_TEMP_OSR_SETTING;

/* Sample Frequency settings */
typedef enum _BARO_ODR_SETTING
	{
	BARO_ODR_200HZ = 0,
	BARO_ODR_100HZ    ,
	BARO_ODR_50HZ     ,
	BARO_ODR_25HZ     ,
	BARO_ODR_25_2HZ   ,
	BARO_ODR_25_4HZ   ,
	BARO_ODR_25_8HZ   ,
	BARO_ODR_25_16HZ  ,
	BARO_ODR_25_32HZ
	} BARO_ODR_SETTING;

/* Baro sensor configuration settings struct */
typedef struct _BARO_CONFIG
	{
	/* Sensor enables */
	BARO_SENSOR_ENABLES enable;

	/* Operating mode */
	BARO_MODE mode;

	/* Pressure Oversampling setting  */
	BARO_PRESS_OSR_SETTING press_OSR_setting;

	/* Temperature Oversampling setting */
	BARO_TEMP_OSR_SETTING temp_OSR_setting;

	/* Sampling frequency */
	BARO_ODR_SETTING ODR_setting;

	} BARO_CONFIG;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Intialize the barometric pressure sensor */
BARO_STATUS baro_init
	(
	BARO_CONFIG* config_ptr
	);
	
/* Configure/intialize the barometric pressure sensor */
BARO_STATUS baro_config
	(
	BARO_CONFIG* config_ptr 
	);

/* read one register for BARO */
BARO_STATUS BARO_Read_Register
	(
	uint8_t reg_addr,
	uint8_t *pData
	);

/* read multiple registers for BARO */
BARO_STATUS BARO_Read_Registers
	(
	uint8_t reg_addr,
	uint8_t *pData,
	uint8_t num_registers
	);

/* write a register to BARO */
BARO_STATUS BARO_Write_Register
	(
	uint8_t reg_addr,
	uint8_t *pData
	);


/* verifies sensor can be accessed */
BARO_STATUS baro_get_device_id
	(
   	uint8_t* baro_id 
	);


/* gets pressure data from sensor */
BARO_STATUS baro_get_pressure
	(
    uint32_t* pressure_ptr 
	);

/* gets temp data from sensor */
BARO_STATUS baro_get_temp
	(
    uint32_t* temp_ptr 
	);

/* converts pressure and temp data into altitude --> do research on formula */
BARO_STATUS baro_get_altitude
	(
    void
	);


#ifdef __cplusplus
}
#endif
#endif /* BARO_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/