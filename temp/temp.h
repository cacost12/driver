/*******************************************************************************
*
* FILE: 
* 		temp.h
*
* DESCRIPTION: 
* 		Contains API functions for reading data from the engine's thermocouple
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TEMP_H 
#define TEMP_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Thermocouple I2C Address */
#define THERMO_I2C_ADDR                   0b11000000

/* Register Identification Codes  */
#define THERMO_HOT_JUNC_TEMP_REG_ID       0b00000000
#define THERMO_TEMP_DELTA_REG_ID          0b00000001
#define THERMO_COLD_JUNC_TEMP_REG_ID      0b00000010
#define THERMO_RAW_ADC_REG_ID             0b00000011
#define THERMO_STATUS_REG_ID              0b00000100
#define THERMO_SENSOR_CONFIG_REG_ID       0b00000101
#define THERMO_DEV_CONFIG_REG_ID          0b00000110
#define THERMO_ALERT1_CONFIG_REG_ID       0b00001000
#define THERMO_ALERT2_CONFIG_REG_ID       0b00001001
#define THERMO_ALERT3_CONFIG_REG_ID       0b00001010
#define THERMO_ALERT4_CONFIG_REG_ID       0b00001011
#define THERMO_ALERT1_HYS_REG_ID          0b00001100
#define THERMO_ALERT2_HYS_REG_ID          0b00001101
#define THERMO_ALERT3_HYS_REG_ID          0b00001110
#define THERMO_ALERT4_HYS_REG_ID          0b00001111
#define THERMO_ALERT1_TEMP_LIM_REG_ID     0b00010000
#define THERMO_ALERT2_TEMP_LIM_REG_ID     0b00010001
#define THERMO_ALERT3_TEMP_LIM_REG_ID     0b00010010
#define THERMO_ALERT4_TEMP_LIM_REG_ID     0b00010011
#define THERMO_DEV_ID_REG_ID              0b00100000

/* Thermocouple factory device ID */
#define THERMO_DEV_ID                     0x41


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Thermocouple return codes */
typedef enum THERMO_STATUS 
    {
	THERMO_OK = 0   ,
    THERMO_I2C_ERROR,
    THERMO_FAIL 
    } THERMO_STATUS;


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/* Get the device id of the thermcouple */
THERMO_STATUS temp_get_device_id 
	(
    uint8_t* device_id_ptr
    );

#endif /* THERMO_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/