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
	THERMO_OK = 0         ,
    THERMO_I2C_ERROR      ,
    THERMO_UNRECOGNIZED_ID,
    THERMO_FAIL 
    } THERMO_STATUS;

/* Thermocouple types */
typedef enum THERMO_TYPE
    {
    THERMO_TYPE_K = 0,
    THERMO_TYPE_J    ,
    THERMO_TYPE_T    ,
    THERMO_TYPE_N    ,
    THERMO_TYPE_S    ,
    THERMO_TYPE_E    ,
    THERMO_TYPE_B    ,
    THERMO_TYPE_R
    } THERMO_TYPE;

/* Filter settings */
typedef enum THERMO_FILTER_COEFF
    {
    THERMO_FILTER_OFF = 0,
    THERMO_FILTER_MIN    ,
    THERMO_FILTER_N_2    ,
    THERMO_FILTER_N_3    ,
    THERMO_FILTER_MID    ,
    THERMO_FILTER_N_5    ,
    THERMO_FILTER_N_6    ,
    THERMO_FILTER_MAX
    } THERMO_FILTER_COEFF;

/* ADC Measurement Resolution */
typedef enum THERMO_ADC_RES
    {
    THERMO_18BIT_ADC = 0,
    THERMO_16BIT_ADC    ,
    THERMO_14BIT_ADC    ,
    THERMO_12BIT_ADC 
    } THERMO_ADC_RES;

/* Cold junction sensor resolution */
typedef enum THERMO_COLD_JUNC_RES
    {
    THERMO_COLD_JUNC_MIN_RES = 0, /* 0.0625 deg C */
    THERMO_COLD_JUNC_MAX_RES      /* 0.25 deg C */
    } THERMO_COLD_JUNC_RES;

/* Thermocouple configuration/state */
typedef struct THERMO_CONFIG
    {
    THERMO_TYPE          type;                 /* Thermocouple type (ex K, T) */
    THERMO_FILTER_COEFF  filter_coeff;         /* Filter setting              */
    THERMO_ADC_RES       adc_resolution;       /* ADC Resolution, 12-18 bits  */
    THERMO_COLD_JUNC_RES cold_junc_resolution; /* Cold junction sensor 
                                                  resolution                  */
    uint8_t              status;               /* Status register contents    */
    } THERMO_CONFIG;


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/* Initialize the thermocouple cold junction compensation chip */
THERMO_STATUS temp_init
    (
    THERMO_CONFIG* thermo_config_ptr /* Pointer to thermocouple settings */
    );

/* Read the status of the thermocouple cold junction compensation chip */
THERMO_STATUS temp_get_status
    (
    uint8_t* status_ptr /* Pointer to output data */
    );

/* Get the device id of the thermcouple */
THERMO_STATUS temp_get_device_id 
	(
    uint8_t* device_id_ptr
    );

#endif /* THERMO_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/