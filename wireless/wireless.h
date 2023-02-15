/*******************************************************************************
*
* FILE: 
* 		wireless.h
*
* DESCRIPTION: 
* 		Contains API functions to transmit data wirelessly using the XBee and 
*       and LoRa modules 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef WIRELESS_H
#define WIRELESS_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Function return codes */
typedef enum RF_STATUS
	{
	RF_OK = 0,
    RF_FAIL  ,
	RF_TIMEOUT
	} RF_STATUS;

/* Wireless Module Codes */
typedef enum WIRELESS_MOD_CODES
	{
	LORA = 0,
    XBEE
	} WIRELESS_MOD_CODES;


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* HAL Settings*/
#define RF_TIMEOUT		( 1 )


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/


/* transmits a byte wirelessly using the xbee module */
RF_STATUS rf_xbee_transmit_byte 
	(
    uint8_t tx_byte	
	);


/* Receives a byte from the xbee module */
RF_STATUS rf_xbee_receive_byte 
	(
	uint8_t* p_rx_byte	
	);

#ifdef __cplusplus
}
#endif

#endif /* WIRELESS_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/