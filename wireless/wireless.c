/*******************************************************************************
*
* FILE: 
* 		wireless.c
*
* DESCRIPTION: 
* 		Contains API functions to transmit data wirelessly using the XBee and 
*       and LoRa modules 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#if defined( GROUND_STATION )
	#include "sdr_pin_defines_A0005.h"
#elif defined( ENGINE_CONTROLLER )
	#include "sdr_pin_defines_L0002.h"
#endif

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "wireless.h"


/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		rf_xbee_transmit_byte                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		transmits a byte wirelessly using the xbee module                      *
*                                                                              *
*******************************************************************************/
RF_STATUS rf_xbee_transmit_byte 
	(
    uint8_t tx_byte	
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef xbee_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
xbee_status = HAL_UART_Transmit( &( XBEE_HUART )  ,
                                 &tx_byte         , 
                                 sizeof( tx_byte ), 
                                 RF_TIMEOUT );

/* Return HAL status */
if ( xbee_status != HAL_OK )
	{
	return xbee_status;
	}
else
	{
	return RF_OK;
	}

} /* rf_xbee_transmit_byte */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		rf_xbee_receieve_byte                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Receives a byte from the xbee module                                   *
*                                                                              *
*******************************************************************************/
RF_STATUS rf_xbee_receive_byte 
	(
	uint8_t* p_rx_byte	
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef xbee_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
xbee_status = HAL_UART_Receive( &( XBEE_HUART )  ,
                                p_rx_byte        , 
                                sizeof( uint8_t ), 
                                RF_TIMEOUT );

/* Return HAL status */
switch ( xbee_status )
	{
	case HAL_TIMEOUT:
		{
		return RF_TIMEOUT;
		break;
		}
	case HAL_OK:
		{
		return RF_OK;
		break;
		}
	default:
		{
		return RF_FAIL;
		break;
        }
	}
} /* rf_xbee_transmit_byte */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/