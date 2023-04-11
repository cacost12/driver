/*******************************************************************************
*
* FILE: 
* 		rs485.c
*
* DESCRIPTION: 
* 		Contains API functions to transmit data over RS485 
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
#include "rs485.h"


/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		rs485_transmit_byte                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		transmits a byte over RS485                                            *
*                                                                              *
*******************************************************************************/
RS485_STATUS rs485_transmit_byte 
	(
    uint8_t tx_byte	
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Return codes from HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
hal_status = HAL_UART_Transmit( &( RS485_HUART  ),
                                &tx_byte         , 
                                sizeof( tx_byte ), 
                                RS485_POLL_TIMEOUT );

/* Return HAL status */
if ( hal_status != HAL_OK )
	{
	return RS485_ERROR;
	}
else
	{
	return RS485_OK;
	}
} /* rs485_transmit_byte */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		rs485_transmit                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		transmits a buffer of bytes over RS485                                 *
*                                                                              *
*******************************************************************************/
RS485_STATUS rs485_transmit
	(
    void*  tx_buffer_ptr,   /* Pointer to buffer data    */
	size_t buffer_size	    /* Number of bytes in buffer */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Return codes from HAL */
uint32_t          timeout;       /* UART transmit timeout */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;
timeout    = RS485_POLL_TIMEOUT*buffer_size;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
hal_status = HAL_UART_Transmit( &( RS485_HUART ),
                                tx_buffer_ptr  , 
                                buffer_size    , 
                                timeout );

/* Return HAL status */
if ( hal_status != HAL_OK )
	{
	return RS485_ERROR;
	}
else
	{
	return RS485_OK;
	}

} /* rs485_transmit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		rs485_receieve_byte                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Receives a byte from the RS485 interface                               *
*                                                                              *
*******************************************************************************/
RS485_STATUS rs485_receive_byte 
	(
	uint8_t* p_rx_byte	
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;


/*------------------------------------------------------------------------------
 Initializations
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Receive byte */
hal_status = HAL_UART_Receive( &( RS485_HUART )  ,
                               p_rx_byte        , 
                               sizeof( uint8_t ), 
                               RS485_POLL_TIMEOUT );

/* Return HAL status */
switch ( hal_status )
	{
	case HAL_TIMEOUT:
		{
		return RS485_TIMEOUT;
		}
	case HAL_OK:
		{
		return RS485_OK;
		}
	default:
		{
		return RS485_ERROR;
        }
	}
} /* rs485_receive_byte */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		rs485_receive                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Receives data from the RS485 interface and outputs to a buffer         *
*                                                                              *
*******************************************************************************/
RS485_STATUS rs485_receive
	(
	void*  rx_buffer_ptr,   /* Pointer to output data buffer */	
	size_t rx_buffer_size   /* Number of bytes to recevie    */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Return codes from HAL */
uint32_t          timeout;       /* UART receive timeout  */


/*------------------------------------------------------------------------------
 Initialization 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;
timeout    = rx_buffer_size*RS485_POLL_TIMEOUT;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Receive data */
hal_status = HAL_UART_Receive( &( RS485_HUART ),
                               rx_buffer_ptr  , 
                               timeout        , 
                               RS485_POLL_TIMEOUT );

/* Return HAL status */
switch ( hal_status )
	{
	case HAL_TIMEOUT:
		{
		return RS485_TIMEOUT;
		}
	case HAL_OK:
		{
		return RS485_OK;
		}
	default:
		{
		return RS485_ERROR;
        }
	}
} /* rs485_receive */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/