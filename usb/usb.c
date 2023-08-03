/*******************************************************************************
*
* FILE: 
* 		usb.c
*
* DESCRIPTION: 
* 		Contains API functions to transmit data over USB 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes  
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( BASE_FLIGHT_COMPUTER )
	#include "zav_pin_defines_A0001.h"
#elif defined( FULL_FLIGHT_COMPUTER )
	#include "zav_pin_defines_A0002.h"
#elif defined( LEGACY_FLIGHT_COMPUTER )
	#include "zav_pin_defines_A0003.h"
#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
	#include "zav_pin_defines_A0004.h"
#else
	#error "No USB compatible device specified in Makefile"
#endif


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "usb.h"


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		usb_transmit                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		transmits a specified number of bytes over USB                         *
*                                                                              *
*******************************************************************************/
USB_STATUS usb_transmit 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size, /* Size of transmit data */ 
	uint32_t timeout       /* UART timeout          */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef usb_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
usb_status = HAL_UART_Transmit( &( usb_huart ),
                                tx_data_ptr   , 
                                tx_data_size  , 
                                timeout );

/* Return HAL status */
if ( usb_status != HAL_OK )
	{
	return USB_ERROR;
	}
else
	{
	return USB_OK;
	}

} /* usb_transmit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		usb_receieve                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Receives bytes from the USB port                                       *
*                                                                              *
*******************************************************************************/
USB_STATUS usb_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef usb_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
usb_status = HAL_UART_Receive( &( usb_huart ),
                               rx_data_ptr   , 
                               rx_data_size  , 
                               timeout );

/* Return HAL status */
switch ( usb_status )
	{
	case HAL_TIMEOUT:
		{
		return USB_TIMEOUT;
		}
	case HAL_OK:
		{
		return USB_OK;
		}
	default:
		{
		return USB_ERROR;
        }
	}

} /* usb_receive */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		usb_detect                                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Detect a USB connection by checking the power on the USB 5V line       *
*                                                                              *
*******************************************************************************/
USB_STATE usb_detect
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
uint8_t usb_detect_pinstate;    /* USB detect state, return value from HAL    */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
usb_detect_pinstate = 0;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Read voltage on usb detect pin */
usb_detect_pinstate = HAL_GPIO_ReadPin( USB_DETECT_GPIO_PORT, USB_DETECT_PIN );

/* Set return value */
if ( usb_detect_pinstate == 0 )
	{
	return USB_INACTIVE;
	}
else
	{
	return USB_ACTIVE;
	}
} /* usb_detect */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/