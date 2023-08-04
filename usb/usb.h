/*******************************************************************************
*
* FILE: 
* 		usb.h
*
* DESCRIPTION: 
* 		Contains API functions to transmit data over USB 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef USB_H
#define USB_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Defines and Typdefs 
------------------------------------------------------------------------------*/

/* Function return codes */
typedef enum USB_STATUS
	{
	USB_OK = 0,
    USB_ERROR ,
	USB_TIMEOUT
	} USB_STATUS;

/* USB Detect States */
#define USB_ACTIVE    true
#define USB_INACTIVE  false
typedef bool USB_STATE;

/* Timeouts */
#ifndef ZAV_DEBUG 
	#define USB_DEFAULT_TIMEOUT    ( 5 ) 
#else
	#define USB_DEFAULT_TIMEOUT    ( 0xFFFFFFFF )
#endif


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* transmits bytes over USB */
USB_STATUS usb_transmit 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size, /* Size of transmit data */ 
	uint32_t timeout       /* UART timeout          */
	);

/* Receives bytes from the USB port */
USB_STATUS usb_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	);

/* Checks for an active USB connection */
USB_STATE usb_detect
	(
	void
	);


#ifdef __cplusplus
}
#endif

#endif /* USB_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/