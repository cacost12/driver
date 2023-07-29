/*******************************************************************************
*
* FILE: 
* 		flash.c
*
* DESCRIPTION: 
* 		Contains API functions for writing and reading data from the engine 
*       controller's flash 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( BASE_FLIGHT_COMPUTER        )
	#include "zav_pin_defines_A0001.h"
#elif defined( FULL_FLIGHT_COMPUTER        )
	#include "zav_pin_defines_A0002.h"
#elif defined( LEGACY_FLIGHT_COMPUTER      )
	#include "zav_pin_defines_A0003.h"
#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
	#include "zav_pin_defines_A0004.h"
#else
	#error "No flash module compatible device specified in Makefile"
#endif 


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "flash.h"
#include "led.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
static bool write_enabled = false;


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

/* Converts a flash memory address in uint32_t format to a byte array */
static void address_to_bytes
	(
	uint32_t address,
	uint8_t* address_bytes
	);

/* Converts a flash memory address in byte format to uint32_t format */
static inline uint32_t bytes_to_address 
	(
	uint8_t address_bytes[3]
	);

/* Set the flash chip write enable latch to arm flash write commands */
static FLASH_STATUS enable_write_latch
    (
    void 
    );

/* Reset the flash chip write enable latch to disable flash write commands */
static FLASH_STATUS disable_write_latch
    (
    void 
    );

/* Writes a single byte to the external flash */
static FLASH_STATUS flash_write_byte 
    (
	uint8_t  byte,
	uint32_t address
    );


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_init                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Initializes the flash chip                                             *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_init 
	(
	FLASH_CONFIG flash_config
	)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
FLASH_STATUS flash_status;    /* Flash API function return codes        */
uint8_t      initial_status;  /* Status of flash chip on initial read   */
uint8_t      updated_status;  /* Desired status register contents       */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status   = FLASH_OK;
initial_status = FLASH_STATUS_REG_RESET_VAL;
updated_status = FLASH_STATUS_REG_RESET_VAL;


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/

/* Check for invalid BPL setting */
if ( flash_config.bpl_bits > FLASH_BPL_ALL )
	{
	return FLASH_INVALID_INPUT; 
	}

/* Determine the desired status register contents */
updated_status &= flash_config.bpl_bits;
updated_status |= flash_config.bpl_write_protect;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Check the flash chip status register to confirm chip can be reached */
flash_status = flash_get_status( &initial_status );
if ( ( flash_status != FLASH_OK ) ||  ( initial_status == 0xFF ) )
	{
	return FLASH_CANNOT_REACH_DEVICE;
	}

/* Reset write enable latch, latch enabled on each flash write call */
if ( reset_write_latch() != FLASH_OK )
	{
	return FLASH_INIT_FAIL;
	}

/* Set global write protection */
if ( flash_config.write_protected )
	{
	flash_write_disable();
	}
else
	{
	flash_write_enable();
	}

/* Set the bpl bits in the flash chip */
if ( flash_set_status( updated_status ) != FLASH_OK )
	{
	return FLASH_INIT_FAIL;
	}

/* Confirm Status register contents */
while( flash_is_flash_busy() == FLASH_BUSY ){}
flash_status = flash_get_status( &initial_status );
if ( initial_status != updated_status )
	{
	return FLASH_INIT_FAIL;
	}
else
	{
	return flash_status;
	}

} /* flash_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_get_status                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Read the status register of the flash chip                             *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_get_status
	(
	uint8_t* flash_status_ptr /* Out: flash status register contents */
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status[2];  /* Status codes returned by SPI HAL         */
uint8_t           flash_opcode;   /* Data to be transmitted over SPI          */


/*------------------------------------------------------------------------------
 Initialiazations 
------------------------------------------------------------------------------*/
flash_opcode = FLASH_OP_HW_RDSR;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Flash SPI Sequence: drive SS Low -> send RDSR opcode -> read register ->
                       drive SS high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status[0] = HAL_SPI_Transmit( &( flash_hspi )        ,
                                  &flash_opcode         ,
                                  sizeof( flash_opcode ),
                                  FLASH_DEFAULT_TIMEOUT );
hal_status[1] = HAL_SPI_Receive( &( flash_hspi )  ,
                                 flash_status_ptr ,
                                 sizeof( uint8_t ),
							     FLASH_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Return flash status code */
if ( hal_status[0] != HAL_OK || hal_status[1] != HAL_OK )
	{
	return FLASH_SPI_ERROR;
	}
else
	{
	return FLASH_OK;
	}

} /* flash_get_status */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_set_status                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Writes to the status register                                          *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_set_status
	(
	uint8_t flash_status	/* In: desired status register contents */
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t hal_status   [3];   /* Status code returned by hal spi functions      */
uint8_t flash_opcodes[2];   /* Flash instruction cycle bytes                  */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_opcodes[0] = FLASH_OP_HW_EWSR;
flash_opcodes[1] = FLASH_OP_HW_WRSR;


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/
if ( !( write_enabled ) )
	{
	return FLASH_WRITE_PROTECTED;
	}


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Set status SPI sequence: drive SS low -> Send EWSR command -> drive SS high
                        ->  drive SS low -> send WRSR command -> send desired 
						    status register contents -> drive SS high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status[0] = HAL_SPI_Transmit( &( flash_hspi )   ,
                                  &flash_opcodes[0],
                                  sizeof( uint8_t ),
                                  HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status[1] = HAL_SPI_Transmit( &( flash_hspi )   ,
                                  &flash_opcodes[1],
                                  sizeof( uint8_t ),
							      HAL_DEFAULT_TIMEOUT );
hal_status[2] = HAL_SPI_Transmit( &( flash_hspi )        ,
                                  &flash_status         ,
                                  sizeof( flash_status ),
							      HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Return flash status */
if ( hal_status[0] != HAL_OK || hal_status[1] != HAL_OK || hal_status[2] != HAL_OK )
	{
	return FLASH_SPI_ERROR;
	}
else
	{
	return FLASH_OK;
	}

} /* flash_set_status */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		flash_is_flash_busy                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Polls the flash status register to see if the chip is busy. False      *
*       boolean return value indicates the chip is ready for write operations  *
*                                                                              *
*******************************************************************************/
bool flash_is_flash_busy
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
FLASH_STATUS  flash_status; /* Flash API return codes      */
uint8_t       status_reg;   /* Contents of status register */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status = FLASH_OK;
status_reg   = 0xFF;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Read status register */
flash_status = flash_get_status( &status_reg );
if      ( flash_status != FLASH_OK        )
	{
	return FLASH_BUSY;
	}
else if ( status_reg & FLASH_BUSY_BITMASK )
	{
	return FLASH_BUSY;
	}
else
	{
	return FLASH_READY;
	}

} /* flash_is_flash_busy */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write_enable                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Enable writing to the external flash chip by configuring the global    *
*       write protection variable                                              *
*                                                                              *
*******************************************************************************/
void flash_write_enable
	(
	void
	)
{
write_enabled = true;
} /* flash_write_enable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write_disable                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Disable writing to the external flash chip                             *
*                                                                              *
*******************************************************************************/
void flash_write_disable
    (
    void 
    )
{
write_enabled = false;
} /* flash_write_disable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       writes bytes from a flash buffer to the external flash                 *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_write 
    (
	FLASH_BUFFER flash_buffer 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status[3];    /* Status codes returned by HAL           */
FLASH_STATUS      flash_status;     /* Status codes returned by flash API     */
uint8_t           flash_opcode;     /* Opcode for flash instructions          */
uint32_t          timeout;          /* Timeout for flash write calls          */
uint32_t          timeout_ctr;      /* Counter to trigger timeout             */
uint8_t           address_bytes[3]; /* Flash memory address in byte form      */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_opcode  = FLASH_OP_HW_AAI_PROGRAM; 
flash_status  = FLASH_OK;
timeout       = flash_buffer.buffer_size; /* 1 ms/byte timeout */
timeout_ctr   = 0;
address_to_bytes( flash_buffer.address, &address_bytes[0] );


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/

/* Check if write_enabled */
if( !( write_enabled ) )
	{
	return FLASH_WRITE_PROTECTED;
	}

/* Check for 1/0 byte edge case */
if      ( flash_buffer.buffer_size == 1 )
	{
	return flash_write_byte( ( *flash_buffer.buffer_ptr ), flash_buffer.address );
	}
else if ( flash_buffer.buffer_size == 0 )
	{
	return FLASH_ERROR_MISSING_DATA;
	}


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Arm the byte program command */
flash_status = enable_write_latch();
if ( flash_status != FLASH_OK )
	{
	return FLASH_CANNOT_WRITE_ENABLE;
	}

/* SPI sequence: drive SS low -> transmit AAI command ->  transmit address -> 
                 transmit first two bytes -> drive SS high  
		         For each two bytes: drive SS low -> transmit AAI command -> 
				 transmit two bytes -> drive SS high
				 If odd: transmit last byte */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status[0] = HAL_SPI_Transmit( &( flash_hspi )       ,
							      &flash_opcode         ,
							      sizeof( flash_opcode ),
							      FLASH_DEFAULT_TIMEOUT );
hal_status[1] = HAL_SPI_Transmit( &( flash_hspi )        ,
							      &address_bytes[0]      ,
							      sizeof( address_bytes ),
							      FLASH_DEFAULT_TIMEOUT );
hal_status[2] = HAL_SPI_Transmit( &( flash_hspi )        , 
                                  flash_buffer.buffer_ptr,
								  2                      , 
								  FLASH_DEFAULT_TIMEOUT );						
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Check for SPI errors */
if ( hal_status[0] != HAL_OK || hal_status[1] != HAL_OK || hal_status[2] != HAL_OK )
	{
	return FLASH_SPI_ERROR;
	}

/* Transmit remaining data */
for ( int i = 2; i < flash_buffer.buffer_size; i += 2 )
	{
	/* Setup buffer pointer  */
	flash_buffer.buffer_ptr += 2;

	/* Wait for flash to be ready */
	while( flash_is_flash_busy() == FLASH_BUSY )
		{
		if ( timeout_ctr >= timeout )
			{
			return FLASH_TIMEOUT;
			}
		else
			{
			timeout_ctr++;
			HAL_Delay( 1 );
			}
		}

	/* SPI Transmission */	
	HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
	hal_status[0] = HAL_SPI_Transmit( &( flash_hspi )     ,
									&flash_opcode         ,
									sizeof( flash_opcode ),
									FLASH_DEFAULT_TIMEOUT );
	hal_status[1] = HAL_SPI_Transmit( &( flash_hspi )      , 
									flash_buffer.buffer_ptr,
									2                      , 
									FLASH_DEFAULT_TIMEOUT );						
	HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

	/* Check for errors */
	if ( hal_status[0] != HAL_OK || hal_status[1] != HAL_OK )
		{
		return FLASH_WRITE_ERROR;
		}
	} 

/* Wait for AAI to complete */
while ( flash_is_flash_busy() == FLASH_BUSY ){}

/* Terminate AAI Programming */
flash_status = write_disable();
if ( flash_status != FLASH_OK )
	{
	return FLASH_CANNOT_EXIT_AAI;
	}

/* Wait for WRDI to complete */
while ( flash_is_flash_busy() == FLASH_BUSY ){}

/* Transmit final byte if num_bytes is odd */
if ( ( flash_buffer.buffer_size ) %2 == 1 )
	{
	flash_buffer.buffer_ptr += 1;
	return flash_write_byte( (*flash_buffer.buffer_ptr), flash_buffer.address );
	}
else
	{
	/* Flash write successful */
	return FLASH_OK;
	}

} /* flash_write */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_read                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       reads a specified number of bytes using a flash buffer                 *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_read
    (
	FLASH_BUFFER flash_buffer
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status[4];    /* Return codes from hal spi functions    */
uint8_t           flash_opcode;     /* Data to be transmitted over SPI        */
uint8_t           address_bytes[3]; /* Flash address in byte format           */
uint8_t           dummy_byte;       /* Dummy byte needed for high speed read  */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_opcode = FLASH_OP_HW_READ_HS;
dummy_byte   = 0;
address_to_bytes( flash_buffer.address, &address_bytes[0] );


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* SPI sequence: drive SS low -> transmit READ_HS command -> transmit address 
              -> transmit dummy byte -> Receive N bytes -> drive SS high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status[0] = HAL_SPI_Transmit( &( flash_hspi )       ,
							      &flash_opcode         ,
							      sizeof( flash_opcode ),
							      FLASH_DEFAULT_TIMEOUT );
hal_status[1] = HAL_SPI_Transmit( &( flash_hspi )        ,
							      &( address_bytes[0] )  ,
							      sizeof( address_bytes ),
							      FLASH_DEFAULT_TIMEOUT );
hal_status[2] = HAL_SPI_Transmit( &( flash_hspi )     ,
							      &( dummy_byte )     ,
							      sizeof( dummy_byte ),
							      FLASH_DEFAULT_TIMEOUT );
hal_status[3] = HAL_SPI_Receive( &( flash_hspi )         ,
                                 flash_buffer.buffer_ptr ,
								 flash_buffer.buffer_size,
								 FLASH_DEFAULT_TIMEOUT*flash_buffer.buffer_size );
/* Drive chip enable line high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Check for SPI errors */
if ( hal_status[0] != HAL_OK || hal_status[1] != HAL_OK || 
	 hal_status[2] != HAL_OK || hal_status[3] != HAL_OK )
	{   
	return FLASH_SPI_ERROR;
	}
return FLASH_OK;

} /* flash_read */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		flash_erase                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       erases the entire flash chip                                           *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_erase
    (
    void 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
int8_t       hal_status;    /* Status code return by hal spi functions        */
uint8_t      flash_opcode;  /* Data to be transmitted over SPI                */
FLASH_STATUS flash_status;


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_opcode = FLASH_OP_HW_FULL_ERASE;


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/

/* Check if write_enabled */
if( !( write_enabled ) )
    {
    return FLASH_WRITE_PROTECTED;
    }


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Enable writing to flash */
flash_status = write_enable();

/* SPI sequence: drive SS low -> transmit FULL_ERASE command -> drive SS high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status = HAL_SPI_Transmit( &( flash_hspi )        ,
							   &flash_opcode         ,
							   sizeof( flash_opcode ),
							   HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Return status code */
if      ( hal_status   != HAL_OK   )
	{
	return FLASH_SPI_ERROR;
	}
else if ( flash_status != FLASH_OK )
	{
	return FLASH_FAIL;
	}
else
	{
	return FLASH_OK;
	}

} /* flash_erase */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		flash_block_erase                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Erase a block of flash                                                 *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_block_erase
	(
	FLASH_BLOCK      flash_block_num, /* Block of flash to erase */
	FLASH_BLOCK_SIZE size             /* Size of block           */
	)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
int8_t       hal_status[2];       /* Status code return by hal spi functions  */
uint8_t      flash_opcode;        /* Data to be transmitted over SPI          */
FLASH_STATUS flash_status;        /* Return codes from flash API              */
uint32_t     flash_addr;          /* Address of block to erase                */
uint8_t      flash_addr_bytes[3]; /* Address of block to erase in byte form   */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status = FLASH_OK;


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/

/* Determine block setting */
switch( size )
	{
	case FLASH_BLOCK_4K:
		{
		flash_opcode = FLASH_OP_HW_4K_ERASE;
		flash_addr   = flash_block_num*(0x1000);
		address_to_bytes( flash_addr, &flash_addr_bytes[0] );
		break;
		}

	case FLASH_BLOCK_32K:
		{
		flash_opcode = FLASH_OP_HW_32K_ERASE;
		flash_addr   = flash_block_num*(0x8000);
		address_to_bytes( flash_addr, &flash_addr_bytes[0] );
		break;
		}

	case FLASH_BLOCK_64K:
		{
		flash_opcode = FLASH_OP_HW_64K_ERASE;
		flash_addr   = flash_block_num*(0x10000);
		address_to_bytes( flash_addr, &flash_addr_bytes[0] );

		/* Error check */
		if ( flash_block_num >= FLASH_BLOCK_8 )
			{
			return FLASH_ADDR_OUT_OF_BOUNDS;
			}
		break;
		}
	}

/* Check if write_enabled */
if( !( write_enabled ) )
    {
    return FLASH_WRITE_PROTECTED;
    }


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Enable writing to flash */
flash_status = write_enable();

/* Sector erase sequence */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status[0] = HAL_SPI_Transmit( &( flash_hspi )        ,
							      &flash_opcode         ,
							      sizeof( flash_opcode ),
							      HAL_DEFAULT_TIMEOUT );
hal_status[1] = HAL_SPI_Transmit( &( flash_hspi )            , 
                                  &flash_addr_bytes[0]      ,
								  sizeof( flash_addr_bytes ), 
								  HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Return status code */
if      ( hal_status[0] != HAL_OK || hal_status[1] != HAL_OK )
	{
	return FLASH_SPI_ERROR;
	}
else if ( flash_status != FLASH_OK )
	{
	return FLASH_FAIL;
	}
else
	{
	return FLASH_OK;
	}
} /* flash_block_erase */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		address_to_bytes                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Converts a flash memory address in uint32_t format to a byte array     *
*                                                                              *
*******************************************************************************/
static void address_to_bytes
	(
	uint32_t address,
	uint8_t* address_bytes
	)
{
address_bytes[0] = (address >> 16) & 0xFF;
address_bytes[1] = (address >> 8 ) & 0xFF;
address_bytes[2] =  address        & 0xFF;
} /* address_to_bytes */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		bytes_to_address                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Converts a flash memory address in byte format to uint32_t format      *
*                                                                              *
*******************************************************************************/
static inline uint32_t bytes_to_address 
	(
	uint8_t address_bytes[3]
	)
{
return ( (uint32_t) address_bytes[0] << 16 ) |
	   ( (uint32_t) address_bytes[1] << 8  ) |
	   ( (uint32_t) address_bytes[2] << 0  );
} /* bytes_to_address */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		enable_write_latch                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set the flash chip write enable latch to arm flash write commands      *
*                                                                              *
*******************************************************************************/
static FLASH_STATUS enable_write_latch 
    (
    void 
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
uint8_t hal_status;        /* Status code return by hal spi functions         */
uint8_t flash_opcode;      /* Flash operation/instruction cyle byte           */


/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
hal_status    = HAL_OK;
flash_opcode  = FLASH_OP_HW_WREN;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Send the write enable instruction to the flash over SPI */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_RESET );
hal_status = HAL_SPI_Transmit( &( flash_hspi )        ,
                               &flash_opcode         ,
                               sizeof( flash_opcode ),
                               HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_SET );

/* Set write enabled bit in flash buffer handle */
if ( hal_status != HAL_OK )
	{
	return FLASH_SPI_ERROR;
    }
else
	{
	return FLASH_OK;
    }

} /* enable_write_latch */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		reset_write_latch                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Reset the flash chip write enable latch to disable flash write         *
*       commands                                                               *
*                                                                              *
*******************************************************************************/
static FLASH_STATUS reset_write_latch 
    (
    void 
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
uint8_t hal_status;        /* Status code return by hal spi functions         */
uint8_t flash_opcode;      /* Flash operation/instruction cyle byte           */


/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
hal_status    = HAL_OK;
flash_opcode  = FLASH_OP_HW_WRDI;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Send the write enable instruction to the flash over SPI */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_RESET );
hal_status = HAL_SPI_Transmit( &( flash_hspi )        ,
                               &flash_opcode         ,
                               sizeof( flash_opcode ),
                               HAL_DEFAULT_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT,
                   FLASH_SS_PIN      ,
                   GPIO_PIN_SET );

/* Return results of SPI HAL calls */
if ( hal_status != HAL_OK )
	{
	return FLASH_SPI_ERROR;
    }
else
	{
	return FLASH_OK;
    }

} /* reset_write_latch */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		flash_write_byte                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       writes a single byte to the external flash                             *
*                                                                              *
*******************************************************************************/
static FLASH_STATUS flash_write_byte 
    (
	uint8_t   byte,     /* In: Byte to write to flash */
	uint32_t  address   /* In: Address to write to    */
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status[3];    /* Status code return by hal functions    */
FLASH_STATUS      flash_status;     /* Status code returned by flash API      */
uint8_t           flash_opcode;     /* Data to be transmitted over SPI        */
uint8_t           address_bytes[3]; /* Flash memory address in byte form      */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status  = FLASH_OK;
flash_opcode  = FLASH_OP_HW_BYTE_PROGRAM;
address_to_bytes( address, &address_bytes[0] );


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/

/* Check if write_enabled */
if( !( write_enabled ) )
	{
	return FLASH_WRITE_PROTECTED;
	}


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Arm the flash write command */
flash_status = enable_write_latch();
if ( flash_status != FLASH_OK )
	{
	return FLASH_CANNOT_WRITE_ENABLE;
	}

/* Flash write SPI sequence: drive SS pin low -> transmit BYTE_PROGRAM command 
                          -> transmit address bytes -> transmit program byte 
						  -> drive SS pin high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );
hal_status[0] = HAL_SPI_Transmit( &( flash_hspi )       ,
							      &flash_opcode        ,
							      sizeof( flash_opcode ),
							      HAL_DEFAULT_TIMEOUT );
hal_status[1] = HAL_SPI_Transmit( &( flash_hspi )   ,
							      &address[0]      ,
							      sizeof( address ),
							      HAL_DEFAULT_TIMEOUT );
hal_status[2] = HAL_SPI_Transmit( &( flash_hspi ),
							     &byte          ,
							     sizeof( byte ) ,
							     HAL_FLASH_TIMEOUT );
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Return flash status */
if ( hal_status[0] != HAL_OK || hal_status[1] != HAL_OK || hal_status[2] != HAL_OK )
	{
	return FLASH_SPI_ERROR;
	}
else
	{
	return FLASH_OK;
	}

} /* flash_write_byte */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/