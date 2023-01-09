/*******************************************************************************
*
* FILE: 
* 		flash.h
*
* DESCRIPTION: 
* 		Contains API functions for writing and reading data from the engine 
*       controller's flash 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FLASH_H 
#define FLASH_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/

/* Standard includes */
#include <stdbool.h>

/* Project includes */
#include "sensor.h"


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Flash subcommand bitmasks */
#define FLASH_SUBCMD_OP_BITMASK     0b11100000 
#define FLASH_NBYTES_BITMASK        0b00011111

/* Write protection ON/OFF States */
#define FLASH_WP_READ_ONLY          false
#define FLASH_WP_WRITE_ENABLED      true

/* Flash Chip operation codes from datasheet */
#define FLASH_OP_HW_READ	        0x03
#define FLASH_OP_HW_READ_HS         0x0B
#define FLASH_OP_HW_4K_ERASE        0x20
#define FLASH_OP_HW_32K_ERASE       0x52
#define FLASH_OP_HW_64K_ERASE       0xD8
#define FLASH_OP_HW_FULL_ERASE      0x60
#define FLASH_OP_HW_BYTE_PROGRAM    0x02
#define FLASH_OP_HW_AAI_PROGRAM     0xAD
#define FLASH_OP_HW_RDSR            0x05
#define FLASH_OP_HW_EWSR            0x50
#define FLASH_OP_HW_WRSR            0x01
#define FLASH_OP_HW_WREN            0x06
#define FLASH_OP_HW_WRDI            0x04
#define FLASH_OP_HW_RDID            0x90
#define FLASH_OP_HW_JEDEC_ID        0x9F
#define FLASH_OP_HW_EBSY            0x70
#define FLASH_OP_HW_DBSY            0x80

/* Maximum Flash address */
#define FLASH_MAX_ADDR              0x07FFFF

/* Reset state of flash register */
#define FLASH_REG_RESET_VAL         0b00111000

/* Timeouts */
#ifndef SDR_DEBUG
	#define HAL_FLASH_TIMEOUT       100
#else
	#define HAL_FLASH_TIMEOUT       0xFFFFFFFF
#endif

/* Flash busy/ready boolean codes */
#define FLASH_BUSY                  true
#define FLASH_READY                 false

/* Status register bitmasks */
#define FLASH_BUSY_BITMASK          0b00000001


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Block Protection Codes from datasheet (pg.7) */
typedef enum FLASH_BPL_BITS
	{
	FLASH_BPL_NONE       = 0b11000011, /* No write protection               */
	FLASH_BPL_UPPER_1_16 = 0b11000111, /* 0x070000 - 0x07FFFF protected     */
	FLASH_BPL_UPPER_1_8  = 0b11001011, /* 0x060000 - 0x07FFFF protected     */
	FLASH_BPL_UPPER_1_4  = 0b11001111, /* 0x040000 - 0x07FFFF protected     */
	FLASH_BPL_UPPER_1_2  = 0b11010011, /* 0x000000 - 0x07FFFF protected (?) */
	FLASH_BPL_ALL        = 0b11011100  /* 0x000000 - 0x07FFFF protected     */
	} FLASH_BPL_BITS;

/* Write protect setting for BPL bits in status register */
typedef enum FLASH_BPL_WP
	{
	FLASH_BPL_READ_WRITE = 0b0000000,
	FLASH_BPL_READ_ONLY  = 0b1000000
	} FLASH_BPL_WP;

/* Buffer object with handle to a user-definable buffer. FLASH_BUFFER struct 
   should be filled with info about the buffer size, a reference to the buffer,
   and an SPI handle */
typedef struct _FLASH_BUFFER_TAG {
	
	/* Number of bytes in buffer */
	uint8_t        num_bytes;

	/* Base flash address for read/write operations */
	uint32_t       address;

	/* Buffer reference */
	uint8_t*       pbuffer;

    /* Write protection state */
    bool           write_protected;

	/* BPL settings for block protection */
	FLASH_BPL_BITS bpl_bits;

	/* Write protection setting of flash BPL bits */
	FLASH_BPL_WP   bpl_write_protect;

	/* Contents of status register */
	uint8_t        status_register;

} HFLASH_BUFFER; 

/* Flash subcommand codes */
typedef enum FLASH_SUBCMD_CODES 
	{
	FLASH_SUBCMD_READ = 0,
	FLASH_SUBCMD_ENABLE  ,
	FLASH_SUBCMD_DISABLE ,
	FLASH_SUBCMD_WRITE   ,
	FLASH_SUBCMD_ERASE   ,
	FLASH_SUBCMD_STATUS  ,
	FLASH_SUBCMD_EXTRACT ,
	FLASH_SUBCMD_HS_READ ,
	FLASH_SUBCMD_4K_ERASE
	} FLASH_SUBCMD_CODE;

/* Flash return value codes */
typedef enum FLASH_STATUS 
	{
	FLASH_OK = 0              ,
	FLASH_FAIL                ,
	FLASH_UNSUPPORTED_OP      ,
	FLASH_UNRECOGNIZED_OP     ,
	FLASH_TIMEOUT             ,
	FLASH_WRITE_PROTECTED     ,
	FLASH_WRITE_TIMEOUT       ,
	FLASH_USB_ERROR           ,
	FLASH_SPI_ERROR           ,
	FLASH_CANNOT_WRITE_ENABLE ,
	FLASH_INVALID_INPUT       ,
	FLASH_WRITE_ERROR         ,
	FLASH_ERROR_MISSING_DATA  , 
	FLASH_CANNOT_EXIT_AAI     ,
	FLASH_CANNOT_WRITE_DISABLE,
	FLASH_INIT_FAIL
	} FLASH_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Executes a flash subcommand based on user input from the sdec terminal */
FLASH_STATUS flash_cmd_execute
	(
    uint8_t        flash_subcommand,
    HFLASH_BUFFER* pflash_handle   
    );

/* Initializes the flash chip */
FLASH_STATUS flash_init 
	(
	HFLASH_BUFFER* pflash_handle  
	);

/* Read the status register of the flash chip */
FLASH_STATUS flash_get_status
	(
	HFLASH_BUFFER* pflash_handle
    );

/* Write to the status register of the flash chip */
FLASH_STATUS flash_set_status
	(
	uint8_t        flash_status
    );

/* Check if the flash chip is ready for write operations */
bool flash_is_flash_busy
	(
	void
	);

/* Enable writing to the external flash chip */
void flash_write_enable 
    (
	void
    );

/* Disable writing to the external flash chip */
void flash_write_disable
    (
    void  
    );

/* Write bytes from a flash buffer to the external flash */
FLASH_STATUS flash_write
    (
	HFLASH_BUFFER* pflash_handle
    );

/* Write a byte to the external flash */
FLASH_STATUS flash_write_byte 
    (
	HFLASH_BUFFER* pflash_handle,
	uint8_t        byte
    );

/* Read a specified number of bytes using a flash buffer */
FLASH_STATUS flash_read
    (
	HFLASH_BUFFER* pflash_handle,
    uint32_t       num_bytes
    );

/* Erase the entire flash chip */
FLASH_STATUS flash_erase
    (
    HFLASH_BUFFER* pflash_handle	
    );


/* Block erase 4 bit of data from Flash chip */
FLASH_STATUS flash_4k_erase
    (
    HFLASH_BUFFER* pflash_handle,
    uint8_t        num_bytes
    );

#endif /* FLASH_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
