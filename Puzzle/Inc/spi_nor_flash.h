#ifndef __SPI_NOR_FLASH__
#define __SPI_NOR_FLASH__

#include "stm32f0xx_hal.h"
#include "n25q256_commands.h"

typedef enum{
	RESULT_OK,
	RESULT_ERROR
	
}result_t;

void init_spi_flash(void);

void write_enable(uint8_t enable);

result_t read_status_register( uint8_t* reg_val );
result_t read_random(uint32_t address, uint32_t size, uint8_t* dest_buf);
result_t erase_sector(uint32_t sector);
result_t erase_subsector(uint32_t subsector);
// page      = 256bytes
// subsector = 4k
// sector    = 64k
// program limitation is 1 page
// In order to preserve existing data in same subsector as destination page,
// subsector data must be cached
// Also, beware of writes that cross a page or subsector boundary
result_t program(uint32_t address, uint16_t size, uint8_t* source);

// wen [0|1]
// rdsts
// rd u32 u32
// erasesec u32
// erasesub u32
// program u32 u16 u8+

#endif /* __SPI_NOR_FLASH__ */ 
