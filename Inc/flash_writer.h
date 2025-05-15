#ifndef INC_FLASH_WRITER_H_
#define INC_FLASH_WRITER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "main.h"
extern const uint32_t __SECTOR_ADDRS[];
/* Base address của các Flash sectors cho STM32H747 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kb */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kb */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kb */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kb */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kb */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kb */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kb */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kb */



typedef struct {
	bool ready;
	uint32_t base;
	uint32_t sector;
}FlashWriter;

void flash_writer_init(FlashWriter *fw, uint32_t sector);
bool flash_writer_ready(FlashWriter fw);
void flash_writer_open(FlashWriter *fw);
void flash_writer_write_int(FlashWriter fw, uint32_t index, int x);
void flash_writer_write_uint(FlashWriter fw, uint32_t index, unsigned int x);
void flash_writer_write_float(FlashWriter fw, uint32_t index, float x);
void flash_writer_close(FlashWriter *fw);
int flash_read_int(FlashWriter fw, uint32_t index);
uint32_t flash_read_uint(FlashWriter fw, uint32_t index);
float flash_read_float(FlashWriter fw, uint32_t index);

#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH_WRITER_H_ */
