#include "flash_writer.h"

const uint32_t __SECTOR_ADDRS[] = {
    ADDR_FLASH_SECTOR_0_BANK1, ADDR_FLASH_SECTOR_1_BANK1,
    ADDR_FLASH_SECTOR_2_BANK1, ADDR_FLASH_SECTOR_3_BANK1,
    ADDR_FLASH_SECTOR_4_BANK1, ADDR_FLASH_SECTOR_5_BANK1,
    ADDR_FLASH_SECTOR_6_BANK1, ADDR_FLASH_SECTOR_7_BANK1
};

void flash_writer_init(FlashWriter *fw, uint32_t sector) {
	if(sector>7) sector = 7;
	fw->sector = sector;
	fw->base = __SECTOR_ADDRS[sector];
	fw->ready = false;
}
bool flash_writer_ready(FlashWriter fw) {
    return fw.ready;
}

void flash_writer_open(FlashWriter * fw) {
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    HAL_FLASH_Unlock();

    /* Clear error flags */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

    /* Fill EraseInit structure */
    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Banks         = FLASH_BANK_1;
    EraseInitStruct.Sector        = fw->sector;
    EraseInitStruct.NbSectors     = 1;

    HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    fw->ready = true;
}

void flash_writer_write_int(FlashWriter fw, uint32_t index, int x) {
    uint32_t flash_word[8] = {0}; // 32 bytes = 8 x uint32_t
    flash_word[0] = (uint32_t)x;
    
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                     fw.base + 32 * index, // Địa chỉ phải chia hết cho 32
                     (uint32_t)flash_word);
}

void flash_writer_write_uint(FlashWriter fw, uint32_t index, unsigned int x) {
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
	                     fw.base + 4 * index, x);
//    FLASH_ProgramWord(fw.base + 4 * index, x);
}

void flash_writer_write_float(FlashWriter fw, uint32_t index, float x) {
    uint32_t flash_word[8] = {0};
    union {
        float f;
        uint32_t u;
    } data;
    data.f = x;
    flash_word[0] = data.u;
    
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                     fw.base + 32 * index,
                     (uint32_t)flash_word);
}

void flash_writer_close(FlashWriter * fw) {
    HAL_FLASH_Lock();
    fw->ready = false;
}
int flash_read_int(FlashWriter fw, uint32_t index) {
    return *(int*) (__SECTOR_ADDRS[fw.sector] + 4 * index);
}

uint32_t flash_read_uint(FlashWriter fw, uint32_t index) {
    return *(uint32_t*) (__SECTOR_ADDRS[fw.sector] + 4 * index);
}

float flash_read_float(FlashWriter fw, uint32_t index) {
    return *(float*) (__SECTOR_ADDRS[fw.sector] + 4 * index);
}


