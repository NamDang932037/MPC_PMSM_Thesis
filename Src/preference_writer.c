#include "preference_writer.h"
#include "flash_writer.h"
#include "user_config.h"
/*
PreferenceWriter::PreferenceWriter(uint32_t sector) {
    writer = new FlashWriter(sector);
    __sector = sector;
    __ready = false;
}
*/

void preference_writer_init(PreferenceWriter * pr, uint32_t sector){
	flash_writer_init(&pr->fw, sector);
	pr->sector = sector;
}


void preference_writer_open(PreferenceWriter * pr) {
    flash_writer_open(&pr->fw);
    pr->ready = true;
}

bool  preference_writer_ready(PreferenceWriter pr) {
    return pr.ready;
}

void preference_writer_write_int(int x, int index) {
    __int_reg[index] = x;
}

void preference_writer_write_float(float x, int index) {
    __float_reg[index] = x;
}

void preference_writer_flush(PreferenceWriter * pr) {
    uint32_t flash_word[8] = {0};
    int i;
    
    // Ghi các giá trị integer
    for (i = 0; i < 256; i += 8) {
        for (int j = 0; j < 8; j++) {
            flash_word[j] = (uint32_t)__int_reg[i + j];
        }
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                         pr->fw.base + 32 * i/8,
                         (uint32_t)flash_word);
    }
    
    // Ghi các giá trị float
    for (i = 0; i < 64; i += 8) {
        for (int j = 0; j < 8; j++) {
            union {
                float f;
                uint32_t u;
            } data;
            data.f = __float_reg[i + j];
            flash_word[j] = data.u;
        }
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                         pr->fw.base + 32 * (32 + i/8),
                         (uint32_t)flash_word);
    }
    if (HAL_FLASH_GetError() != 0) {
        // Xử lý lỗi
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
        return;
    }
    pr->ready = false;
}

void preference_writer_load(PreferenceWriter pr) {
    int offs;
    for (offs = 0; offs < 256; offs++) {
        __int_reg[offs] = flash_read_int(pr.fw, offs);
    }
    for(; offs < 320; offs++) {
        __float_reg[offs - 256] = flash_read_float(pr.fw, offs);
    }
}

void preference_writer_close(PreferenceWriter *pr) {
    pr->ready = false;
    flash_writer_close(&pr->fw);
}


