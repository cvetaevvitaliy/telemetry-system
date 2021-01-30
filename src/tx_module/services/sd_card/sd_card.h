#ifndef TELEMETRY_SYSTEM_SD_CARD_H
#define TELEMETRY_SYSTEM_SD_CARD_H
#include <stdbool.h>
#include "fatfs.h"
#include "cmsis_os.h"

typedef struct {
    FATFS   fatfs;
    FATFS   *sd_fs;
    FIL     fd;
    DIR     dir;
    FILINFO finfo;
    uint32_t mem_free;      // in KB
    uint32_t mem_total;     // in KB
    BYTE work[_MAX_SS];
    bool insert;
    bool initialized;
    bool mount_fs;
    bool sd_not_fmt;
    bool log_enable;
    osMutexId lock_file;

} SD_Card_State_t;



void sd_card_init(void);

void sd_card_deinit(void);

FRESULT sd_card_format(void);

void sd_card_info(void);

void sd_card_insert_event(void);

void sd_card_enable_pin_detect(bool state);

SD_Card_State_t *sd_card_get_state(void);

#endif //TELEMETRY_SYSTEM_SD_CARD_H
