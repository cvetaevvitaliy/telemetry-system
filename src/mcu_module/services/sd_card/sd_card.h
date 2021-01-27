#ifndef TELEMETRY_SYSTEM_SD_CARD_H
#define TELEMETRY_SYSTEM_SD_CARD_H
#include <stdbool.h>
#include "fatfs.h"

typedef struct {
    FATFS   fatfs;
    FIL     fd;
    DIR     dir;
    FILINFO finfo;
    BYTE work[_MAX_SS];
    bool insert;
    bool initialized;

} SD_Card_State_t;



void sd_card_init(void);

void sd_card_deinit(void);

void sd_card_format(void);

void sd_card_info(void);

void sd_card_insert_event(void);

void sd_card_enable_pin_detect(bool state);

SD_Card_State_t *sd_card_get_state(void);

#endif //TELEMETRY_SYSTEM_SD_CARD_H
