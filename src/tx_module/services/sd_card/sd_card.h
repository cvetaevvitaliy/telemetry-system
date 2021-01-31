/*
 * This file is part of "Telemetry system" project.
 *
 * "Telemetry system" are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * "Telemetry system" are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
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
