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
#ifndef TELEMETRY_SYSTEM_DATA_STRUCT_DEF_H
#define TELEMETRY_SYSTEM_DATA_STRUCT_DEF_H
#include <stdint.h>
#include <stdbool.h>

typedef struct __attribute__((__packed__)){
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t msec;
} TimeStamp_t;

typedef struct __attribute__((__packed__)){
    uint8_t type_msg;
    uint8_t msg_len;
    uint8_t *msg;
} INF_Data_t;

typedef struct __attribute__((__packed__)){
    int8_t RSSI;
    int8_t SNR;
    uint16_t system_status;
    uint8_t cpu_load;
} MON_Data_t;

typedef struct __attribute__((__packed__)){
    float vbat;
    float vbat_backup;
    float vbat_rtc;
    float temperature;
    uint8_t power_status;
} POW_Data_t;

typedef struct __attribute__((__packed__)) {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} GPS_time_t;

typedef struct __attribute__((__packed__)) {
    uint8_t day;
    uint8_t month;
    uint8_t year;
} GPS_date_t;

typedef struct __attribute__((__packed__)) {
    TimeStamp_t time_stamp;
    float latitude;
    float longitude;
    float gps_speed;
    float hdop;
    float pdop;
    float vdop;
    uint8_t sats;
    uint8_t fix_quality;
    uint8_t fix_type;
    GPS_time_t time;
    GPS_date_t date;

} GPS_Data_t;


typedef struct {
    uint8_t sync;
    uint8_t type;
    uint8_t msg_id;
    uint8_t len;
    uint32_t *payload;
    uint8_t crc8;
} Send_Message_t;


#endif //TELEMETRY_SYSTEM_DATA_STRUCT_DEF_H
