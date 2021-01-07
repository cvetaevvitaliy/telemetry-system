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
#include "gps_service.h"
#include "stm32f4xx_hal.h"
#include "hardware_init.h"
#include "cmsis_os.h"
#include "minmea.h"
#include "cli.h"

#define GPS_UART_BUFFER     160

extern UART_HandleTypeDef huart1;

void _gps_parser(uint8_t *data);


typedef struct {
    uint8_t gps_buffer[GPS_UART_BUFFER];
    uint16_t in_ptr;
    bool find_header;
    bool data_ready;
} GPS_State_t;


typedef struct {
    int hours;
    int minutes;
    int seconds;
} GPS_time_t ;

typedef struct {
    int day;
    int month;
    int year;
} GPS_date_t ;

typedef struct {
    float latitude;
    float longitude;
    float gps_speed;
    uint8_t sats;
    uint8_t fix_quality;
    uint8_t fix_type;
    float hdop;
    float pdop;
    float vdop;
    GPS_time_t time;
    GPS_date_t date;

} GPS_Data_t;


static uint8_t ch;
FAST_RAM GPS_State_t GPS_State = {0};
GPS_Data_t GPS_Data = {0};

struct minmea_sentence_rmc frame_rmc;


void gps_service_init(void)
{
    MX_DMA_Init();
    MX_USART1_UART_Init();
    HAL_UART_Receive_DMA(&huart1,&ch, 1);
}

void gps_service_put_char_handle(void)
{
    if (ch == '$') {
        GPS_State.find_header = true;
    }

    if (ch == '\n' && GPS_State.find_header == true)
    {
        uint8_t  buffer[81] = {0};
        memcpy(buffer, GPS_State.gps_buffer, GPS_State.in_ptr);

        GPS_State.data_ready = false;
        _gps_parser(buffer);
        GPS_State.data_ready = true;

        GPS_State.in_ptr = 0;
        GPS_State.find_header = false;
        return;
    }
    else
    {
        GPS_State.gps_buffer[GPS_State.in_ptr] = ch;
        GPS_State.in_ptr++;
        if (GPS_State.in_ptr > GPS_UART_BUFFER)
            GPS_State.in_ptr = 0;
    }
}



void _gps_parser(uint8_t *data)
{

    switch (minmea_sentence_id(data, false)) {

        case MINMEA_INVALID: ULOG_ERROR("MINMEA_INVALID\n");
            break;

        case MINMEA_UNKNOWN:
            break;

        case MINMEA_SENTENCE_RMC: {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, (char *) data)) {
                GPS_Data.latitude = (float)minmea_tocoord(&frame.latitude);
                GPS_Data.longitude = (float)minmea_tocoord(&frame.longitude);
                GPS_Data.gps_speed = (float)minmea_tofloat(&frame.speed);

                GPS_Data.time.hours = frame.time.hours;
                GPS_Data.time.minutes = frame.time.minutes;
                GPS_Data.time.seconds = frame.time.seconds;

                GPS_Data.date.year = frame.date.year;
                GPS_Data.date.month = frame.date.month;
                GPS_Data.date.day = frame.date.day;

            }
        }
            break;

        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, (char *) data)) {
                GPS_Data.fix_quality = (uint8_t)frame.fix_quality;
                GPS_Data.sats = (uint8_t)frame.satellites_tracked;
            }
        }
            break;

        case MINMEA_SENTENCE_GSA: {
            struct minmea_sentence_gsa frame;
            if (minmea_parse_gsa(&frame, (char *) data)) {
                GPS_Data.fix_type = (uint8_t)frame.fix_type;
            }
        }

            break;

        case MINMEA_SENTENCE_GLL:
            break;

        case MINMEA_SENTENCE_GST: {
//            struct minmea_sentence_gst frame;
//            if (minmea_parse_gst(&frame, (char *) data)) {
//                ULOG_DEBUG("$GNGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\n",
//                         frame.latitude_error_deviation.value, frame.latitude_error_deviation.scale,
//                         frame.longitude_error_deviation.value, frame.longitude_error_deviation.scale,
//                         frame.altitude_error_deviation.value, frame.altitude_error_deviation.scale);
//                ULOG_DEBUG("$GNGST: fixed point latitude,longitude and altitude error deviation"
//                         " scaled to one decimal place: (%d,%d,%d)\n",
//                         minmea_rescale(&frame.latitude_error_deviation, 10),
//                         minmea_rescale(&frame.longitude_error_deviation, 10),
//                         minmea_rescale(&frame.altitude_error_deviation, 10));
//                ULOG_DEBUG("$GNGST: floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
//                         minmea_tofloat(&frame.latitude_error_deviation),
//                         minmea_tofloat(&frame.longitude_error_deviation),
//                         minmea_tofloat(&frame.altitude_error_deviation));
//            }
        }
            break;

        case MINMEA_SENTENCE_GSV:
            break;

        case MINMEA_SENTENCE_VTG: {
//            struct minmea_sentence_vtg frame;
//            if (minmea_parse_vtg(&frame, (char *) data)) {
//                ULOG_DEBUG("$GNVTG: speed knots = %.2f\n",
//                         minmea_tofloat(&frame.speed_knots));
//                ULOG_DEBUG("$GNVTG: speed kph = %.2f\n",
//                         minmea_tofloat(&frame.speed_kph));
//            }
        }

            break;

        case MINMEA_SENTENCE_ZDA: {
//            struct minmea_sentence_zda frame;
//            if (minmea_parse_zda(&frame, (char *) data)) {
//                ULOG_DEBUG("$GNZDA: Time: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
//                         frame.time.hours,
//                         frame.time.minutes,
//                         frame.time.seconds,
//                         frame.date.day,
//                         frame.date.month,
//                         frame.date.year,
//                         frame.hour_offset,
//                         frame.minute_offset);
//            }
        }
            break;
    }

}


void gps_service_execute(void)
{

    if (GPS_State.data_ready == true){
        ULOG_DEBUG("Latitude: %f\n", GPS_Data.latitude);
        ULOG_DEBUG("Longitude: %f\n", GPS_Data.longitude);
        ULOG_DEBUG("GPS speed: %f\n", GPS_Data.gps_speed);
        ULOG_DEBUG("GPS fix quality: %d\n", GPS_Data.fix_quality);
        ULOG_DEBUG("GPS fix type: %d\n", GPS_Data.fix_type);
        ULOG_DEBUG("GPS sats: %d\n", GPS_Data.sats);
        ULOG_DEBUG("GPS Time: %d:%d:%d\n", GPS_Data.time.hours, GPS_Data.time.minutes, GPS_Data.time.seconds);
        ULOG_DEBUG("GPS Date: %02d.%02d.%d\n", GPS_Data.date.day, GPS_Data.date.month, GPS_Data.date.year);

        GPS_State.data_ready = false;

    }

}
