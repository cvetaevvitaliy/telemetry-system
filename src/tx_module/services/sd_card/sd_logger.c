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
#include "sd_logger.h"
#include "cli.h"
#include "cmsis_os.h"


void sd_logger_sys(ulog_level_t level, char *msg)
{
    SD_Card_State_t *sd = sd_card_get_state();
    static FIL fd_sys;
    FRESULT result;

    if (sd->mount_fs)
    {
        result = f_open(&fd_sys, "SYS_logfile.txt", FA_OPEN_APPEND | FA_WRITE);
        if (result == FR_OK)
        {
            f_printf(&fd_sys, "%s [%s] %s", cli_time_get_curr_time_str(), ulog_level_name(level), msg);
            f_close(&fd_sys);
        }
    }

}


void sd_logger_gps(const char *fmt, ...)
{
    SD_Card_State_t *sd = sd_card_get_state();
    static FIL fd_gps;
    FRESULT result;

    static char message[ULOG_MAX_MESSAGE_LENGTH];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(message, ULOG_MAX_MESSAGE_LENGTH, fmt, ap);
    va_end(ap);
    ULOG_TRACE(message);

    if (sd->mount_fs)
    {
        result = f_open(&fd_gps, "GPS_logfile.txt", FA_OPEN_APPEND | FA_WRITE);
        if (result == FR_OK)
        {
            f_printf(&fd_gps, "%s [GPS] %s", cli_time_get_curr_time_str(), message);
            f_close(&fd_gps);
        }
    }

}

