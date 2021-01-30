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

