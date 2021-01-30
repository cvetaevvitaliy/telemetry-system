#ifndef TELEMETRY_SYSTEM_SD_LOGGER_H
#define TELEMETRY_SYSTEM_SD_LOGGER_H
#include "sd_card.h"
#include "sd_cli_cmd.h"
#include "ulog.h"

#define GPS_LOGGER(...) sd_logger_gps(__VA_ARGS__)

void sd_logger_sys(ulog_level_t level, char *msg);

void sd_logger_gps(const char *fmt, ...);

#endif //TELEMETRY_SYSTEM_SD_LOGGER_H
