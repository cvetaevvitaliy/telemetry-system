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

#ifndef TELEMETRY_SYSTEM_SD_LOGGER_H
#define TELEMETRY_SYSTEM_SD_LOGGER_H
#include "sd_card.h"
#include "sd_cli_cmd.h"
#include "ulog.h"

#define GPS_LOGGER(...) sd_logger_gps(__VA_ARGS__)

void sd_logger_sys(ulog_level_t level, char *msg);

void sd_logger_gps(const char *fmt, ...);

#endif //TELEMETRY_SYSTEM_SD_LOGGER_H
