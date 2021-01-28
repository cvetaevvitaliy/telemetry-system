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
#ifndef TELEMETRY_SYSTEM_GPS_SERVICE_H
#define TELEMETRY_SYSTEM_GPS_SERVICE_H
#include <stdint.h>
#include <stdbool.h>
#include <data_struct_def.h>


void gps_service_init(void);

void gps_service_put_char_handle(void);

GPS_Data_t* gps_service_execute(void);

#endif //TELEMETRY_SYSTEM_GPS_SERVICE_H
