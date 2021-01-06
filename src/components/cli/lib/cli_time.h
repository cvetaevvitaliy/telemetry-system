/**
 * @brief Compact & Simple Command Line Interface for microcontrollers
 *
 * Copyright (c) 2018 Vitaliy Nimych - vitaliy.nimych@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Modified 14 January 2019 - Vitaliy Nimych
 * add cli input log commands
 *
 * Modified 20 April 2019 - Vitaliy Nimych
 * add pars buttons: UP, DOWN, LEFT, RIGHT
 *
 * Modified 1 September 2019 - Vitaliy Nimych
 * add pars buttons: TAB, ESC, Ctrl + C
 *
 * Modified 27 December 2020 - Vitaliy Nimych
 * add pars buttons: Ctrl + L, small code refactoring
 *
 * */

#ifndef _CLI_TIME_H_
#define _CLI_TIME_H_

#include "cli_config.h"

#define CLI_GETMS()         ((float) CLI_GET_US() / 1000)    // System time in ms

typedef struct{
    uint32_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t msec;
}CLI_Time_t;


/** @brief Get time in millisecond  */
CLI_Time_t cli_time_get_time_ms(uint32_t msec);

/** @brief Get time in millisecond + correction in ms  */
CLI_Time_t cli_time_get_plus_time_ms(uint32_t msec);

char* cli_time_get_curr_time_str(void);

#endif // _CLI_TIME_H_
