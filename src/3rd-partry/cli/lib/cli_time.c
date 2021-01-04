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

#include "cli_time.h"

static uint32_t def_time_ms;

/// \brief Generate CLI_Time_t struct from milliseconds
CLI_Time_t cli_time_get_time_ms(uint32_t msec)
{
    CLI_Time_t res;
    res.msec = msec % 1000;
    uint32_t s = msec / 1000;
    res.second = s % 60;
    uint32_t m = s / 60;
    res.minute = m % 60;
    uint32_t h = s / 3600;
    res.hour = h;

    return res;
}

inline CLI_Time_t cli_time_get_plus_time_ms(uint32_t msec)
{
    return cli_time_get_time_ms(msec + def_time_ms);
}

char* cli_time_get_curr_time_str(void)
{
    static char tv_str[20];
    CLI_Time_t tv = {0};
#if (CLI_TIMELEFT_EN == 1)
    uint32_t ms = CLI_GETMS();
    tv = cli_time_get_plus_time_ms(ms);
#endif
    sprintf(tv_str, "%02dh:%02dm:%02ds.%03d", (int) tv.hour, (int) tv.minute, (int) tv.second, (int) tv.msec);
    return tv_str;
}
