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

#ifndef _TERMINAL_LOG_H_
#define _TERMINAL_LOG_H_
#include "cli_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "cli_queue.h"


void cli_log_init();
void cli_log_cmd_push(const char* cmd);
const char* cli_log_cmd_get(uint8_t index);
const char* cli_log_get_next_cmd(void);
const char* cli_log_get_last_cmd(void);
void cli_log_cur_reset(void);

#endif // _TERMINAL_LOG_H_
