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

#ifndef _CLI_INPUT_H_
#define _CLI_INPUT_H_

#include "cli_config.h"

typedef struct
{
    char keyCode;
    bool isValid;
    bool isAlphaBet;
}CLI_InputValue_t;

typedef enum
{
    MainBuffer		= 0x00,
    TransitBuffer	= 0x01
}CLI_InputBufferType_t;

/** @brief Init CLI imput parser module */
void cli_input_init(void);

/** @brief Get CLI imput buffer */
char* cli_input_get_buffer(CLI_InputBufferType_t type);

/** @brief Set CLI imput buffer */
void cli_input_set_buffer(CLI_InputBufferType_t type, char* buffer, uint32_t len);

/** @brief Check CLI imput buffer is empty */
bool cli_input_is_empty(void);

/** @brief Check CLI imput buffer is full */
bool cli_input_is_full(void);

/** @brief Delete chars in buffer */
void cli_input_rem_char(void);

/** @brief Add char in pars buffer */
void cli_input_add_char(char c);

/** @brief CLI input process cache  */
void cli_input_cache(void);

/** @brief CLI process flush cache */
void cli_input_reset(void);

/** @brief Get last char from cache */
char cli_input_get_last_char(void);

/** @brief Refresh last char from cache */
void cli_input_refresh(const char* newCmd);

/** @brief Add char to cache */
CLI_InputValue_t cli_input_put_char(char c);

/** @brief Send "chars key" delete buttom to user */
void cli_input_delete(void);

/** @brief Send "chars key" backspace buttom to user*/
void cli_input_backspace(void);

/** @brief Send "chars key"  home buttom to user */
void cli_input_cursor_to_home(void);

/** @brief Send "chars key"  end  buttom to user */
void cli_input_cursor_to_end();

/** @brief Send "chars key"  jump cursor to numper chars user */
void cli_input_cursor_to(uint16_t pos);

/** @brief Send "chars key"  left buttom to user */
void cli_input_cursor_to_left(void);

/** @brief Send "chars key"  right buttom to user */
void cli_input_cursor_to_right(void);

/** @brief Send "chars key"  shift buttom  user */
void cli_input_cursor_shift(int16_t shift);

#endif // _CLI_INPUT_H_
