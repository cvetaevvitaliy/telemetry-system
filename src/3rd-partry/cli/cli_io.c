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
 * add pars buttons for clear screen : Ctrl + L, small code refactoring
 *
 * */

#include "cli_io.h"

/** Your implementation of acceptance a character with IO stream
 * just call this function and put character symbol with IO */
void CLI_AppendChar(char c)
{
    cli_append_char(c);
}

/** Your implementation of sending a character to IO stream */
void CLI_PrintChar(char c)
{
    CDC_Transmit_SET_BUFF(c);
}

void CLI_PrintStr(char *str)
{
    uint16_t i = 0;
    while (str[i] != '\0') {
        CLI_PrintChar(str[i]);
        i++;
    }
}
