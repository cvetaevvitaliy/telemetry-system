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

#ifndef _CLI_H_
#define _CLI_H_

#include "cli_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <tinystring.h>


/** @brief CLI Result Execute */
typedef enum{ 
	CLI_OK = 0,
	CLI_Err,
	CLI_NotFound,
	CLI_ArgErr,
	CLI_ExecErr,
	CLI_WorkInt
} CLI_Result_t;

/** @brief CLI add new command result */
typedef enum{ 
	ADD_CMD_OK = 0,
	ADD_CMD_MaxCmd,
	ADD_CMD_FcnNull,
	ADD_CMD_EmptyName,
	ADD_CMD_RetryName
} CLI_Add_Result_t;

/** @brief CLI Char append result */
typedef enum{ 
	CLI_APPEND_OK = 0,
    CLI_APPEND_Enter,
    CLI_APPEND_BufFull,
    CLI_APPEND_Reset,
    CLI_APPEND_Ignore
} CLI_Append_Result_t;

/** @brief CLI Mode Execute command */
typedef enum{
	CLI_PrintNone = 0x0000,
	CLI_PrintStartTime = 0x0001,
	CLI_PrintStopTime = 0x0002,
	CLI_PrintDiffTime = 0x0004,
	CLI_Print_All = 0xFFFF,
} CLI_Type_Mode_Cmd_t;


bool cli_get_int_state(void); // todo: need implement - abort run current job
#define CLI_CHECK_ABORT()   { if (cli_get_int_state()){return TE_WorkInt;}}

/** @brief Terminal initialize */
void cli_init(void);

/** @brief CLI loop Execute input command */
bool cli_loop_service(void);


/**
 * @brief Function for Add command
 * @param name - input name
 * @param fcn - callback function
 * @param argc - min count arguments
 * @param mode - execute mode
 * @param descr - description
 * @return result append command
 * */
CLI_Add_Result_t cli_add_new_cmd(const char* name, CLI_Result_t (*fcn)(), uint8_t argc, CLI_Type_Mode_Cmd_t mode, const char* descr);


/** @brief Append new symbols for cli input parser */
CLI_Append_Result_t cli_append_char(char ch);

/** @brief This function for check arguments flag */
bool cli_is_arg_flag(const char* flag);

/** @brief This function for check convert string arguments to Dec */
bool cli_get_arg_dec_by_flag(const char* flag, uint32_t* outValue);

/** @brief This function for check convert string arguments to Hex */
bool cli_get_arg_hex_by_flag(const char* flag, uint32_t* outValue);

/** @brief This function for get argument in string */
int cli_get_arg_str(uint8_t index, char* str);

char* cli_get_arg(uint8_t index);
/** @brief Convert and Get argument in Dec 
 * @param index argument
 * @return results conversion
*/
uint32_t cli_get_arg_dec(uint8_t index);

/** @brief Convert and Get argument in Hex 
 * @param index argument
 * @return results conversion
*/
uint32_t cli_get_arg_hex(uint8_t index);


void cli_set_first_in_cli(bool set);

void SysTick_CLI(void); // todo: need refactor name

#endif // _CLI_H_
