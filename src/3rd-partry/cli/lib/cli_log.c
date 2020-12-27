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

#include "cli_log.h"
#include "string.h"


static struct{
	char cmds[CLI_CMD_LOG_SIZE][CLI_CMD_BUF_SIZE];
	int8_t _curCmd;
	int8_t _cntCmd;
}CLI_Log_s;

void cli_log_init(void)
{
    CLI_Log_s._cntCmd = 0;
    cli_log_cur_reset();
	
	for(uint8_t i = 0; i < CLI_CMD_LOG_SIZE; i++)
	{
        CLI_Log_s.cmds[i][0] = '\0';
	}
}

void cli_log_cmd_push(const char* cmd)
{
	if (CLI_Log_s._cntCmd < CLI_CMD_LOG_SIZE)
	{
		if (CLI_Log_s._cntCmd > 0)
		{
			if (_strcmp(cmd, (const char*) CLI_Log_s.cmds[CLI_Log_s._cntCmd - 1]) == 0)
			{
				cli_memcpy(CLI_Log_s.cmds[CLI_Log_s._cntCmd], cmd, CLI_CMD_BUF_SIZE);
				CLI_Log_s._cntCmd++;
			}
		}
		else
		{
			cli_memcpy(CLI_Log_s.cmds[CLI_Log_s._cntCmd], cmd, CLI_CMD_BUF_SIZE);
			CLI_Log_s._cntCmd++;
		}
	}
	else
	{
		if (_strcmp(cmd, (const char*) CLI_Log_s.cmds[CLI_Log_s._cntCmd - 1]) == 0)
		{
			cli_memcpy(&CLI_Log_s.cmds[0][0], &CLI_Log_s.cmds[1][0], CLI_CMD_BUF_SIZE * (CLI_CMD_LOG_SIZE - 1));
			cli_memcpy(&CLI_Log_s.cmds[CLI_Log_s._cntCmd - 1][0], cmd, CLI_CMD_BUF_SIZE);
            CLI_Log_s._cntCmd = CLI_CMD_LOG_SIZE;
		}
	}
}

const char* cli_log_cmd_get(uint8_t index)
{
	if (index < CLI_CMD_LOG_SIZE)
	{
		return &CLI_Log_s.cmds[index][0];
	}

	return NULL;
}

const char* cli_log_get_next_cmd(void)
{
	if (CLI_Log_s._curCmd < CLI_Log_s._cntCmd - 1)
	{
		CLI_Log_s._curCmd++;
		return &CLI_Log_s.cmds[CLI_Log_s._curCmd][0];
	}

	return NULL;
}

const char* cli_log_get_last_cmd(void)
{
	if (CLI_Log_s._curCmd > 0)
	{
		CLI_Log_s._curCmd--;
		return &CLI_Log_s.cmds[CLI_Log_s._curCmd][0];
	}

	return NULL;
}

void cli_log_cur_reset(void)
{
    CLI_Log_s._curCmd = CLI_Log_s._cntCmd;
}
