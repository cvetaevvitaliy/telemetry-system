/**
 * @brief Сompact & Simple Command Line Interface for microcontrollers
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

/*************************************************************************
 *			CLI configure file
 *	---------------------------------------------------------------------
 *
 ************************************************************************/

#ifndef _CLI_CONFIG_H_
#define _CLI_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "cli.h"
#include "cli_io.h"
#include "cli_time.h"
#include "ulog.h"


// ****************************** Code keys ********************************

#define _KEY_INIT(c)                            ((char)c)

#define CLI_KEY_ENTER                           (_KEY_INIT(0x0D))   // Enter command symbol
#define CLI_KEY_BACKSPACE                       (_KEY_INIT(0x08))   // Delete character before cursor position
#define CLI_KEY_LSHIFT                          (_KEY_INIT(0x08))   // Left shift symbol
#define CLI_KEY_ESCAPE                          (_KEY_INIT(0xF0))   // Exception execute command symbol
#define CLI_KEY_UP                              (_KEY_INIT(0xF1))   // KeyUp symbol
#define CLI_KEY_RIGHT                           (_KEY_INIT(0xF2))   // KeyRight symbol
#define CLI_KEY_DOWN                            (_KEY_INIT(0xF3))   // KeyDown symbol
#define CLI_KEY_LEFT                            (_KEY_INIT(0xF4))   // KeyLeft symbol
#define CLI_KEY_DEL                             (_KEY_INIT(0xF5))   // Delete character after cursor position
#define CLI_KEY_HOME                            (_KEY_INIT(0xA0))   // Home key
#define CLI_KEY_END                             (_KEY_INIT(0xA1))   // End key
#define CLI_KEY_TAB                             (_KEY_INIT(0x09))   // TAB key
#define CLI_KEY_CLEAR_SCR                       (_KEY_INIT(0x0C))   // Clear screen CTRL+L
#define CLI_KEY_CLEAR_LINE                      (_KEY_INIT(0x9B))   // clear line

// **************************************************************************

// **********************    CLI Settings   *********************************
#ifndef BUILD_NAME
#define BUILD_NAME                              ("______________")  // If not defined before build -DBUILD_NAME, will be defined here
#endif
#ifndef GIT_BRANCH
#define GIT_BRANCH                              ("__________")      // If not defined before build -DGIT_BRANCH, will be defined here
#endif
#ifndef GIT_HASH
#define GIT_HASH                                ("________")        // If not defined before build -DGIT_HASH will be defined here
#endif
#ifndef MINOR
#define MINOR                                   ("_")               // If not defined before build -DMINOR, will be defined here
#endif
#ifndef MAJOR
#define MAJOR                                   ("_")               // If not defined before build -DMAJOR, will be defined here
#endif
#ifndef PATCH
#define PATCH                                   ("_")               // If not defined before build -DPATCH, will be defined here
#endif
#ifndef MCU
#define MCU                                     ("_________")       // If not defined before build -DMCU, will be defined here
#endif
#ifndef _WHERE_BUILD
#define _WHERE_BUILD                            ("Local build")     // If not defined before build -D_WHERE, will be defined here only Local build
#endif

#define _TERM_VER_                              ("v0.0.2")          // CLI version
#define CLI_SIZE_MAX_CMD                        (20)                // Max number of commands
#define CLI_CMD_BUF_SIZE                        (20)                // Max number of character buffer string command
#define CLI_CMD_LOG_SIZE                        (10)                // Max number of logging command
#define CLI_ARGS_BUF_SIZE                       (10)                // Max number of arguments in one command
#define CLI_ARG_SIZE                            (10)                 // Max number character of one arguments
#define CHAR_INTERRUPT                          (0x03)              // Abort execute command key-code symbol
#define STRING_TERM_ENTER                       ("\n\r")            // String new line
#define STRING_TERM_ARROW                       (">> ")             // String arrow enter

#define CLI_TIMELEFT_EN                         (1)                 // Calculate time (If you need performance, don't turn it on )
#define CLI_DEFAULT_ALLOC_EN                    (1)                 // Default Memory Allocate functions (use static or malloc memmory for add new command)
#define CLI_DEFAULT_STRING_EN                   (1)                 // Default String functions
#define CLI_TINY_SPRINTF                        (1)                 // Default sprintf functions
#define CLI_PRINT_ERROR_EXEC_EN                 (1)                 // Print error after execute command
#define CLI_PRINT_ERROR_ADD_CMD_EN              (1)                 // Print error after added command
#define ECHO_EN                                 (1)                 // Enter echo enable

// **************************************************************************


// *************************     Tiny sprintf     ***************************
#if (CLI_TINY_SPRINTF == 1)
#include "tinyprintf.h"
#endif


// *************************   IO CLI Settings    ***************************

extern char output_print_buffer[256];
#define CLI_PRINTF(...)                         {sprintf(output_print_buffer,__VA_ARGS__);CLI_PrintStr(output_print_buffer);}
#if (ECHO_EN == 1)
#define CLI_PUT_CHAR                            CLI_PrintChar
#else	// ECHO_EN != 1 ECHO off
#define CLI_PUT_CHAR
#endif  // ECHO_EN == 1


// ***********************   IO Debug CLI Settings    ***********************

/**< This macro for debug software */
#if (DEBUG == 1) // todo: need implement LOG LEVEL : 3 - Debug, 2 - Info, 1 - Error, 0 - Nothing print in console
#if (DEBUG_TIMESTAMP == 1)
#define LOG_DEBUG(f_, ...)                      CLI_PRINTF(("%s [DEBUG] "f_), cli_time_get_curr_time_str(), ##__VA_ARGS__)
#define LOG_INFO(f_, ...)                       CLI_PRINTF(("%s [INFO] "f_), cli_time_get_curr_time_str(), ##__VA_ARGS__)
#define LOG_ERROR(f_, ...)                      CLI_PRINTF(("%s [ERROR] "f_), cli_time_get_curr_time_str(), ##__VA_ARGS__)
#else
#define LOG_DEBUG(f_, ...)                      CLI_PRINTF(("\n[DEBUG] "f_), ##__VA_ARGS__)
#define LOG_INFO(f_, ...)                       CLI_PRINTF(("\n[INFO] "f_), ##__VA_ARGS__)
#define LOG_ERROR(f_, ...)                      CLI_PRINTF(("\n[ERROR] "f_), ##__VA_ARGS__)
#endif
#else
#define LOG_DEBUG(f_, ...)
#define LOG_INFO(f_, ...)
#define LOG_ERROR(f_, ...)
#endif


// ************************ Time calculate Settings *************************

#if (CLI_TIMELEFT_EN == 1)

// yout implementation
extern volatile uint64_t _tick;
#define CLI_GET_US()                            ((float)_tick * 1000)   // System time in us
#define CLI_GetFastUs()                         (_tick << 3)            // System time in us (not exact)
#define CLI_GetFastMs()                         (_tick >> 7)            // System time in ms (not exact)
#define CLI_CounterReset()                      {_tick = 0;}
#else	// CLI_TIMELEFT_EN != 1
#define CLI_GetUs()                             (0)                     // System time in us
#define CLI_GetFastUs()                         (0)                     // System time in us (not exact)
#define CLI_GetFastMs()                         (0)                     // System time in ms (not exact)
#define CLI_CounterReset()                      {}

#endif    // CLI_TIMELEFT_EN == 1


// ********************** memory allocate functions *************************

#if (CLI_DEFAULT_ALLOC_EN == 1)
#include <malloc.h>
#define cli_malloc                              malloc                  // dynamic memory
#define cli_free                                free                    // dynamic memory
#else
#define cli_malloc                                                      // your implementation
#define cli_free                                                        // your implementation
#endif


// *************************** string functions *****************************

#if (CLI_DEFAULT_STRING_EN == 1)
#include <string.h>
#include <tinystring.h>  // todo: need implement tiny memcpy, see tiny inystring.h
#define cli_memcpy                              memcpy
#else
#define cli_memcpy                                                      // your implementation
#endif


#endif
