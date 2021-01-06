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

#ifndef _CLI_QUEUE_H_
#define _CLI_QUEUE_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "cli_config.h"

/// \brief Struct Queue
typedef struct
{
    void* ptrObj;           // pointer on objects
    uint16_t size;          // queue size
    uint16_t _cntr;         // queue count added objects
    uint8_t sizeObj;        // one object size
    uint32_t mode;          // queue work mode
} CLI_Queue_t;

#define QUEUE_FORCED_PUSH_POP_Msk		(0x01)		// Forced queue filling

typedef CLI_Queue_t QueueObj;

/** @brief CLI Queue init */
void cli_queue_init(QueueObj* qd, uint16_t sizeQueue, uint8_t sizeObj, uint32_t mode);

/** @brief Push value to Queue  */
bool cli_queue_push(QueueObj* qd, const void* value);

/** @brief Pop value from Queue  */
bool cli_queue_pop(QueueObj* qd, void* value);

/** @brief check Queue on: full  */
bool cli_queue_is_full(QueueObj* qd);

/** @brief check Queue on: empty  */
bool cli_queue_is_empty(QueueObj* qd);

/** @brief check Queue on: equal  */
bool cli_queue_is_equal(QueueObj* qd, const void* items, uint32_t size);

#endif // _CLI_QUEUE_H_
