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

#include "cli_queue.h"

#include "stdlib.h"
#include "string.h"


static char queue_arary[3 * sizeof(char)] = {0};

void cli_queue_init(QueueObj* qdObj, uint16_t sizeQueue, uint8_t sizeObj, uint32_t mode)
{
    CLI_Queue_t* qd = (CLI_Queue_t*) qdObj;

    qd->ptrObj = queue_arary;   //(void*) _mem(sizeObj * sizeQueue);

    qd->size = sizeQueue;
    qd->_cntr = 0;
    qd->sizeObj = sizeObj;
    qd->mode = mode;
}

bool cli_queue_push(QueueObj* qdObj, const void* value)
{
    CLI_Queue_t* qd = (CLI_Queue_t*) qdObj;

    if (qd->_cntr >= qd->size)
    {
        if ((qd->mode & QUEUE_FORCED_PUSH_POP_Msk) != 0)
        {
            memcpy(qd->ptrObj, qd->ptrObj + qd->sizeObj, qd->sizeObj * (qd->size - 1));
            memcpy(qd->ptrObj + qd->sizeObj * (qd->_cntr - 1), value, qd->sizeObj);
            qd->_cntr = qd->size;
        }
        else
            return false;
    }
    else
    {
        memcpy(qd->ptrObj + qd->sizeObj * qd->_cntr, value, qd->sizeObj);
        qd->_cntr++;
    }

    return true;
}

bool cli_queue_pop(QueueObj* qdObj, void* value)
{
    CLI_Queue_t* qd = (CLI_Queue_t*) qdObj;

    if (qd->_cntr == 0)
        return false;

    memcpy((uint8_t*)value, qd->ptrObj, qd->sizeObj);
    for(uint8_t i = 0; i < qd->_cntr; i++)
    {
        memcpy((uint8_t*)(qd->ptrObj + qd->sizeObj * i), (uint8_t*)(qd->ptrObj + qd->sizeObj * (i+1)), qd->sizeObj);
    }

    qd->_cntr--;
    return true;
}

bool cli_queue_is_full(QueueObj* qdObj)
{
    CLI_Queue_t* qd = (CLI_Queue_t*) qdObj;

    return qd->_cntr >= qd->size;
}

bool cli_queue_is_empty(QueueObj* qdObj)
{
    CLI_Queue_t* qd = (CLI_Queue_t*) qdObj;

    return qd->_cntr == 0;
}

bool cli_queue_is_equal(QueueObj* qd, const void* items, uint32_t size)
{
    if (size > qd->_cntr)
        return false;

    return (memcmp(qd->ptrObj, items, qd->sizeObj * size) == 0);
}
