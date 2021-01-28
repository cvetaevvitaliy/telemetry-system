/*
 * This file is part of "Telemetry system" project.
 *
 * "Telemetry system" are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * "Telemetry system" are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TELEMETRY_SYSTEM_HARDWARE_INIT_H
#define TELEMETRY_SYSTEM_HARDWARE_INIT_H
#include "main.h"

void hardware_init(void);

void MX_USART1_UART_Init(void);
void MX_DMA_Init(void);

#endif //TELEMETRY_SYSTEM_HARDWARE_INIT_H
