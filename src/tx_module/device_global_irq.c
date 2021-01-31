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
#include "stm32f4xx.h"
#include "radio.h"
#include "sx1276.h"
#include "sx1276-board.h"
#include "cli.h"
#include "gps_service.h"
#include "sd_card.h"


extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM11 && htim->Instance->CR1 == 1) {
        HAL_TIM_Base_Stop(&htim11);
        SX1276OnTimeoutIrq();
    }
    else if(htim->Instance == TIM13 && htim->Instance->CR1 == 1) {
        HAL_TIM_Base_Stop(&htim13);
        SX1276OnTimeoutIrq();
    }
    else if(htim->Instance == TIM14 && htim->Instance->CR1 == 1) {
        HAL_TIM_Base_Stop(&htim14);
        SX1276OnTimeoutIrq();
    }
    else if(htim->Instance == TIM10 && htim->Instance->CR1 == 1) {
        CDC_SEND_BUFF(); // for send CLI buff
    }

    /** This timer for HAL SysTick*/
    else if (htim->Instance == TIM6) {
        HAL_IncTick();
        SysTick_CLI();
    }

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case E19_DIO_0_Pin:
            SX1276OnDio0Irq();
            break;

        case E19_DIO_1_Pin:
            SX1276OnDio1Irq();
            break;

        case E19_DIO_2_Pin:
            SX1276OnDio2Irq();
            break;

        case E19_DIO_3_Pin:
            SX1276OnDio3Irq();
            break;

        case E19_DIO_4_Pin:
            SX1276OnDio4Irq();
            break;

        case E19_DIO_5_Pin:
            SX1276OnDio5Irq();
            break;

        case SD_PRESET_Pin:
            sd_card_insert_event();
            break;

        default:
            break;

    }

}


void HAL_SYSTICK_Callback(void)
{
    SysTick_CLI();
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        gps_service_put_char_handle();

}


void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        gps_service_put_char_handle();
    ULOG_ERROR("HAL_UART_RxHalfCpltCallback\n");
}

#include "hardware_init.h"
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    HAL_NVIC_SystemReset();
    ULOG_ERROR("HAL_UART_ErrorCallback\n");
}