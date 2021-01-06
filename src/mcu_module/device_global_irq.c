#include "stm32f4xx.h"
#include "radio.h"
#include "sx1276.h"
#include "sx1276-board.h"
#include "cli.h"


extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM11 && htim->Instance->CR1 == 1) {
        SX1276OnTimeoutIrq();
        HAL_TIM_Base_Stop(&htim11);
    }
    else if(htim->Instance == TIM13 && htim->Instance->CR1 == 1) {
        SX1276OnTimeoutIrq();
        HAL_TIM_Base_Stop(&htim13);
    }
    else if(htim->Instance == TIM14 && htim->Instance->CR1 == 1) {
        SX1276OnTimeoutIrq();
        HAL_TIM_Base_Stop(&htim14);
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
    switch (GPIO_Pin) {

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

        default:
            break;

    }

}

void HAL_SYSTICK_Callback(void)
{
    SysTick_CLI();
}