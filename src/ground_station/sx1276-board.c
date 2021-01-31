#include "radio.h"
#include "sx1276.h"
#include "sx1276-board.h"

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby, 
    SX1276SetRx,
    SX1276StartCad,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength
};


void SX1276IoInit( void )
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    /** Configure GPIO pins : E19_DIO_0_Pin E19_DIO_1_Pin */
    GPIO_InitStruct.Pin = E19_DIO_0_Pin|E19_DIO_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /** Configure GPIO pins : E19_DIO_2_Pin E19_DIO_3_Pin E19_DIO_4_Pin E19_DIO_5_Pin */
    GPIO_InitStruct.Pin = E19_DIO_2_Pin|E19_DIO_3_Pin|E19_DIO_4_Pin|E19_DIO_5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void SX1276IoIrqInit( void )
{
    /** E19_DIO_0_Pin */
    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    /** E19_DIO_1_Pin */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /** E19_DIO_2_Pin */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    /** E19_DIO_3_Pin */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /** E19_DIO_4_Pin */
    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    /** E19_DIO_5_Pin */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void SX1276IoDeInit( void )
{
    HAL_GPIO_DeInit(GPIOC, E19_DIO_0_Pin);
    HAL_GPIO_DeInit(GPIOC, E19_DIO_1_Pin);

    HAL_GPIO_DeInit(GPIOB, E19_DIO_2_Pin);
    HAL_GPIO_DeInit(GPIOB, E19_DIO_3_Pin);
    HAL_GPIO_DeInit(GPIOB, E19_DIO_4_Pin);
    HAL_GPIO_DeInit(GPIOB, E19_DIO_5_Pin);
}


extern SPI_HandleTypeDef hspi1;
uint16_t SpiInOut(uint16_t outData );

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    SpiInOut( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        SpiInOut( buffer[i] );
    }

    //NSS = 1;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

    SpiInOut( addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( 0 );
    }

    //NSS = 1;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

uint8_t SX1276GetPaSelect( uint32_t channel )
{
    if( channel < RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276SetAntSwLowPower( bool status )
{
    if( RadioIsActive != status )
    {
        RadioIsActive = status;
    
        if( status == false )
        {
            SX1276AntSwInit( );
        }
        else
        {
            SX1276AntSwDeInit( );
        }
    }
}


void SX1276AntSwInit( void )
{
    GPIO_InitTypeDef GPIO_InitStruct;
    HAL_GPIO_WritePin(E19_RX_GPIO_Port, E19_RX_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E19_RX_GPIO_Port, E19_TX_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : E19_TX_Pin E19_RX_Pin */
    GPIO_InitStruct.Pin =  E19_TX_Pin | E19_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(E19_RX_GPIO_Port, &GPIO_InitStruct);
}


void SX1276AntSwDeInit( void )
{
    HAL_GPIO_DeInit(E19_RX_GPIO_Port, E19_TX_Pin | E19_RX_Pin);
}

void SX1276SetAntSw( uint8_t rxTx )
{
    if( rxTx != 0 ) // 1: TX, 0: RX
    {
        HAL_GPIO_WritePin(E19_TX_GPIO_Port, E19_TX_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(E19_RX_GPIO_Port, E19_RX_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(E19_TX_GPIO_Port, E19_TX_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(E19_RX_GPIO_Port, E19_RX_Pin, GPIO_PIN_SET);
    }
    HAL_Delay(5);
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}


void SX1276Reset( void )
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = E19_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(E19_RST_GPIO_Port, &GPIO_InitStruct);

    // Set RESET pin to 0
    HAL_GPIO_WritePin(E19_RST_GPIO_Port, E19_RST_Pin, GPIO_PIN_RESET);

    HAL_Delay(100);

    HAL_GPIO_WritePin(E19_RST_GPIO_Port, E19_RST_Pin, GPIO_PIN_SET);

    HAL_Delay(10);

    HAL_GPIO_DeInit(E19_RST_GPIO_Port, E19_RST_Pin);


}


TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;


/*!
 * \brief Initializes the timer object
 *
 * \remark TimerSetValue function must be called before starting the timer.
 *         this function initializes timestamp and reload value at 0.
 *
 * \param [IN] obj          Structure containing the timer object parameters
 * \param [IN] callback     Function callback called at the end of the timeout
 */
void TimerInit( TimerEvent_t *timer)
{
    if (timer->id == TIMER_TxTimeout) {
        htim11.Instance = TIM11;
        htim11.Init.Prescaler = (uint16_t) (SystemCoreClock / 100) - 1;
        htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim11.Init.Period = 0;
        htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }
    }
    else if (timer->id == TIMER_RxTimeout)
    {
        htim13.Instance = TIM13;
        htim13.Init.Prescaler = (uint16_t) (SystemCoreClock / 200) - 1;
        htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim13.Init.Period = 0;
        htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        if (HAL_TIM_Base_Init(&htim13) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }
    }
    else if (timer->id == TIMER_RxTimeoutSyncWord)
    {
        htim14.Instance = TIM14;
        htim14.Init.Prescaler = (uint16_t) (SystemCoreClock / 200) - 1;
        htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim14.Init.Period = 0;
        htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }
    }

}


void TimerStart( TimerEvent_t *obj )
{
    switch (obj->id) {

        case TIMER_TxTimeout:
            HAL_TIM_Base_Start_IT(&htim11);
            break;

        case TIMER_RxTimeout:
            HAL_TIM_Base_Start_IT(&htim13);
            break;

        case TIMER_RxTimeoutSyncWord:
            HAL_TIM_Base_Start_IT(&htim14);

            break;

        default:
            _Error_Handler(__FILE__, __LINE__);
            break;
    }

}


void TimerStop( TimerEvent_t *obj )
{
    switch (obj->id) {

        case TIMER_TxTimeout:
            HAL_TIM_Base_Stop(&htim11);
            break;

        case TIMER_RxTimeout:
            HAL_TIM_Base_Stop(&htim13);
            break;

        case TIMER_RxTimeoutSyncWord:
            HAL_TIM_Base_Stop(&htim14);
            break;

        default:
            _Error_Handler(__FILE__, __LINE__);
            break;
    }

}


void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    switch (obj->id) {

        case TIMER_TxTimeout:
            HAL_TIM_Base_Stop(&htim11);
            //__HAL_TIM_SET_COUNTER(&htim11, value - 1);
            htim11.Init.Period = (uint16_t) value - 1;
            htim11.State = HAL_TIM_STATE_BUSY;
            TIM_Base_SetConfig(htim11.Instance, &htim11.Init);
            htim11.State = HAL_TIM_STATE_READY;
            break;

        case TIMER_RxTimeout:
            HAL_TIM_Base_Stop(&htim13);
            htim13.Init.Period = (uint16_t) value - 1;
            htim13.State = HAL_TIM_STATE_BUSY;
            TIM_Base_SetConfig(htim13.Instance, &htim13.Init);
            htim13.State = HAL_TIM_STATE_READY;
            break;

        case TIMER_RxTimeoutSyncWord:
            HAL_TIM_Base_Stop(&htim14);
            htim14.Init.Period = (uint16_t) value - 1;
            htim14.State = HAL_TIM_STATE_BUSY;
            TIM_Base_SetConfig(htim14.Instance, &htim14.Init);
            htim14.State = HAL_TIM_STATE_READY;

            break;

        default:
            _Error_Handler(__FILE__, __LINE__);
            break;
    }

}


TimerTime_t TimerGetCurrentTime( void )
{
    return HAL_GetTick();
}


TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{

    uint32_t ts = HAL_GetTick();
    uint32_t elapsed = 0;
    if (savedTime < ts)
        elapsed = ts - savedTime;

    return elapsed;
}


uint16_t SpiInOut(uint16_t outData )
{
    uint8_t rxData = 0;

    HAL_SPI_TransmitReceive( &hspi1, ( uint8_t* )&outData, &rxData, 1, 2000 );

    return( rxData );
}