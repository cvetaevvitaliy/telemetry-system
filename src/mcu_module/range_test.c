#include "range_test.h"
#include "stm32f4xx_hal.h"
#include "radio.h"
#include "tinyprintf.h"
#include "cli_config.h"
#include "sx1276.h"


#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             20        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,//  1: 250 kHz,//  2: 500 kHz,//  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,//  2: 4/6,//  3: 4/7,//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define TX_TIMEOUT_VALUE                            150
#define BUFFER_SIZE                                 64 // Define the payload size here

RadioEvents_t RadioEvents;

static uint8_t Buffer[BUFFER_SIZE];
static uint32_t delay;

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
 * \brief Callback Function to be executed on Radio Tx Done event
 */
void RangeTest_OnTxDone(void);

/*!
 * \brief Callback Function to be executed on Radio Rx Done event
 */
void RangeTest_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Callback Function executed on Radio Tx Timeout event
 */
void RangeTest_OnTxTimeout(void);

/*!
 * \brief Callback Function executed on Radio Rx Timeout event
 */
void RangeTest_OnRxTimeout(void);

/*!
 * \brief Callback Function executed on Radio Rx Error event
 */
void RangeTest_OnRxError(void);


void range_test_init(void)
{

    /** Radio initialization callback */
    RadioEvents.TxDone = RangeTest_OnTxDone;
    RadioEvents.RxDone = RangeTest_OnRxDone;
    RadioEvents.TxTimeout = RangeTest_OnTxTimeout;
    RadioEvents.RxTimeout = RangeTest_OnRxTimeout;
    RadioEvents.RxError = RangeTest_OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

    /** Radio config */
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                       LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                       LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                       true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                       LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                       0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.Rx( RX_TIMEOUT_VALUE );

    delay = HAL_GetTick();


}


void range_test_execute(void)
{
    if (HAL_GetTick() - delay > TX_TIMEOUT_VALUE) {
        static uint64_t i = 0;
        i++;
        sprintf(Buffer,"%d PING ",i);
        Radio.Send(Buffer, BUFFER_SIZE);
        delay = HAL_GetTick();
        HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
    }

}

void RangeTest_OnTxDone(void )
{
    Radio.Sleep( );
    State = TX;
}


void RangeTest_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
    memcpy( Buffer, payload, size );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;

    LOG_DEBUG("RSSI: %ddBm",RssiValue);
    LOG_DEBUG("SNR: %ddb",SnrValue);
    LOG_DEBUG("Payload: %s",Buffer);
}

void RangeTest_OnTxTimeout(void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void RangeTest_OnRxTimeout(void )
{
    //HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
    State = RX_TIMEOUT;
    State = RX;
}

void RangeTest_OnRxError(void )
{
    static uint8_t error = 0;
    error++;
    char str[15];
    sprintf(str, "RX Error: %d", error);
    State = RX;

    LOG_ERROR("RX Error: %d", error);
}