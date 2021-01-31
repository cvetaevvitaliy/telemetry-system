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
#define RX_TIMEOUT_VALUE                            250
#define TX_TIMEOUT_VALUE                            150
#define BUFFER_SIZE                                 64 // Define the payload size here

RadioEvents_t RadioEvents;

static uint8_t Buffer[BUFFER_SIZE];

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
    TX_DONE,
    IDLE,
}States_t;

States_t State = TX;

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



}


void range_test_execute(void)
{
    static uint64_t i = 0;
    if (State == TX)
    {
        State = IDLE;
        ULOG_DEBUG("Send\n");
        sprintf(Buffer, "%d PING ", i);
        i++;
        Radio.Send(Buffer, BUFFER_SIZE);
        HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
    }

    if (State == TX_DONE)
    {
        State = IDLE;
        Radio.Rx(RX_TIMEOUT_VALUE);
    }


}

void RangeTest_OnTxDone(void )
{
    ULOG_DEBUG("TxDone\n");
    State = TX_DONE;
}


void RangeTest_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
    memcpy( Buffer, payload, size );
    RssiValue = rssi;
    SnrValue = snr;
    State = TX;

    ULOG_DEBUG("RSSI: %ddBm\n",RssiValue);
    ULOG_DEBUG("SNR: %ddb\n",SnrValue);
    ULOG_DEBUG("Payload: %s\n",Buffer);
}

void RangeTest_OnTxTimeout(void )
{
    State = TX_DONE;
}

void RangeTest_OnRxTimeout(void )
{
    State = TX;
}

void RangeTest_OnRxError(void )
{
    static uint8_t error = 0;
    error++;
    char str[15];
    sprintf(str, "RX Error: %d", error);
    State = RX;

    ULOG_ERROR("RX Error: %d\n", error);
}