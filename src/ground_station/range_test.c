#include "range_test.h"
#include "stm32f4xx_hal.h"
#include "ssd1306.h"
#include "bq2589x_charger.h"
#include "radio.h"
#include "tinyprintf.h"
#include "cli_config.h"


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
}States_t;

States_t State = RX;

int8_t RssiValue = 0;
int8_t SnrValue = 0;
static uint32_t delay;

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
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    ssd1306_Init();
    ssd1306_FlipScreenVertically();
    ssd1306_Draw_String("Range test", 0, 0, &Font_8x10);
    ssd1306_Draw_String("RX Error: 0", 0, 40, &Font_8x10);
    ssd1306_UpdateScreen();

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
                       true, 0, 0, LORA_IQ_INVERSION_ON, 0 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                       LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                       0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.Rx( 100 );

    delay = HAL_GetTick();


}


void range_test_execute(void)
{
    char str[BUFFER_SIZE];

    /** Super simple way to poweroff device, UI not yet implemented  */
    if (!HAL_GPIO_ReadPin(BUTTON_SELECT_GPIO_Port, BUTTON_SELECT_Pin))
        bq2589x_enter_ship_mode();

    /** For print battery voltage */
    if (HAL_GetTick() - delay > 500 ) {
        float bat = (float ) bq2589x_adc_read_battery_volt() / 1000.0f;
        delay = HAL_GetTick();
        sprintf(str, "%.2fV", bat);
        ssd1306_SetColor(Black);
        ssd1306_Draw_String(str, 90, 0, &Font_8x10);
        ssd1306_SetColor(White);
    }

    sprintf(str, "RSSI: %ddBm       ", RssiValue);
    ssd1306_Draw_String(str, 0, 10, &Font_8x10);
    sprintf(str, "SNR: %ddb        ", SnrValue);
    ssd1306_Draw_String(str, 0, 20, &Font_8x10);
    sprintf(str, "%s           ", Buffer);
    ssd1306_Draw_String(str, 0, 30, &Font_8x10);


    static uint32_t i = 0;
    if (!HAL_GPIO_ReadPin(BUTTON_LEFT_GPIO_Port, BUTTON_LEFT_Pin))
    {
        i++;
        sprintf(Buffer, "%d PING ", i);
        Radio.Send(Buffer, BUFFER_SIZE);
        Radio.Rx( RX_TIMEOUT_VALUE );
        sprintf(str, "%s           ", Buffer);
        ssd1306_Draw_String(str, 0, 50, &Font_8x10);
    }

    ssd1306_UpdateScreen();


}

void RangeTest_OnTxDone(void )
{
    //Radio.Sleep();
    LOG_INFO("TxDone\n");
    Radio.Rx( RX_TIMEOUT_VALUE );
}


void RangeTest_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
    memcpy( Buffer, payload, size );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;

    LOG_DEBUG("RSSI: %ddBm\n",RssiValue);
    LOG_DEBUG("SNR: %ddb\n",SnrValue);
    LOG_DEBUG("Payload: %s\n",Buffer);
}

void RangeTest_OnTxTimeout(void )
{
    //Radio.Sleep( );
    LOG_INFO("TxTimeout\n");
    //Radio.Rx( RX_TIMEOUT_VALUE );
}

void RangeTest_OnRxTimeout(void )
{
    LOG_INFO("RxTimeout\n");
    State = RX_TIMEOUT;
    State = RX;
    //Radio.Rx( RX_TIMEOUT_VALUE );
}

void RangeTest_OnRxError(void )
{
    static uint8_t error = 0;
    error++;
    char str[15];
    sprintf(str, "RX Error: %d", error);
    ssd1306_Draw_String(str, 0, 40, &Font_8x10);
    State = RX;

    LOG_ERROR("RX Error: %d\n", error);
}