#ifndef __MAIN_H__
#define __MAIN_H__

#define LED_1_Pin                       GPIO_PIN_0
#define LED_1_GPIO_Port                 GPIOC
#define LED_2_Pin                       GPIO_PIN_1
#define LED_2_GPIO_Port                 GPIOC
#define ADC1_VBAT_Pin                   GPIO_PIN_2
#define ADC1_VBAT_GPIO_Port             GPIOC
#define ADC1_RESERV_VBAT_Pin            GPIO_PIN_3
#define ADC1_RESERV_VBAT_GPIO_Port      GPIOC
#define E19_DIO_0_Pin                   GPIO_PIN_4
#define E19_DIO_0_GPIO_Port             GPIOC
#define E19_DIO_1_Pin                   GPIO_PIN_5
#define E19_DIO_1_GPIO_Port             GPIOC
#define E19_DIO_2_Pin                   GPIO_PIN_0
#define E19_DIO_2_GPIO_Port             GPIOB
#define E19_DIO_3_Pin                   GPIO_PIN_1
#define E19_DIO_3_GPIO_Port             GPIOB
#define E19_DIO_4_Pin                   GPIO_PIN_2
#define E19_DIO_4_GPIO_Port             GPIOB
#define E19_DIO_5_Pin                   GPIO_PIN_10
#define E19_DIO_5_GPIO_Port             GPIOB
#define E19_RST_Pin                     GPIO_PIN_11
#define E19_RST_GPIO_Port               GPIOB
#define E19_TX_Pin                      GPIO_PIN_12
#define E19_TX_GPIO_Port                GPIOB
#define E19_RX_Pin                      GPIO_PIN_13
#define E19_RX_GPIO_Port                GPIOB
#define SPI3_INT_Pin                    GPIO_PIN_6
#define SPI3_INT_GPIO_Port              GPIOC
#define SPI3_NSS_Pin                    GPIO_PIN_7
#define SPI3_NSS_GPIO_Port              GPIOC
#define BUZZER_Pin                      GPIO_PIN_8
#define BUZZER_GPIO_Port                GPIOA
#define USB_RESET_Pin                   GPIO_PIN_10
#define USB_RESET_GPIO_Port             GPIOA
#define SD_PRESET_Pin                   GPIO_PIN_15
#define SD_PRESET_GPIO_Port             GPIOA

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */


#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
