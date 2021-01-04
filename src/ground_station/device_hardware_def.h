#ifndef TELEMETRY_SYSTEM_DEVICE_HARDWARE_DEF_H
#define TELEMETRY_SYSTEM_DEVICE_HARDWARE_DEF_H

#define LED_1_Pin                   GPIO_PIN_0
#define LED_1_GPIO_Port             GPIOC
#define LED_2_Pin                   GPIO_PIN_1
#define LED_2_GPIO_Port             GPIOC
#define E19_DIO_0_Pin               GPIO_PIN_4
#define E19_DIO_0_GPIO_Port         GPIOC
#define E19_DIO_0_EXTI_IRQn         EXTI4_IRQn
#define E19_DIO_1_Pin               GPIO_PIN_5
#define E19_DIO_1_GPIO_Port         GPIOC
#define E19_DIO_1_EXTI_IRQn         EXTI9_5_IRQn
#define E19_DIO_2_Pin               GPIO_PIN_0
#define E19_DIO_2_GPIO_Port         GPIOB
#define E19_DIO_2_EXTI_IRQn         EXTI0_IRQn
#define E19_DIO_3_Pin               GPIO_PIN_1
#define E19_DIO_3_GPIO_Port         GPIOB
#define E19_DIO_3_EXTI_IRQn         EXTI1_IRQn
#define E19_DIO_4_Pin               GPIO_PIN_2
#define E19_DIO_4_GPIO_Port         GPIOB
#define E19_DIO_4_EXTI_IRQn         EXTI2_IRQn
#define E19_DIO_5_Pin               GPIO_PIN_10
#define E19_DIO_5_GPIO_Port         GPIOB
#define E19_DIO_5_EXTI_IRQn         EXTI15_10_IRQn
#define E19_RST_Pin                 GPIO_PIN_11
#define E19_RST_GPIO_Port           GPIOB
#define E19_TX_Pin                  GPIO_PIN_12
#define E19_TX_GPIO_Port            GPIOB
#define E19_RX_Pin                  GPIO_PIN_13
#define E19_RX_GPIO_Port            GPIOB
#define BUTTON_RIGHT_Pin            GPIO_PIN_14
#define BUTTON_RIGHT_GPIO_Port      GPIOB
#define BUTTON_RIGHT_EXTI_IRQn      EXTI15_10_IRQn
#define BUTTON_LEFT_Pin             GPIO_PIN_15
#define BUTTON_LEFT_GPIO_Port       GPIOB
#define BUTTON_LEFT_EXTI_IRQn       EXTI15_10_IRQn
#define BUTTON_DOWN_Pin             GPIO_PIN_6
#define BUTTON_DOWN_GPIO_Port       GPIOC
#define BUTTON_DOWN_EXTI_IRQn       EXTI9_5_IRQn
#define BUTTON_UP_Pin               GPIO_PIN_7
#define BUTTON_UP_GPIO_Port         GPIOC
#define BUTTON_UP_EXTI_IRQn         EXTI9_5_IRQn
#define BUZZER_Pin                  GPIO_PIN_8
#define BUZZER_GPIO_Port            GPIOA
#define USB_RESET_Pin               GPIO_PIN_10
#define USB_RESET_GPIO_Port         GPIOA
#define SD_PRESET_Pin               GPIO_PIN_15
#define SD_PRESET_GPIO_Port         GPIOA
#define BUTTON_SELECT_Pin           GPIO_PIN_3
#define BUTTON_SELECT_GPIO_Port     GPIOB
#define BUTTON_SELECT_EXTI_IRQn     EXTI3_IRQn
#define LCD_RESET_Pin               GPIO_PIN_5
#define LCD_RESET_GPIO_Port         GPIOB
#define SPI1_CS_Pin                 GPIO_PIN_1
#define SPI1_CS_GPIO_Port           GPIOA

#endif //TELEMETRY_SYSTEM_DEVICE_HARDWARE_DEF_H
