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
#ifndef __MAIN_H__
#define __MAIN_H__

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

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

void add_test_cli_cmd(void);

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
