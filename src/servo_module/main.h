#ifndef __MAIN_H__
#define __MAIN_H__
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
#define LED1_Pin                        GPIO_PIN_0
#define LED1_GPIO_Port                  GPIOC
#define LED2_Pin                        GPIO_PIN_1
#define LED2_GPIO_Port                  GPIOC
#define SERVO_2_Pin                     GPIO_PIN_0
#define SERVO_2_GPIO_Port               GPIOA
#define SPI1_INT1_Pin                   GPIO_PIN_4
#define SPI1_INT1_GPIO_Port             GPIOC
#define SERVO_3_Pin                     GPIO_PIN_6
#define SERVO_3_GPIO_Port               GPIOC
#define SPI2_ITN1_Pin                   GPIO_PIN_7
#define SPI2_ITN1_GPIO_Port             GPIOC
#define OUT_2_Pin                       GPIO_PIN_8
#define OUT_2_GPIO_Port                 GPIOC
#define OUT_1_Pin                       GPIO_PIN_9
#define OUT_1_GPIO_Port                 GPIOC
#define SERVO_1_Pin                     GPIO_PIN_8
#define SERVO_1_GPIO_Port               GPIOA
#define USB_PRESET_Pin                  GPIO_PIN_10
#define USB_PRESET_GPIO_Port            GPIOA
#define SPI3_INT1_Pin                   GPIO_PIN_2
#define SPI3_INT1_GPIO_Port             GPIOD
#define SERVO_4_Pin                     GPIO_PIN_6
#define SERVO_4_GPIO_Port               GPIOB

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
