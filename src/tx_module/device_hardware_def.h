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
#ifndef TELEMETRY_SYSTEM_DEVICE_HARDWARE_DEF_H
#define TELEMETRY_SYSTEM_DEVICE_HARDWARE_DEF_H

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
#define SPI1_CS_Pin                     GPIO_PIN_4
#define SPI1_CS_GPIO_Port               GPIOA

#endif //TELEMETRY_SYSTEM_DEVICE_HARDWARE_DEF_H
