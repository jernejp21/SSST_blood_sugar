/**
 * SSST blood sugar. Device for contacless measuring of blood sugar.
 * Copyright (C) 2022  Azurtest
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* Defines */
#define RAW_DATA_SIZE 10  // This will change after testing
#define AVG_DATA_SIZE 100  // This will change after testing
#define BTN_SHUTDN_DELAY 2500  // value in ms
#define ADC_RESOLUTION 4096  // 12-bit resolution
#define SAMPLING_RATE 10000  // 1000000/SAMPLING_RATE is samp. freq.
#define PIN_MAP(port, pin) (port * 32 + pin)

/* HW defines. LEDs, Buttons etc. */
#define BTN_SHUTDN PIN_MAP(0, 24)
#define BTN_ADC_CHG PIN_MAP(0, 25)
#define LED_STATUS PIN_MAP(0, 13)
#define LED_BATT PIN_MAP(0, 14)
#define DCDC_SHTDN_3V3 PIN_MAP(0, 3)
#define DCDC_SHTDN_5V PIN_MAP(0, 28)
#define AIN_BATT_SENS PIN_MAP(0, 29)
#define AIN_BLOOD_SENS PIN_MAP(0, 4)
#define SPI_SCK_PIN PIN_MAP(1, 8)
#define SPI_MOSI_PIN PIN_MAP(1, 5)
#define SPI_MISO_PIN PIN_MAP(1, 7)
#define SPI_CS_PIN PIN_MAP(1, 6)

/* HW configuration. LOW, HIGH logic etc. */
#define BUTTON_POS_LOGIC 0  // Is button positve logic? Released LOW state, pressed HIGH state
#define SUPPLY_VOLTAGE (3.0)
#define BATT_MIN_VOLT (SUPPLY_VOLTAGE * 12 / 16)  // Not actual voltage, but adjusted where full battery (about 4.2 V) is suppy voltage

#if BUTTON_POS_LOGIC
#define BUTTON_LOGIC 0
#else
#define BUTTON_LOGIC 1
#endif

#endif  //__CONFIG