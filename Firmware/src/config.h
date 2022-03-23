/**
 * SSST blood sugar. Device for contacless measuring of blood sugar.
 * Copyright (C) 2022  Azurtest
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* Defines */
#define RAW_DATA_SIZE 10  // This will change after testing
#define AVG_DATA_SIZE 100  // This will change after testing
#define BTN_SHUTDN_DELAY 2500  // value in ms
#define ADC_RESOLUTION 4096 // 12-bit resolution

/* HW defines. LEDs, Buttons etc. */
#define BTN_SHUTDN 24
#define BTN_ADC_CHG 25
#define LED_STATUS 13
#define LED_BATT 14
#define DCDC_SHTDN_3V3 3
#define DCDC_SHTDN_5V 28
#define AIN_BATT_SENS 29
#define AIN_BLOOD_SENS 4

/* HW configuration. LOW, HIGH logic etc. */
#define BUTTON_POS_LOGIC 0  // Is button positve logic? Released LOW state, pressed HIGH state
#define BATT_MIN_VOLT (3.0)

#if BUTTON_POS_LOGIC
#define BUTTON_LOGIC 0
#else
#define BUTTON_LOGIC 1
#endif

#endif  //__CONFIG