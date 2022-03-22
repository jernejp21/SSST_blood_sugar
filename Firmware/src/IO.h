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

#include <stdint.h>
#include <stdio.h>

#ifndef __IO_H
#define __IO_H

void LED_StatusOn();
void LED_StatusOff();
void LED_StatusToggle();
void LED_BatteryOn();
void LED_BatteryOff();
void DCDC_3V3SwitchOff();
void DCDC_3V3SwitchOn();
void DCDC_5VSwitchOff();
void DCDC_5VSwitchOn();
uint8_t BTN_ShutdnStatus();
uint8_t BTN_AdcChgStatus();
void SAADC_EnableIntADC();
void SAADC_DisableIntADC();
void SAADC_EnableExtADC();
void SAADC_DisableExtADC();

#endif  //__IO_H