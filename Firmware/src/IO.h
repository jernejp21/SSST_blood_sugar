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
void SAADC_EnableExtADC(uint8_t* rx_buffer, uint8_t* tx_buffer, uint8_t buffer_size);
void SAADC_DisableExtADC();
void timerStopSampling();
void timerStartSampling();

#endif  //__IO_H