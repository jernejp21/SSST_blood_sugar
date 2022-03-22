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

#include "IO.h"
#include "config.h"
#include <hal/nrf_gpio.h>

void LED_StatusOn()
{
  nrf_gpio_pin_clear(LED_STATUS);
}

void LED_StatusOff()
{
  nrf_gpio_pin_clear(LED_STATUS);
}

void LED_StatusToggle()
{
  nrf_gpio_pin_toggle(LED_STATUS);
}

void LED_BatteryOn()
{
}

void LED_BatteryOff()
{
}

void DCDC_3V3SwitchOff()
{
  nrf_gpio_pin_clear(DCDC_SHTDN_3V3);
}

void DCDC_3V3SwitchOn()
{
  nrf_gpio_pin_set(DCDC_SHTDN_3V3);
}

void DCDC_5VSwitchOff()
{
  nrf_gpio_pin_clear(DCDC_SHTDN_5V);
}

void DCDC_5VSwitchOn()
{
  nrf_gpio_pin_set(DCDC_SHTDN_5V);
}

uint8_t BTN_ShutdnStatus()
{
  uint8_t status = nrf_gpio_pin_read(BTN_SHUTDN);
  return status;
}

uint8_t BTN_AdcChgStatus()
{
  uint8_t status = nrf_gpio_pin_read(BTN_ADC_CHG);
  return status;
}

void SAADC_EnableIntADC();
void SAADC_DisableIntADC();
void SAADC_EnableExtADC();
void SAADC_DisableExtADC();
