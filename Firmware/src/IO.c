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
#include <hal/nrf_saadc.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_spim.h>

void LED_StatusOn()
{
  nrf_gpio_pin_set(LED_STATUS);
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
  nrf_gpio_pin_set(LED_BATT);
}

void LED_BatteryOff()
{
  nrf_gpio_pin_clear(LED_BATT);
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

void SAADC_EnableIntADC()
{
  // Enable SAADC and start it
  nrf_saadc_enable(NRF_SAADC);
  nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_START);

  // Start timer
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);
}

void SAADC_DisableIntADC()
{
  // Stop and clear timer
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_STOP);
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_CLEAR);
  nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0);
  nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);

  // Disable SAADC EasyDMA interrupt
  nrf_saadc_disable(NRF_SAADC);
  nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_STOP);
  nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_STARTED);
  nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_END);
  nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);
  nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_RESULTDONE);
  nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_STOPPED);
}

void SAADC_EnableExtADC()
{
  // Enable SPI
  nrf_spim_enable(NRF_SPIM3);

  // Start timer
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);
}

void SAADC_DisableExtADC()
{
  // Stop and clear timer
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_STOP);
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_CLEAR);
  nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0);
  nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);

  // Disable SPI EasyDMA interrupt
  nrf_spim_disable(NRF_SPIM3);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_STOPPED);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_ENDRX);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_END);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_ENDTX);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_STARTED);
}
