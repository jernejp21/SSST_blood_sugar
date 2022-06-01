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

#include "IO.h"
#include "config.h"
#include <hal/nrf_gpio.h>
#include <hal/nrf_saadc.h>
#include <hal/nrf_spim.h>
#include <hal/nrf_timer.h>

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
}

void SAADC_DisableIntADC()
{
  // Stop and clear timer
  timerStopSampling();

  // Disable SAADC EasyDMA interrupt
  nrf_saadc_disable(NRF_SAADC);
  nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_STOP);
  nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_STARTED);
  nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_END);
  nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);
  nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_RESULTDONE);
  nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_STOPPED);
}

void SAADC_EnableExtADC(uint8_t* rx_buffer, uint8_t* tx_buffer, uint8_t buffer_size)
{
  int status;

  // Enable SPI
  nrf_spim_enable(NRF_SPIM3);

  nrf_spim_rx_buffer_set(NRF_SPIM3, rx_buffer, buffer_size);
  nrf_spim_tx_buffer_set(NRF_SPIM3, tx_buffer, buffer_size);
  nrf_spim_task_trigger(NRF_SPIM3, NRF_SPIM_TASK_START);
  do
  {
    status = nrf_spim_event_check(NRF_SPIM3, NRF_SPIM_EVENT_END);
  } while(status == 0);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_END);
}

void SAADC_DisableExtADC()
{
  int status = 0;

  // Stop and clear timer
  timerStopSampling();

  // Shut down external ADC.
  nrf_spim_rx_buffer_set(NRF_SPIM3, (uint8_t*)&status, 1);
  nrf_spim_tx_buffer_set(NRF_SPIM3, (uint8_t*)&status, 1);
  nrf_spim_task_trigger(NRF_SPIM3, NRF_SPIM_TASK_START);
  do
  {
    status = nrf_spim_event_check(NRF_SPIM3, NRF_SPIM_EVENT_END);
  } while(status == 0);

  // Disable SPI EasyDMA interrupt
  nrf_spim_disable(NRF_SPIM3);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_STOPPED);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_ENDRX);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_END);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_ENDTX);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_STARTED);
}

void timerStopSampling()
{
  nrf_timer_task_trigger(NRF_TIMER2, NRF_TIMER_TASK_STOP);
  nrf_timer_task_trigger(NRF_TIMER2, NRF_TIMER_TASK_CLEAR);
  nrf_timer_event_clear(NRF_TIMER2, NRF_TIMER_EVENT_COMPARE0);
  nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_CLEAR);
}

void timerStartSampling()
{
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);
}
