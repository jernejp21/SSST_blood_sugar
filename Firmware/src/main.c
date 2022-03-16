/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *
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

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <zephyr.h>

#include <bluetooth/conn.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_lpcomp.h>
#include <hal/nrf_power.h>

#include "config.h"

/* Variables */
static uint8_t isInternalAdc = 1;
static uint16_t adcAvgData[AVG_DATA_SIZE];
static uint16_t adcRawData[RAW_DATA_SIZE];

/* Initial button state */
static uint8_t buttonCurrStatus = BUTTON_LOGIC;
static uint8_t buttonPrevStatus = BUTTON_LOGIC;

static uint8_t startShutdnTimer;

static struct bt_conn_cb btConnectionCb;
static struct k_timer shutdnTimer;

extern void shutdnTimerCb(struct k_timer* timer_id)
{
  uint8_t buttonStart;
  uint8_t buttonCurrent;

  buttonStart = nrf_gpio_pin_read(BTN_SHUTDN_SW);

  nrf_gpio_pin_set(LED_SHUTDN);
  nrf_lpcomp_enable(NRF_LPCOMP);

  do
  {
    buttonCurrent = nrf_gpio_pin_read(BTN_SHUTDN_SW);
  } while(buttonStart == buttonCurrent);

  nrf_gpio_cfg_sense_set(BTN_SHUTDN_SW, NRF_GPIO_PIN_SENSE_LOW);
  nrf_power_system_off(NRF_POWER);

  while(1)
  {
    // Loop needed for debugging purpose.
  }
}

static void systemInit()
{
  // Peripheral initialization
  nrf_gpio_cfg_input(BTN_SHUTDN_SW, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_output(LED_SHUTDN);
  nrf_gpio_pin_set(LED_SHUTDN);

  // BT initialization

  // Kernel init
  k_timer_init(&shutdnTimer, shutdnTimerCb, NULL);
}

static void BTN_AdcSwitch()
{
  // Switch between internal and external ADC. Interrupt
  if(isInternalAdc == 1)
  {
    // Switch to external ADC

    isInternalAdc = 0;
  }
  else
  {
    // Switch to internal ADC

    isInternalAdc = 1;
  }
}

static void BTN_ShutDown()
{
  // Check if button is pressed for 3 s. Polling.

  buttonCurrStatus = nrf_gpio_pin_read(BTN_SHUTDN_SW);

  if(buttonPrevStatus != buttonCurrStatus)
  {
    // Button pressed or released
    if(startShutdnTimer == 0)
    {
      // Button pressed; start timer
      nrf_gpio_pin_clear(LED_SHUTDN);
      k_timer_start(&shutdnTimer, K_MSEC(BTN_SHUTDN_DELAY), K_NO_WAIT);
      startShutdnTimer = 1;
    }
    else
    {
      // Button released; stop timer
      nrf_gpio_pin_set(LED_SHUTDN);
      k_timer_stop(&shutdnTimer);
      startShutdnTimer = 0;
    }
  }

  buttonPrevStatus = buttonCurrStatus;
}

static uint16_t adcSampleAverage(uint16_t* array, size_t size)
{
  uint32_t avg;

  for(size_t index = 0; index < size; index++)
  {
    avg = avg + *(array + index);
  }

  avg = avg / size;

  return (uint16_t)avg;
}

static void readADC()
{
  if(isInternalAdc == 1)
  {
    // Read internal ADC
  }
  else
  {
    // Read external ADC
  }
}

static void BT_Connected(struct bt_conn* conn, uint8_t err)
{
  // Start ADC, EasyDMA
}

static void BT_Disconnected(struct bt_conn* conn, uint8_t reason)
{
  // Stop ADC, EasyDMA
}

static void BT_SendData()
{
  // Send averaged and compressed data
}

static void compressData(uint16_t* rawData, uint16_t* compressedData)
{
}

static void LED_PowerOn()
{
}

static void LED_PowerOff()
{
}

static void LED_externalAdcSelected()
{
}

static void LED_internalAdcSelected()
{
}

static void LED_LowBattery()
{
}

void main(void)
{
  systemInit();
  //  BT, ADC, LEDs, GPIOs everything is prepared

  while(1)
  {
    // k_uptime_get_32();
    BTN_ShutDown();  // read on 50 ms interval
    readADC();  // read on sampling interval
    BT_SendData();  // send when buffer full
  }
}
