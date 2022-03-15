/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <zephyr.h>

#include <bluetooth/conn.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0 ""
#define PIN 0
#define FLAGS 0
#endif

/* Defines */
#define RAW_DATA_SIZE 10  // This will change after testing
#define AVG_DATA_SIZE 100  // This will change after testing

/* Variables */
uint8_t isInternalAdc = 1;
uint16_t adcAveragedData[AVG_DATA_SIZE];
uint16_t adcRawData[RAW_DATA_SIZE];

struct bt_conn_cb btConnectionCallback;

static void systemInit()
{
  // Peripheral initialization

  // BT initialization
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
  // systemInit();
  //  BT, ADC, LEDs, GPIOs everything is prepared

  while(1)
  {
    // k_uptime_get_32();
    BTN_ShutDown();  // read on 50 ms interval
    readADC();  // read on sampling interval
    BT_SendData();  // send when buffer full
  }
}
