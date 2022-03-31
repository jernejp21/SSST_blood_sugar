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
#include <hal/nrf_ppi.h>
#include <hal/nrf_saadc.h>
#include <hal/nrf_spim.h>
#include <hal/nrf_timer.h>

#include "IO.h"
#include "config.h"

#define BUFFER_SIZE 3

/* Variables */
volatile uint8_t isInternalAdc;
volatile uint8_t isLowBattery;
static uint16_t adcAvgData1[AVG_DATA_SIZE];
static uint16_t adcAvgData2[AVG_DATA_SIZE];
static uint16_t adcRawData[RAW_DATA_SIZE] __attribute__((aligned(2)));
static uint16_t* p_adcAvgData;
static uint8_t rx_buff[BUFFER_SIZE] __attribute__((aligned(2)));
static uint8_t tx_buff[BUFFER_SIZE] __attribute__((aligned(2)));
static uint32_t adcRawDataCnt;
static uint32_t adcAvgDataCnt;

/* Kernel variables */
static uint8_t startShutdnTimer;
static struct bt_conn_cb btConnectionCb;
static struct k_timer shutdnTimer;
static struct k_timer ledStartupBlink;

K_THREAD_STACK_DEFINE(stk_ShutDnBtn, 1024);
struct k_thread thr_ShutDnBtn;
K_THREAD_STACK_DEFINE(stk_adcSwitchBtn, 1024);
struct k_thread thr_adcSwitchBtn;
K_THREAD_STACK_DEFINE(stk_heartBeat, 1024);
struct k_thread thr_heartBeat;
K_THREAD_STACK_DEFINE(stk_blueTooth, 1024);
struct k_thread thr_blueTooth;

/* Functions */

static void ledExtADCSelPattern()
{
  LED_StatusOn();
  k_busy_wait(200000);
  LED_StatusOff();
}

static void ledIntADCSelPattern()
{
  LED_StatusOn();
  LED_BatteryOn();
  k_busy_wait(200000);
  LED_StatusOff();
  LED_BatteryOff();
}

static void ledLowBattery()
{
  LED_BatteryOn();
  k_busy_wait(200000);
  LED_BatteryOff();
}

extern void ledStartupBlinkCb(struct k_timer* timer_id)
{
  static int blinkCnt = 0;
  int nrOfBlinks = 6;  // 3 times on, 3 times off

  LED_StatusToggle();
  blinkCnt++;
  if(blinkCnt == nrOfBlinks)
  {
    k_timer_start(&ledStartupBlink, K_MSEC(2000), K_NO_WAIT);  // after blinks, turn on led for some time
    LED_StatusToggle();
  }
  if(blinkCnt == (nrOfBlinks + 1))
  {
    k_timer_stop(&ledStartupBlink);
    LED_StatusOff();
  }
}

extern void shutdnTimerCb(struct k_timer* timer_id)
{
  uint8_t buttonStart;
  uint8_t buttonCurrent;

  buttonStart = BTN_ShutdnStatus();

  LED_StatusOn();
  LED_BatteryOn();

  do
  {
    buttonCurrent = BTN_ShutdnStatus();
  } while(buttonStart == buttonCurrent);

  LED_StatusOff();
  LED_BatteryOff();

  DCDC_5VSwitchOff();
  DCDC_3V3SwitchOff();

  // This is needed for debugging purpose.
  k_thread_abort(&thr_adcSwitchBtn);
  k_thread_abort(&thr_blueTooth);
  k_thread_abort(&thr_heartBeat);
  k_thread_abort(&thr_ShutDnBtn);
  irq_disable(TIMER0_IRQn);
  irq_disable(TIMER1_IRQn);
  irq_disable(LPCOMP_IRQn);

  while(1)
  {
  }
}

static void adcSwitchBtn(void* p1, void* p2, void* p3)
{
  uint8_t buttonCurrStatus = BUTTON_LOGIC;
  uint8_t buttonPrevStatus = BUTTON_LOGIC;

  while(1)
  {

    buttonCurrStatus = BTN_AdcChgStatus();

    if((buttonPrevStatus != buttonCurrStatus) && (buttonCurrStatus == BUTTON_LOGIC))
    {
      // Switch between internal and external ADC. Interrupt
      if(isInternalAdc == 1)
      {
        // Switch to external ADC
        SAADC_DisableIntADC();
        SAADC_EnableExtADC();
        ledExtADCSelPattern();

        isInternalAdc = 0;
      }
      else
      {
        // Switch to internal ADC
        SAADC_DisableExtADC();
        SAADC_EnableIntADC();
        ledIntADCSelPattern();

        isInternalAdc = 1;
      }

      adcRawDataCnt = 0;
      adcAvgDataCnt = 0;
    }

    buttonPrevStatus = buttonCurrStatus;

    k_sleep(K_MSEC(50));
  }
}

static void shutDnBtn(void* p1, void* p2, void* p3)
{
  uint8_t buttonCurrStatus = BUTTON_LOGIC;
  uint8_t buttonPrevStatus = BUTTON_LOGIC;

  while(1)
  {

    buttonCurrStatus = BTN_ShutdnStatus();

    if(buttonPrevStatus != buttonCurrStatus)
    {
      // Button pressed or released
      if(startShutdnTimer == 0)
      {
        // Button pressed; start timer
        // When timer runs out, shutdnTimerCb is called.
        k_timer_start(&shutdnTimer, K_MSEC(BTN_SHUTDN_DELAY), K_NO_WAIT);
        startShutdnTimer = 1;
      }
      else
      {
        // Button released; stop timer
        k_timer_stop(&shutdnTimer);
        startShutdnTimer = 0;
      }
    }

    buttonPrevStatus = buttonCurrStatus;

    k_sleep(K_MSEC(50));
  }
}

static uint16_t adcSampleAverage(uint16_t* array, size_t size)
{
  uint32_t avg = 0;

  for(size_t index = 0; index < size; index++)
  {
    avg = avg + array[index];
  }

  avg = avg / size;

  return (uint16_t)avg;
}

static void heartBeat(void* p1, void* p2, void* p3)
{
  void (*hbPattern)();

  while(1)
  {
    if(isLowBattery == 1)
    {
      hbPattern = ledLowBattery;
    }
    else
    {
      if(isInternalAdc == 1)
      {
        hbPattern = ledIntADCSelPattern;
      }
      else
      {
        hbPattern = ledExtADCSelPattern;
      }
    }

    hbPattern();
    k_sleep(K_SECONDS(5));
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

static void BT_SendData(void* p1, void* p2, void* p3)
{
  // Send averaged and compressed data
  uint32_t cnt = 0;
  volatile uint32_t value = 1000000;

  while(1)
  {
    while(cnt < value)
    {
      cnt++;
    }

    cnt = 0;

    k_thread_suspend(&thr_blueTooth);
  }
}

static void compressData(uint16_t* rawData, uint16_t* compressedData)
{
}

static void LED_Init()
{
  nrf_gpio_cfg_output(LED_STATUS);
  nrf_gpio_cfg_output(LED_BATT);
  LED_StatusOff();
  LED_BatteryOff();
}

static void BTN_Init()
{
  nrf_gpio_cfg_input(BTN_SHUTDN, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(BTN_ADC_CHG, NRF_GPIO_PIN_PULLUP);
}

static void GPIO_Init()
{
  DCDC_3V3SwitchOn();
  DCDC_5VSwitchOn();
  nrf_gpio_cfg_output(DCDC_SHTDN_3V3);
  nrf_gpio_cfg_output(DCDC_SHTDN_5V);
}

static void TIMER_Init()
{
  /* Init TIMER0 as timer */
  nrf_timer_bit_width_set(NRF_TIMER0, NRF_TIMER_BIT_WIDTH_32);
  nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0, SAMPLING_RATE);
  nrf_timer_int_enable(NRF_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);
  nrf_timer_shorts_set(NRF_TIMER0, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);

  /* Init TIMER1 as counter */
  nrf_timer_mode_set(NRF_TIMER1, NRF_TIMER_MODE_LOW_POWER_COUNTER);
  nrf_timer_bit_width_set(NRF_TIMER1, NRF_TIMER_BIT_WIDTH_8);
  nrf_timer_cc_set(NRF_TIMER1, NRF_TIMER_CC_CHANNEL0, RAW_DATA_SIZE);
  nrf_timer_int_enable(NRF_TIMER1, NRF_TIMER_INT_COMPARE0_MASK);
}

static void SPIM_Init()
{
  nrf_spim_configure(NRF_SPIM3, NRF_SPIM_MODE_2, NRF_SPIM_BIT_ORDER_MSB_FIRST);
  nrf_spim_pins_set(NRF_SPIM3, SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN);
  nrf_spim_csn_configure(NRF_SPIM3, SPI_CS_PIN, NRF_SPIM_CSN_POL_LOW, 2);
  nrf_spim_frequency_set(NRF_SPIM3, NRF_SPIM_FREQ_1M);
  nrf_spim_rx_buffer_set(NRF_SPIM3, rx_buff, BUFFER_SIZE);
  nrf_spim_tx_buffer_set(NRF_SPIM3, tx_buff, BUFFER_SIZE);
  nrf_spim_enable(NRF_SPIM3);
}

static void SAADC_Init()
{
  nrf_saadc_channel_config_t config;
  config.acq_time = NRF_SAADC_ACQTIME_40US;
  config.burst = NRF_SAADC_BURST_DISABLED;
  config.gain = NRF_SAADC_GAIN1_4;
  config.mode = NRF_SAADC_MODE_SINGLE_ENDED;
  config.reference = NRF_SAADC_REFERENCE_VDD4;
  config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
  config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;

  nrf_saadc_channel_init(NRF_SAADC, 0, &config);
  nrf_saadc_channel_input_set(NRF_SAADC, 0, NRF_SAADC_INPUT_AIN2, NRF_SAADC_INPUT_DISABLED);
  nrf_saadc_buffer_init(NRF_SAADC, (int16_t*)rx_buff, 1);
  nrf_saadc_resolution_set(NRF_SAADC, NRF_SAADC_RESOLUTION_12BIT);
  // nrf_saadc_int_enable(NRF_SAADC, NRF_SAADC_INT_END);
  nrf_saadc_enable(NRF_SAADC);
  nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_START);
}

static void LPCOMP_Init()
{
  nrf_lpcomp_config_t config;
  config.detection = NRF_LPCOMP_DETECT_CROSS;
  config.reference = NRF_LPCOMP_REF_SUPPLY_6_8;  // VDD * 12 / 16
  config.hyst = NRF_LPCOMP_HYST_ENABLED;

  nrf_lpcomp_configure(NRF_LPCOMP, &config);
  nrf_lpcomp_input_select(NRF_LPCOMP, NRF_LPCOMP_INPUT_5);
  nrf_lpcomp_int_enable(NRF_LPCOMP, NRF_LPCOMP_INT_CROSS_MASK);
  nrf_lpcomp_enable(NRF_LPCOMP);
  nrf_lpcomp_task_trigger(NRF_LPCOMP, NRF_LPCOMP_TASK_START);
}

static void BT_Init()
{
}

static void adcSamplingIRQ(const void* arg)
{
  // Called on sampling rate. Currently it's 1000 Hz

  uint8_t status = 0;
  uint16_t compoundData;

  if(isInternalAdc == 1)
  {
    nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_SAMPLE);
    do
    {
      status = nrf_saadc_event_check(NRF_SAADC, NRF_SAADC_EVENT_END);
    } while(status == 0);

    compoundData = (rx_buff[1] << 8) | rx_buff[0];

    // ABS value
    uint16_t tmp;
    tmp = compoundData >> 15;
    compoundData ^= tmp;
    compoundData += tmp & 1;
    adcRawData[adcRawDataCnt] = compoundData;

    nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_STARTED);
    nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_END);
    nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);
    nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_RESULTDONE);
    nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_START);
  }
  else
  {
    // Sample data from external ADC via SPI
    nrf_spim_task_trigger(NRF_SPIM3, NRF_SPIM_TASK_START);
    do
    {
      status = nrf_spim_event_check(NRF_SPIM3, NRF_SPIM_EVENT_END);
    } while(status == 0);

    compoundData = (rx_buff[0] << 12) | (rx_buff[1] << 4) | (rx_buff[2] >> 4);
    adcRawData[adcRawDataCnt] = compoundData;

    nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_END);
  }

  adcRawDataCnt++;
  nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_COUNT);
  nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0);
}

static void adcAvgIRQ(const void* arg)
{

  // Called when EasyDMA is full. Currently is on 10 samples - 100 Hz.

  adcRawDataCnt = 0;
  nrf_timer_event_clear(NRF_TIMER1, NRF_TIMER_EVENT_COMPARE0);
  nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);

  p_adcAvgData[adcAvgDataCnt] = adcSampleAverage(adcRawData, RAW_DATA_SIZE);
  adcAvgDataCnt++;

  if(adcAvgDataCnt == AVG_DATA_SIZE)
  {
    // Start BT transatction
    // k_thread_resume(&thr_blueTooth);
    // Change buffer
    if(p_adcAvgData == adcAvgData1)
    {
      p_adcAvgData = adcAvgData2;
    }
    else
    {
      p_adcAvgData = adcAvgData1;
    }

    adcAvgDataCnt = 0;
  }
}

static void checkBattStatus(const void* arg)
{
  uint8_t status;

  status = nrf_lpcomp_event_check(NRF_LPCOMP, NRF_LPCOMP_EVENT_DOWN);
  if(status == 1)
  {
    // Low battery
    isLowBattery = 1;
    SAADC_DisableExtADC();
    SAADC_DisableIntADC();
    k_thread_suspend(&thr_adcSwitchBtn);
  }

  status = nrf_lpcomp_event_check(NRF_LPCOMP, NRF_LPCOMP_EVENT_UP);
  if(status == 1)
  {
    // Battery is charging. No more low battery
    isLowBattery = 0;
    if(isInternalAdc == 1)
    {
      SAADC_EnableIntADC();
    }
    else
    {
      SAADC_EnableExtADC();
    }

    k_thread_resume(&thr_adcSwitchBtn);
  }

  nrf_lpcomp_event_clear(NRF_LPCOMP, NRF_LPCOMP_EVENT_DOWN);
  nrf_lpcomp_event_clear(NRF_LPCOMP, NRF_LPCOMP_EVENT_UP);
  nrf_lpcomp_event_clear(NRF_LPCOMP, NRF_LPCOMP_EVENT_CROSS);
}

static void kernelInit()
{
  int priority;

  /* Init threads but don't start them yet. */
  priority = 3;
  k_thread_create(&thr_ShutDnBtn, stk_ShutDnBtn, K_THREAD_STACK_SIZEOF(stk_ShutDnBtn), shutDnBtn, NULL, NULL, NULL, priority, 0, K_MSEC(3000));
  k_thread_name_set(&thr_ShutDnBtn, "Shutdown button");

  priority = 4;
  k_thread_create(&thr_adcSwitchBtn, stk_adcSwitchBtn, K_THREAD_STACK_SIZEOF(stk_adcSwitchBtn), adcSwitchBtn, NULL, NULL, NULL, priority, 0, K_MSEC(3020));
  k_thread_name_set(&thr_adcSwitchBtn, "ADC switch button");

  priority = 5;
  k_thread_create(&thr_heartBeat, stk_heartBeat, K_THREAD_STACK_SIZEOF(stk_heartBeat), heartBeat, NULL, NULL, NULL, priority, 0, K_MSEC(3040));
  k_thread_name_set(&thr_heartBeat, "Heartbeat");

  priority = 6;
  k_thread_create(&thr_blueTooth, stk_blueTooth, K_THREAD_STACK_SIZEOF(stk_blueTooth), BT_SendData, NULL, NULL, NULL, priority, 0, K_NO_WAIT);
  k_thread_name_set(&thr_blueTooth, "Bluetooth");
  k_thread_suspend(&thr_blueTooth);

  /* Init IRQs */
  priority = -15;
  irq_connect_dynamic(TIMER0_IRQn, priority, adcSamplingIRQ, NULL, 0);
  irq_enable(TIMER0_IRQn);

  priority = -14;
  irq_connect_dynamic(TIMER1_IRQn, priority, adcAvgIRQ, NULL, 0);
  irq_enable(TIMER1_IRQn);

  priority = -13;
  irq_connect_dynamic(LPCOMP_IRQn, priority, checkBattStatus, NULL, 0);
  irq_enable(LPCOMP_IRQn);

  /* Timer init */
  k_timer_init(&shutdnTimer, shutdnTimerCb, NULL);
  k_timer_init(&ledStartupBlink, ledStartupBlinkCb, NULL);
}

void main(void)
{
  /* System init */
  LED_Init();
  BTN_Init();
  GPIO_Init();
  TIMER_Init();
  SPIM_Init();
  SAADC_Init();
  LPCOMP_Init();
  // BT_Init();
  kernelInit();
  /* End of system init */

  isInternalAdc = 0;
  p_adcAvgData = adcAvgData1;

  // BT, ADC, LEDs, GPIOs everything is prepared
  // Begin LED start-up complete sequence
  k_timer_start(&ledStartupBlink, K_MSEC(100), K_MSEC(100));
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);

  // main is only for system init. Afterwards everyhing is done in threads.
}
