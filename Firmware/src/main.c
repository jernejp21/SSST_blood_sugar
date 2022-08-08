/**
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
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

#define NRF_DEFAULT_IRQ_PRIORITY 5

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_lpcomp.h>
#include <hal/nrf_power.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_saadc.h>
#include <hal/nrf_spim.h>
#include <hal/nrf_timer.h>

#include "IO.h"
#include "config.h"

#define BT_UUID_SSST_SERVICE_VAL BT_UUID_128_ENCODE(0x0179bbd0, 0x5351, 0x48b5, 0xbf6d, 0x2167639bc867)
#define BT_UUID_SSST_DATA_VAL BT_UUID_128_ENCODE(0x0179bbd1, 0x5351, 0x48b5, 0xbf6d, 0x2167639bc867)
#define BT_UUID_SSST_STATUS_VAL BT_UUID_128_ENCODE(0x0179bbd2, 0x5351, 0x48b5, 0xbf6d, 0x2167639bc867)

#define BT_UUID_SSST_SERVICE BT_UUID_DECLARE_128(BT_UUID_SSST_SERVICE_VAL)
#define BT_UUID_SSST_DATA BT_UUID_DECLARE_128(BT_UUID_SSST_DATA_VAL)
#define BT_UUID_SSST_STATUS BT_UUID_DECLARE_128(BT_UUID_SSST_STATUS_VAL)

#define BUFFER_SIZE 3

/* Variables */
volatile uint8_t isInternalAdc;
volatile uint8_t isLowBattery;
volatile uint8_t isNotifyEnabled;
static uint16_t adcAvgData1[AVG_DATA_SIZE];
static uint16_t adcAvgData2[AVG_DATA_SIZE];
static uint16_t adcRawData[RAW_DATA_SIZE] __attribute__((aligned(2)));
static uint16_t* p_adcAvgData;
static uint8_t rx_buff[BUFFER_SIZE] __attribute__((aligned(2)));
static uint8_t tx_buff[BUFFER_SIZE] __attribute__((aligned(2)));
static uint32_t adcRawDataCnt;
static uint32_t adcAvgDataCnt;
static uint8_t statusFlags;

/* Kernel variables */
static uint8_t startShutdnTimer;
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

K_SEM_DEFINE(bleSemaphor, 0, 1);

/* Bluetooth config */
void BT_Connected(struct bt_conn*, uint8_t);
void BT_Disconnected(struct bt_conn*, uint8_t);
ssize_t ssstStatusSend(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
ssize_t ssstDataRead(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buf, uint16_t len, uint16_t offset, uint8_t flags);

static const struct bt_data advert[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR))};

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = BT_Connected,
    .disconnected = BT_Disconnected,
};

BT_GATT_SERVICE_DEFINE(ssst_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_SSST_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_SSST_DATA,
                                              BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              NULL, ssstDataRead, NULL),
                       BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_SSST_STATUS,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              ssstStatusSend, NULL, NULL), );

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

static void resetBufferIndex()
{
  nrf_timer_event_clear(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE0);
  nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_CLEAR);
  adcRawDataCnt = 0;
  adcAvgDataCnt = 0;
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
      timerStopSampling();
      k_sem_take(&bleSemaphor, K_NO_WAIT);

      // Switch between internal and external ADC. Interrupt
      if(isInternalAdc == 1)
      {
        // Switch to external ADC
        SAADC_DisableIntADC();
        ledExtADCSelPattern();
        SAADC_EnableExtADC(rx_buff, tx_buff, BUFFER_SIZE);

        statusFlags &= ~(1 << FLAG_SENS_BIT);
        isInternalAdc = 0;
      }
      else
      {
        // Switch to internal ADC
        SAADC_DisableExtADC();
        ledIntADCSelPattern();
        SAADC_EnableIntADC();

        statusFlags |= (1 << FLAG_SENS_BIT);
        isInternalAdc = 1;
      }

      resetBufferIndex();
      if(isNotifyEnabled == 1)
      {
        nrf_timer_task_trigger(NRF_TIMER2, NRF_TIMER_TASK_START);
      }
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
  uint8_t* p_avg = &avg;

  for(size_t index = 0; index < size; index++)
  {
    avg = avg + array[index];
  }

  avg = avg / size;
  avg = (*p_avg << 8) | *(p_avg + 1);

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

void BT_Connected(struct bt_conn* conn, uint8_t err)
{
}

void BT_Disconnected(struct bt_conn* conn, uint8_t reason)
{
  timerStopSampling();
  k_sem_take(&bleSemaphor, K_NO_WAIT);
  resetBufferIndex();
}

ssize_t ssstStatusSend(struct bt_conn* conn,
                       const struct bt_gatt_attr* attr,
                       void* buf,
                       uint16_t len,
                       uint16_t offset)
{
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &statusFlags, sizeof(statusFlags));
}

ssize_t ssstDataRead(struct bt_conn* conn,
                     const struct bt_gatt_attr* attr,
                     const void* buf,
                     uint16_t len,
                     uint16_t offset,
                     uint8_t flags)
{
  const uint8_t* buf_val = buf;
  // Reverse byte order, to send the correct message value.
  uint16_t command = (buf_val[1] << 8) | buf_val[0];

  switch(command)
  {
  case 0x11AA:
    // Start reading data from sensor
    nrf_timer_task_trigger(NRF_TIMER2, NRF_TIMER_TASK_START);
    isNotifyEnabled = 1;
    bt_gatt_notify(NULL, &ssst_svc.attrs[1], &command, sizeof(command));
    break;

  case 0x12AA:
    // Stop reading data from sensor
    nrf_timer_task_trigger(NRF_TIMER2, NRF_TIMER_TASK_STOP);
    k_sem_take(&bleSemaphor, K_NO_WAIT);
    resetBufferIndex();
    isNotifyEnabled = 0;
    bt_gatt_notify(NULL, &ssst_svc.attrs[1], &command, sizeof(command));
    break;

  default:
    break;
  }

  return 0;
}

static void BT_SendData(void* p1, void* p2, void* p3)
{
  int error;

  while(1)
  {

    k_sem_take(&bleSemaphor, K_FOREVER);

    if(p_adcAvgData == adcAvgData1)
    {
      error = bt_gatt_notify(NULL, &ssst_svc.attrs[1], adcAvgData2, AVG_DATA_SIZE * sizeof(uint16_t));
    }
    else
    {
      error = bt_gatt_notify(NULL, &ssst_svc.attrs[1], adcAvgData1, AVG_DATA_SIZE * sizeof(uint16_t));
    }
  }
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
  nrf_gpio_cfg_input(BTN_SHUTDN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(BTN_ADC_CHG, NRF_GPIO_PIN_NOPULL);
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
  nrf_timer_bit_width_set(NRF_TIMER2, NRF_TIMER_BIT_WIDTH_32);
  nrf_timer_cc_set(NRF_TIMER2, NRF_TIMER_CC_CHANNEL0, SAMPLING_RATE);
  nrf_timer_int_enable(NRF_TIMER2, NRF_TIMER_INT_COMPARE0_MASK);
  nrf_timer_shorts_set(NRF_TIMER2, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);

  /* Init TIMER1 as counter */
  nrf_timer_mode_set(NRF_TIMER3, NRF_TIMER_MODE_LOW_POWER_COUNTER);
  nrf_timer_bit_width_set(NRF_TIMER3, NRF_TIMER_BIT_WIDTH_8);
  nrf_timer_cc_set(NRF_TIMER3, NRF_TIMER_CC_CHANNEL0, RAW_DATA_SIZE);
  nrf_timer_int_enable(NRF_TIMER3, NRF_TIMER_INT_COMPARE0_MASK);
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
  nrf_lpcomp_input_select(NRF_LPCOMP, NRF_LPCOMP_INPUT_1);
  nrf_lpcomp_int_enable(NRF_LPCOMP, NRF_LPCOMP_INT_CROSS_MASK);
  nrf_lpcomp_enable(NRF_LPCOMP);
  nrf_lpcomp_task_trigger(NRF_LPCOMP, NRF_LPCOMP_TASK_START);
}

static void BT_Init()
{
  int err;

  err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, advert, ARRAY_SIZE(advert), NULL, 0);
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
    tmp = tmp ? 0xFFFF : 0;
    compoundData ^= tmp;
    compoundData += tmp & 1;
    adcRawData[adcRawDataCnt] = compoundData << 4;  // This is 12-bit data. Shift 4 bits to left to get full-scale 16-bit.

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
  nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_COUNT);
  nrf_timer_event_clear(NRF_TIMER2, NRF_TIMER_EVENT_COMPARE0);
}

static void adcAvgIRQ(const void* arg)
{

  // Called when EasyDMA is full. Currently is on 10 samples - 100 Hz.

  adcRawDataCnt = 0;
  nrf_timer_event_clear(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE0);
  nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_CLEAR);

  p_adcAvgData[adcAvgDataCnt] = adcSampleAverage(adcRawData, RAW_DATA_SIZE);
  adcAvgDataCnt++;

  if(adcAvgDataCnt == AVG_DATA_SIZE)
  {
    // Start BT transatction
    k_sem_give(&bleSemaphor);

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
    statusFlags |= (1 << FLAG_BATT_BIT);
    SAADC_DisableExtADC();
    SAADC_DisableIntADC();
    k_thread_suspend(&thr_adcSwitchBtn);
  }

  status = nrf_lpcomp_event_check(NRF_LPCOMP, NRF_LPCOMP_EVENT_UP);
  if(status == 1)
  {
    // Battery is charging. No more low battery
    isLowBattery = 0;
    statusFlags &= ~(1 << FLAG_BATT_BIT);
    if(isInternalAdc == 1)
    {
      SAADC_EnableIntADC();
    }
    else
    {
      SAADC_EnableExtADC(rx_buff, tx_buff, BUFFER_SIZE);
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
  priority = 5;
  k_thread_create(&thr_ShutDnBtn, stk_ShutDnBtn, K_THREAD_STACK_SIZEOF(stk_ShutDnBtn), shutDnBtn, NULL, NULL, NULL, priority, 0, K_MSEC(3000));
  k_thread_name_set(&thr_ShutDnBtn, "Shutdown button");

  priority = 6;
  k_thread_create(&thr_adcSwitchBtn, stk_adcSwitchBtn, K_THREAD_STACK_SIZEOF(stk_adcSwitchBtn), adcSwitchBtn, NULL, NULL, NULL, priority, 0, K_NO_WAIT);
  k_thread_name_set(&thr_adcSwitchBtn, "ADC switch button");
  k_thread_suspend(&thr_adcSwitchBtn);

  priority = 15;
  k_thread_create(&thr_heartBeat, stk_heartBeat, K_THREAD_STACK_SIZEOF(stk_heartBeat), heartBeat, NULL, NULL, NULL, priority, 0, K_MSEC(3040));
  k_thread_name_set(&thr_heartBeat, "Heartbeat");

  priority = 8;
  k_thread_create(&thr_blueTooth, stk_blueTooth, K_THREAD_STACK_SIZEOF(stk_blueTooth), BT_SendData, NULL, NULL, NULL, priority, 0, K_NO_WAIT);
  k_thread_name_set(&thr_blueTooth, "Bluetooth");

  /* Init IRQs */
  priority = -3;
  irq_connect_dynamic(TIMER2_IRQn, priority, adcSamplingIRQ, NULL, 0);
  irq_enable(TIMER2_IRQn);

  priority = -2;
  irq_connect_dynamic(TIMER3_IRQn, priority, adcAvgIRQ, NULL, 0);
  irq_enable(TIMER3_IRQn);

  priority = -4;
  irq_connect_dynamic(LPCOMP_IRQn, priority, checkBattStatus, NULL, 0);
  irq_enable(LPCOMP_IRQn);

  /* Timer init */
  k_timer_init(&shutdnTimer, shutdnTimerCb, NULL);
  k_timer_init(&ledStartupBlink, ledStartupBlinkCb, NULL);
}

static void checkStartupBattery()
{
  uint8_t status = 0;

  /* Enable SAADC on AIN5 */
  nrf_saadc_channel_config_t config;
  config.acq_time = NRF_SAADC_ACQTIME_40US;
  config.burst = NRF_SAADC_BURST_DISABLED;
  config.gain = NRF_SAADC_GAIN1_4;
  config.mode = NRF_SAADC_MODE_SINGLE_ENDED;
  config.reference = NRF_SAADC_REFERENCE_VDD4;
  config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
  config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;

  nrf_saadc_channel_init(NRF_SAADC, 0, &config);
  nrf_saadc_channel_input_set(NRF_SAADC, 0, NRF_SAADC_INPUT_AIN1, NRF_SAADC_INPUT_DISABLED);
  nrf_saadc_buffer_init(NRF_SAADC, (int16_t*)rx_buff, 1);
  nrf_saadc_resolution_set(NRF_SAADC, NRF_SAADC_RESOLUTION_12BIT);
  nrf_saadc_enable(NRF_SAADC);

  /* SAADC calibration */
  nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_CALIBRATEOFFSET);
  do
  {
    status = nrf_saadc_event_check(NRF_SAADC, NRF_SAADC_EVENT_CALIBRATEDONE);
  } while(status == 0);

  /* Start SAADC */
  nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_START);
  do
  {
    status = nrf_saadc_event_check(NRF_SAADC, NRF_SAADC_EVENT_STARTED);
  } while(status == 0);

  /* Sample */
  nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_SAMPLE);
  do
  {
    status = nrf_saadc_event_check(NRF_SAADC, NRF_SAADC_EVENT_END);
  } while(status == 0);

  int16_t result = (rx_buff[1] << 8) | rx_buff[0];
  float voltage = SUPPLY_VOLTAGE * (float)result / 4096;

  if(voltage > BATT_MIN_VOLT)
  {
    isInternalAdc = 0;
    isLowBattery = 0;
    statusFlags &= ~(1 << FLAG_BATT_BIT);
    SAADC_DisableIntADC();
    SAADC_EnableExtADC(rx_buff, tx_buff, BUFFER_SIZE);
    k_thread_resume(&thr_adcSwitchBtn);
  }
  else
  {
    isLowBattery = 1;
    statusFlags |= (1 << FLAG_BATT_BIT);
    SAADC_DisableExtADC();
    SAADC_DisableIntADC();
    k_thread_suspend(&thr_adcSwitchBtn);
  }

  nrf_saadc_disable(NRF_SAADC);
}

void main(void)
{
  int err;

  /* System init */
  LED_Init();
  BTN_Init();
  GPIO_Init();
  TIMER_Init();
  SPIM_Init();
  LPCOMP_Init();
  kernelInit();
  /* End of system init */

  /* Check battery status */
  checkStartupBattery();
  SAADC_Init();

  p_adcAvgData = adcAvgData1;

  /* Begin LED start-up complete sequence */
  k_timer_start(&ledStartupBlink, K_MSEC(100), K_MSEC(100));

  /* Start BT advertising */
  err = bt_enable(BT_Init);

  // main is only for system init. Afterwards everyhing is done in threads.
}