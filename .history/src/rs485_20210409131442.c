#include "rs485.h"

#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <math.h>
#include <stdint.h>

#include "adc.h"
#include "crc.h"
#include "esp_system.h"
#include "global.h"
#include "ntc.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "struckt.h"
#include "timer.h"

//#define forceCooldown 1
//#define forceSpeedStart 1

#define dummyPinrx \
  36  // this pin is not used and can be used to point to if no data reception
      // is needed
#define dummyPintx \
  34  // this pin is not used and can be used to point to if no data
      // transmission is needed
//#define PACKET_READ_TICS (100 / portTICK_RATE_MS)

/*********************************************************************
 * Pinout ESP32 RS485 port
 * IO_RS485[DIR][DATA]
 **********************************************************************/
uint8_t IO_RS485[6][2] = {{25, 27}, {14, 4}, {12, 16},
                          {13, 17}, {15, 5}, {0, 18}};  //{DIR,DATA}
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define BUF_SIZE (100)
#define ECHO_READ_TOUT (3)  // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
#define port0 0
#define port1 1
#define port2 2
#define port3 3
#define port4 4
#define port5 5

/*********************************************************************
 * Kabel400 defines
 **********************************************************************/

#define purifyvalve 0x40  // valve purify
#define pump 0x20         // pump   valve purify
#define drain 0x08        // drain
#define extvent 0x10      // swing

#define STORAGE_NAMESPACE "storage"
#define TAG "RS485"

static char writb[20];
static uint8_t readb[20];
static int f;
static int LowPressureTimer = 0;

volatile float hum_FG6485[8];
volatile float tmp_FG6485[8];
volatile double TMP, HUM;
int rs485status;
volatile int nrFG645 = 0;
volatile bool CompressorOn = false;
volatile float FanError = 0;
volatile float PumpError = 0;
volatile uint16_t CompressorOnTime = 0;
static bool HedyFan = false;
static bool HedyPump = false;
uint8_t rotate = 1;
uint16_t HedySetRPM = 0;
static int HedyRPM = 0;
// static int HedyTMP = 0;
static int HedyCUR = 0;
static int HedyVCC = 0;
static int HedyPWR = 0;

/*********************************************************************/
/* RS 485 commands */
/* length , data , ... */
/**********************************************************************/
char FG645rdTMP[] = {6, 0x01, 0x03, 0x00, 0x00, 0x00, 0x02};
char HedyInitialise[] = {9,    0x01, 0x10, 0x00, 0x06,
                         0x00, 0x01, 0x02, 0x17, 0x70};
char HedyKeyPad[] = {6,    0x01, 0x06, 0x00,
                     0x00, 0x00, 0x01};  // p00.00 Auto switchmode
char HedyDspMode[] = {6, 0x01, 0x06, 0x00, 0x01, 0x00, 0x01};  // p00.01 Dsp
                                                               // mode
char HedyCntrMode[] = {6,    0x01, 0x06, 0x00,
                       0x02, 0x00, 0x02};  // p00.03 Control mode
char HedySource[] = {6,    0x01, 0x06, 0x00,
                     0x03, 0x00, 0x05};  // p00.04 Auto source control
char HedyRunOn[] = {6, 0x01, 0x06, 0x01, 0x3F, 0x00, 0x01};  // p03.20
char HedyControlOff[] = {6,    0x01, 0x06, 0x01,
                         0x46, 0x00, 0x00};  // p03.27 Control word
char HedyControlOn[] = {6,    0x01, 0x06, 0x01,
                        0x46, 0x00, 0x02};  // p03.27 Control word
char HedyControlEnable[] = {6,    0x01, 0x06, 0x01,
                            0x47, 0x00, 0x01};  // p03.28 Control word enable
char HedyRunOff[] = {6, 0x01, 0x06, 0x01, 0x3F, 0x00, 0x00};
char HedyRpm0[] = {6, 0x01, 0x06, 0x00, 0x05, 0x00, 0x00};  // p00.06
char HedyRpmMax[] = {6,    0x01, 0x06, 0x00,
                     0x06, 0x13, 0x88};  // p00.07 = 50000 (50Hz)
char HedyRpmAct[] = {6, 0x01, 0x03, 0x01, 0xFE, 0x00, 0x01};  // p05.11 = 0.01Hz
char HedyVccAct[] = {6, 0x01, 0x03, 0x01, 0xFF, 0x00, 0x01};  // p05.12 = 1V
char HedyCurAct[] = {6, 0x01, 0x03, 0x02, 0x01, 0x00, 0x01};  // p05.14 = 0.1A
char HedypwrAct[] = {6, 0x01, 0x03, 0x02, 0x08, 0x00, 0x01};  // p05.21 =
                                                              // 0.01kWh
char HedyTmpAct[] = {6, 0x01, 0x03, 0x02, 0x0C, 0x00, 0x01};  // p05.25 = 1C
char HedySoftVersion[] = {6, 0x01, 0x03, 0x02, 0x10, 0x00, 0x01};
char HedyModelVersion[] = {6, 0x01, 0x03, 0x03, 0xF4, 0x00, 0x01};

// char HedyAutotuneFan[]    		= {6,0x01,0x06,0x00,0x10,0x00,0x00};
// //p00.17 = 0

char HedyMotorVoltageFan[] = {6,    0x01, 0x06, 0x00,
                              0x0C, 0x00, 0xE6};  // p00.13 = 230V
char HedyMotorCurrentFan[] = {6,    0x01, 0x06, 0x00,
                              0x0D, 0x00, 0x24};  // p00.14 = 3.6A
char HedyMotorSpeedFan[] = {6,    0x01, 0x06, 0x00,
                            0x0E, 0x13, 0x88};  // p00.15 = 50Hz
char HedyAutotuneFan[] = {6, 0x01, 0x06, 0x00, 0x10, 0x00, 0x00};  // p00.17 = 2
char HedyMVCMFan[] = {6,    0x01, 0x06, 0x00,
                      0x13, 0x00, 0x03};  // p00.20 = 3 single phase with cap
char HedySFFan[] = {6, 0x01, 0x06, 0x00, 0x14, 0x00, 0x0B};  // p00.21 = 11
char HedyOutputPhaseFan[] = {6,    0x01, 0x06, 0x04,
                             0xB9, 0x00, 0x01};  // p12.10 = 1

char HedyMotorVoltagePump[] = {6,    0x01, 0x06, 0x00,
                               0x0C, 0x00, 0xE6};  // p00.13 = 230V
char HedyMotorCurrentPump[] = {6,    0x01, 0x06, 0x00,
                               0x0D, 0x00, 0x3B};  // p00.14 = 5.9A
char HedyMotorSpeedPump[] = {6,    0x01, 0x06, 0x00,
                             0x0E, 0x13, 0x88};  // p00.15 = 50Hz
char HedyAutotunePump[] = {6, 0x01, 0x06, 0x00, 0x10, 0x00, 0x00};  // p00.17 =
                                                                    // 0
char HedyMVCMPump[] = {
    6,    0x01, 0x06, 0x00,
    0x13, 0x00, 0x03};  // p00.20 = 0   //4 single phase with cap
char HedySFPump[] = {6,    0x01, 0x06, 0x00,
                     0x14, 0x00, 0x0D};  // p00.21 = 13//   11
char HedyOutputPhasePump[] = {6,    0x01, 0x06, 0x04,
                              0xB9, 0x00, 0x01};  // p12.10 = 1

// char HedyMotorVoltagePump[]	= {6,0x01,0x06,0x00,0x0C,0x00,0xE6};
// //p00.13 = 230V char HedyMotorCurrentPump[]	=
// {6,0x01,0x06,0x00,0x0D,0x00,0x24};	//p00.14 = 3.6A char
// HedyMotorSpeedPump[]  	= {6,0x01,0x06,0x00,0x0E,0x13,0x88};	//p00.15
// = 50Hz char HedyAutotunePump[]    	= {6,0x01,0x06,0x00,0x10,0x00,0x00};
// //p00.17 = 2
// char HedyMVCMPump[]        	= {6,0x01,0x06,0x00,0x13,0x00,0x00};
// //p00.20 = 0 single phase with cap char HedySFPump[]          	=
// {6,0x01,0x06,0x00,0x14,0x00,0x11};	//p00.21 = 11 char HedyOutputPhasePump[]
// = {6,0x01,0x06,0x04,0xB9,0x00,0x00};	//p12.10 = 1

/*
Hoi Peter,
Voor de Hedy de volgende instellingen:
Begin met load default
p00.24 = 1
P00.13=230V
P00.14=3,6A
P00.15=50
P00.17=2
P00.20=3
P00.21=11
P12.10=1
*/
/*********************************************************************/
/* Setup output */
/**********************************************************************/
void init_RS485(uint port) {
  ESP_LOGI(TAG, "Port %d setup ", port + 1);
  gpio_pad_select_gpio(IO_RS485[port][0]);
  gpio_set_direction(IO_RS485[port][0], GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(IO_RS485[port][1]);
  gpio_set_direction(IO_RS485[port][1], GPIO_MODE_INPUT);
  gpio_set_level(IO_RS485[0][0], 0);  // set RX mode
}

void init_UART_RS485(const int uart, uint32_t speed) {
  uart_config_t uart_config = {.baud_rate = speed,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_param_config(uart, &uart_config);
  uart_driver_install(uart, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void init_UART_RS485_hedy(const int uart, uint32_t speed) {
  uart_config_t uart_config = {.baud_rate = speed,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_2,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_param_config(uart, &uart_config);
  uart_driver_install(uart, BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void uart_write_bytes_RS485(const int uart, uint port, const char *str,
                                   uint8_t length) {
  gpio_set_level(IO_RS485[port][0], 1);  // set high TX mode
  uart_set_pin(uart, IO_RS485[port][1], dummyPinrx, ECHO_TEST_RTS,
               ECHO_TEST_CTS);
  if (uart_write_bytes(uart, str, length) != length) {
    ESP_LOGE(TAG, "Send data critical failure.");
    uart_set_pin(uart, dummyPintx, IO_RS485[port][1], ECHO_TEST_RTS,
                 ECHO_TEST_CTS);
    gpio_pad_select_gpio(IO_RS485[port][1]);
    gpio_set_direction(IO_RS485[port][1], GPIO_MODE_INPUT);
    gpio_set_level(IO_RS485[port][0], 0);  // set low RX mode
    abort();
  }
  uart_wait_tx_done(uart, 10);
  uart_flush(uart);
  gpio_set_level(IO_RS485[port][0], 0);  // set low RX mode
  uart_set_pin(uart, IO_RS485[2][1], IO_RS485[port][1], ECHO_TEST_RTS,
               ECHO_TEST_CTS);
}

/*
 * preparing data buffer.
 * calculate checksum and add it at the end of the buffer
 * at position 0 the total length will be filled in
 * position 0 contains the original length
 */
char prepair_buf(char *bout, char *data) {
  for (int t = 0; t < data[0]; t++) {
    bout[t] = data[t + 1];
  }
  OutCrc16(bout, data[0]);
  return data[0] + 2;
}

bool initHedyFan(int uart, int port) {
  bool ret = false;
  // ESP_LOGI(TAG, "Searching for Hedy inverter Fan");
  uart_write_bytes_RS485(uart, port, writb,
                         prepair_buf(writb, HedyModelVersion));
  uart_flush_input(uart);
  f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
  if (InCrc16((char *)readb, f)) {
    int Hedy = readb[3] << 8 | readb[4];
    ESP_LOGI(TAG, "Hedy inverter Fan Found! Model: %d ", Hedy);
    if (InCrc16((char *)readb, f)) {
      int Hedy = readb[3] << 8 | readb[4];
      ESP_LOGI(TAG, "Hedy inverter Found! Softwareversion: %d ", Hedy);
    }

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyInitialise));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    // uart_write_bytes_RS485(uart,port,writb, prepair_buf(writb,
    // HedyCntrMode)); uart_flush_input(uart); f = uart_read_bytes(uart,(uint8_t
    // *)readb, 8, 10); vTaskDelay(100);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedySource));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyRpmMax));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyRpm0));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyMotorVoltageFan));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyMotorCurrentFan));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyMotorSpeedFan));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyAutotuneFan));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyMVCMFan));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedySFFan));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyOutputPhaseFan));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyControlEnable));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyDspMode));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyCntrMode));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyKeyPad));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);
    ret = true;

    // Done with init Hedy inverter
  }
  return ret;
}

bool initHedyPump(int uart, int port) {
  bool ret = false;
  // ESP_LOGI(TAG, "Searching for Hedy inverter Pump");
  uart_write_bytes_RS485(uart, port, writb,
                         prepair_buf(writb, HedyModelVersion));
  uart_flush_input(uart);
  f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
  if (InCrc16((char *)readb, f)) {
    int Hedy = readb[3] << 8 | readb[4];
    ESP_LOGI(TAG, "Hedy inverter Pump Found! Model: %d ", Hedy);
    if (InCrc16((char *)readb, f)) {
      int Hedy = readb[3] << 8 | readb[4];
      ESP_LOGI(TAG, "Hedy inverter Found! Softwareversion: %d ", Hedy);
    }

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedySource));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyRpmMax));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyRpm0));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyMotorVoltagePump));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyMotorCurrentPump));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyMotorSpeedPump));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyAutotunePump));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyMVCMPump));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedySFPump));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyOutputPhasePump));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedyControlEnable));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyDspMode));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyCntrMode));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyKeyPad));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);
    ret = true;
    // Done with init Hedy inverter
  }
  return ret;
}

/*********************************************************************
 * FG485 stuff
 **********************************************************************/
bool FG6485_Temp_RH_task(int uart, int port) {
  uint8_t readb[20];
  int l = prepair_buf(writb, FG645rdTMP);              // load command
  uart_write_bytes_RS485(uart, port, writb, l);        // send command
  f = uart_read_bytes(uart, (uint8_t *)readb, 9, 10);  // get response
  if (InCrc16((char *)readb, f)) {
    hum_FG6485[port] =
        ((float)(readb[3] << 8 | readb[4])) / 10;  // extract humidity
    tmp_FG6485[port] =
        ((float)(readb[5] << 8 | readb[6])) / 10;  // extract temperature
    // ESP_LOGI(TAG, "FG6485 port%d temp: %02.2f, FG6485 hum:
    // %02.2f",port,tmp_FG6485,hum_FG6485);
    return 0;  // return 1 on success
  } else {
    // ESP_LOGI("RS485", "No data from FG6485 datasize:%d",f);
    hum_FG6485[port] = NAN;  // extract humidity
    tmp_FG6485[port] = NAN;  // extract temperature
    return 1;                // no new data found
  }
}

/*********************************************************************
 * Humidifier stuff
 **********************************************************************/
void RS487_humidifier_task(int uart, int port) {
  uint8_t l = 0;
  TickType_t timestamp = xTaskGetTickCount();
  TickType_t timeout = xTaskGetTickCount();

  ESP_LOGI(TAG, "Search Humidifier 1");
  while (timeout + pdMS_TO_TICKS(1500) > xTaskGetTickCount()) {
    if (timestamp + pdMS_TO_TICKS(500) < xTaskGetTickCount()) {
      timestamp = xTaskGetTickCount();
      if (WeifangMuhe.fanspeed < 0x0b) WeifangMuhe.fanspeed = 0x0b;
      if (WeifangMuhe.fanspeed > 0x16) WeifangMuhe.fanspeed = 0x16;
      writb[0] = (0xaa);
      if (WeifangMuhe.fanOn)
        writb[1] = 0x12;
      else
        writb[1] = 0x02;
      writb[2] = WeifangMuhe.fanspeed;
      uart_write_bytes_RS485(uart, port, writb, 3);  // send command
    }
    l = uart_read_bytes(uart, (uint8_t *)readb, 5, 1);  // get response
    if (l >= 4) {
      if (readb[0] == 0xAA && readb[1] < 100) {
        WeifangMuhe.temp_set = readb[1];
        WeifangMuhe.current = (float)readb[4] / 10;
        // ESP_LOGI(TAG, "New data form humidifier: temp: %02.2f, Current
        // %02.2f",WeifangMuhe.temp_set ,WeifangMuhe.current);
        timeout = xTaskGetTickCount();
      }
      for (int t = 0; t < l; t++) {
        printf("%x", readb[t]);
      }
      uart_flush_input(UART_NUM_1);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/*only for a simple test*/
static float Previous_ErrorHum = 0;
static float Previous_ErrorPumpFan = 0;
static float Previous_ErrorPumpPump = 0;
static bool runOnceTestPidHum = false;
static bool runOnceTestPidPumpFan = false;
static bool runOnceTestPidPumpPump = false;
static float Error_max;
static float Error_Integral, Error_Derivative;

/*********************************************************************
 * PID Fan part FanPumpbox
 **********************************************************************/
void PumpFanPid(void) {
  float PID_Output;
  float Error, Errorp = 0;
  if (xSemaphoreTake(xSemaphoreSTR, 10 == pdTRUE)) {
    if (!runOnceTestPidPumpFan) {
      runOnceTestPidPumpFan = true;
      Fan.iErrorfan = 0;
      Previous_ErrorPumpFan = 0;
      Fan.dEerrorfan = 0;
    }
    if (xSemaphoreTake(xSemaphoreNTC, 10 == pdTRUE)) {
      if (isnan(NTC[ToutPos]) || isnan(NTC[ToutPos])) {
        Error = 0;
        Errorp = 0;
        // Fan.iErrorpump = 0;
        // ESP_LOGE("RS485", "NTC nan!f");
      } else {
        Error = NTC[ToutPos] - Fan.Tout;
        FanError = Error;
        Errorp = (NTC[TinPos] - Fan.Tout - Pump.TempDelta);
        if (Errorp < 0) {
          Errorp = 0;
        }
      }
      xSemaphoreGive(xSemaphoreNTC);
    } else {
      xSemaphoreGive(xSemaphoreSTR);
      return;
    }
    Fan.iErrorfan = Fan.iErrorfan + Error / 60;
    if (Fan.iErrorfan * Fan.Ifan > 90)  // limit intergrator
      Fan.iErrorfan = 90 / Fan.Ifan;
    // else if(Fan.iErrorfan*Fan.Ifan < -10)
    // Fan.iErrorfan = 10/Fan.Ifan*-1;
    else if (Fan.iErrorfan * Fan.Ifan < 0)
      Fan.iErrorfan = 0;

    Fan.dEerrorfan = Error - Previous_ErrorPumpFan;
    Previous_ErrorPumpFan = Error;
    PID_Output = (Fan.Pfan / 10 * Error) + (Fan.Ifan * Fan.iErrorfan) +
                 (Fan.Dfan * Fan.dEerrorfan) + Errorp * 5;

    if (PID_Output < 0) PID_Output = 0;
    if (PID_Output > Fan.speedmaximalfan) {
      PID_Output = Fan.speedmaximalfan;
    }
    Fan.speedfan = PID_Output;
    xSemaphoreGive(xSemaphoreSTR);
    // ESP_LOGI(TAG, "P:%06.3f    I:%06.3f     D:%06.3f        error:%5.2f",
    // (Fan.Pfan*Error),(Fan.Ifan*Fan.iErrorfan),(Fan.Dfan*Fan.dEerrorfan),Error);
  }
}

/*********************************************************************
 * PID Pump part FanPumpbox
 **********************************************************************/
void PumpPumpPid(void) {
  float PID_Output;
  float Error = 0;
  if (xSemaphoreTake(xSemaphoreSTR, 10 == pdTRUE)) {
    if (!runOnceTestPidPumpPump) {
      runOnceTestPidPumpPump = true;
      //	Error_IntegralPumpPump = 0;
      //	Previous_ErrorPumpPump = 0;
      //	Pump.dEerrorpump = 0;
      //	Pump.iErrorpump = 99/(Pump.Ipump);
    }
    if (xSemaphoreTake(xSemaphoreNTC, 10 == pdTRUE)) {
      if (isnan(NTC[TinPos]) || isnan(NTC[ToutPos])) {
        Error = 0;
        // ESP_LOGE("RS485", "NTC nan!f");
      } else {
        Error = NTC[TinPos] - Fan.Tout - Pump.TempDelta;
        // Error = (NTC[TinPos] - NTC[ToutPos]) - Pump.TempDelta;
        PumpError = Error;
      }
      xSemaphoreGive(xSemaphoreNTC);
    } else {
      xSemaphoreGive(xSemaphoreSTR);
      return;  // No valid data
    }

    Pump.iErrorpump = Pump.iErrorpump + Error / 60;

    if (Pump.iErrorpump * (Pump.Ipump) > 99)  // limit intergrator
      Pump.iErrorpump = 99 / (Pump.Ipump);
    else if (Pump.iErrorpump * (Pump.Ipump) < -10)
      Pump.iErrorpump = 10 / (Pump.Ipump) * -1;

    Pump.dEerrorpump = Error - Previous_ErrorPumpPump;
    Previous_ErrorPumpPump = Error;
    PID_Output = ((Pump.Ppump / 10) * Error) +
                 ((Pump.Ipump) * Pump.iErrorpump) +
                 ((Pump.Dpump / 10) * Pump.dEerrorpump);
    if (PID_Output < 0) PID_Output = 0;
    if (PID_Output > Pump.speedmaximalpump) {
      PID_Output = Pump.speedmaximalpump;
    }
    Pump.speedpump = PID_Output;
    xSemaphoreGive(xSemaphoreSTR);
    // ESP_LOGI(TAG, "Tint:%04.2fC Pump:%04.2f P:%06.3f I:%06.3f D:%06.3f
    // error:%5.2f",NTC[TinPos], Pump.speedpump,
    // ((Pump.Ppump/10)*Error),((Pump.Ipump)*Pump.iErrorpump),((Pump.Dpump/10)*Pump.dEerrorpump),Error);
  }
}

/*********************************************************************
 * PID part Humidifier
 **********************************************************************/
void HumiPid(void) {
  float PID_Output = 0;
  float Error = 0;
  float Acttemp;

  if (!runOnceTestPidHum) {
    runOnceTestPidHum = true;
    Error_Integral = 0;
    Error_max = 1;
    Previous_ErrorHum = 0;
  }
  if (xSemaphoreTake(xSemaphoreNTC, 10 == pdTRUE)) {
    if (isnan(hum_FG6485[1])) {
      ESP_LOGE("RS485", "FG6485 Error -> nan!");
      xSemaphoreGive(xSemaphoreNTC);
      return;
    } else {
      if (hum_FG6485[1] == 0 && tmp_FG6485[1] == 0) {
        ESP_LOGE("RS485",
                 "FG6485 Temp=0 and RH=0 probably still intialising!!");
        xSemaphoreGive(xSemaphoreNTC);
        return;
      }
      Acttemp = hum_FG6485[1];
    }
  } else {
    return;
  }
  xSemaphoreGive(xSemaphoreNTC);
  if (xSemaphoreTake(xSemaphoreSTR, 10 == pdTRUE)) {
    if (Kabel400.ldr == true && DayNight == false) {
      Error = Kabel400.RH_night - Acttemp;
    } else {
      Error = Kabel400.RH_day - Acttemp;
    }

    Error_Integral = Error_Integral + Error * 0.1;

    if (Error_Integral > Error_max)  // limit intergrator
      Error_Integral = Error_max;
    else if (Error_Integral < -Error_max)
      Error_Integral = -Error_max;
    // if(Error <= 0)
    //	Error_Integral = 0;

    Error_Derivative = Error - Previous_ErrorHum;
    Previous_ErrorHum = Error;
    PID_Output = (Kabel400.P * Error) + (Kabel400.I * Error_Integral) +
                 (Kabel400.D * Error_Derivative);
    if (PID_Output < 0) PID_Output = 0;
    // ESP_LOGI(TAG, "P:%02.2f I:%02.2f PID:%02.2f RH_set:%02.2f
    // RS_act:%02.1f",Kabel400.P*Error,Kabel400.I*Error_Integral,PID_Output,
    // RH_set,hum_FG6485);
    Kabel400.fanspeed = PID_Output;
    xSemaphoreGive(xSemaphoreSTR);
  }
  return;
}

/*********************************************************************
 * Humidiefier
 * this loop will be executed once a second!
 * Controls a inverter trough RS485
 * aditional relais can be controled
 * 1 Drain
 * 2 Pump
 * 3 External valve
 * 4 not used
 **********************************************************************/
void humidifierKabel400(int uart, int port) {
  writb[0] = (0x08);
  writb[1] = (0x50);
  writb[2] = (0x00);
  HumiPid();
  if (xSemaphoreTake(xSemaphoreSTR, 100 == pdTRUE)) {
    if (Kabel400.fanManual == true) {
      Kabel400.fanspeed = Kabel400.fanspeedmanual;
    }
    if (Kabel400.fanspeed > 0) {  // fan should be turned on
      Kabel400.pumpOn = true;     // Okay so turn on pump
      Kabel400.pumpTime++;        // increase pump timer
      Kabel400.fanOn = true;      // fan on indicator
      Kabel400.drainOn = false;   // close drain
      Kabel400.offTime = 0;
      Kabel400.extventOn = true;
      // water level protection
      if (Kabel400.floatsensor == 0xFF) {  // Check water level
        Kabel400.floatsensorError = 0;     // reset water low alarm timeout
      } else if (Kabel400.floatsensorError++ >
                 60) {  // increase timeout timer and check if more than 60
                        // second are passed
        Kabel400.pumpOn = false;  // if so turn off pump
      }
    } else {
      Kabel400.pumpOn = false;
      Kabel400.fanOn = false;
      Kabel400.fanspeed = 0;  // Stop fan
      Kabel400.fanspeedinverter = 0;
      Kabel400.offTime++;
      if (Kabel400.floatsensor == 0xFF) {  // Check water level
        Kabel400.extventOn = false;        // close external valve
      }
    }

    // flush on timeout;
    if (Kabel400.onTime >= Kabel400.cleantimeset * 60 * 60) {
      Kabel400.pumpOn = false;        // if so turn off pump
      Kabel400.drainOn = true;        // open drain
      Kabel400.floatsensorError = 0;  // reset water low timeout
      Kabel400.extventOn = false;     // close external valve
      if (Kabel400.onTime >= Kabel400.cleantimeset * 60 * 60 + 60 * 5) {
        Kabel400.onTime = 0;
        Kabel400.drainOn = false;  // close drain
      }
    }
    // flush and do not refill
    if (Kabel400.offTime >= 60 * 60 * 3) {
      Kabel400.drainOn = true;     // empty buffer afer 60 sec * 60 3 (3 hours)
      Kabel400.extventOn = false;  // close external valve
      Kabel400.floatsensorError = 0;  // reset water low timeout
      if (Kabel400.offTime >=
          60 * 60 * 3 + 60 * 5) {  // close drain afer 60 sec *60 * 3(3 hours)
                                   // + 5*60 (and 5 min)
        Kabel400.drainOn = false;  // close drain after 10 min again
      }
    }

    // Check minimum speed limitation
    if (Kabel400.fanspeed > 0) {
      Kabel400.fanspeedinverter = Kabel400.fanspeed;
      // minimal fan speed check
      if (Kabel400.fanspeedminimal > 0) {  // fan should be turned on
        if (Kabel400.fanspeed < Kabel400.fanspeedminimal) {
          Kabel400.fanspeedinverter = Kabel400.fanspeedminimal;
        }
      }
    }

    /*
    //start circulate air after timeout
    //if circulatespeed is 0 fan keeps turned off
    //else the fan will speed up to the value set in to circulate speed
    */
    if (Kabel400.pumpOn == false) {
      if (Kabel400.circulateTimer * 60 > Kabel400.circulateTime)
        Kabel400.circulateTime++;
      else {
        Kabel400.fanspeedinverter = Kabel400.circulateSpeed;
        Kabel400.fanspeed = Kabel400.circulateSpeed;
      }
    } else {
      Kabel400.circulateTime = 0;
    }

    // Max speed limitation
    if (Kabel400.fanspeed > Kabel400.fanspeedmaximal) {
      Kabel400.fanspeedinverter = Kabel400.fanspeedmaximal;
    }

    // Water leakage detection
    if (WaterAlarm) {
      Kabel400.extventOn = false;  // close external valve
      Kabel400.pumpOn = false;     // if so turn off pump
    }

    // write to inverter
    writb[3] = (int)Kabel400.fanspeedinverter * 60 / 100;  // write to buffer
    writb[4] = Kabel400.pumpOn * pump + Kabel400.drainOn * drain +
               Kabel400.extventOn * extvent +
               Kabel400.purifyvalveOn * purifyvalve;
    writb[5] = 1 + writb[0] + writb[1] + writb[2] + writb[3] +
               writb[4];                             // calculate checksum
    uart_write_bytes_RS485(uart, port, writb, 6);    // send command
    uart_read_bytes(uart, (uint8_t *)readb, 9, 50);  // get response
    if (readb[0] == 0x08 && readb[1] == 0x51) {
      Kabel400.connected = true;
      Kabel400.floatsensor =
          readb[5];  // extract aqua level 0xFF means full 0xB4 means empty
      // ESP_LOGI(TAG, "Fan speed %02f RH_set:%02.1f
      // RS_act:%02.1f",Kabel400.fanspeed,Kabel400.RH_set,hum_FG6485 );
      Kabel400.onTime++;
    } else {
      Kabel400.connected = false;
      Kabel400.floatsensor =
          0XFF;  // extract aqua level 0xFF means full 0xB4 means empty
    }
    xSemaphoreGive(xSemaphoreSTR);
  }
}

void write(const int uart, uint port, char *buf, int len) {
  uart_write_bytes_RS485(uart, port, buf, len);
}

void hedy_set_rpm(const int uart, uint port, uint32_t rpm) {
  char buf[10];
  if (rpm > 5000) {
    rpm = 5000;
  }
  for (int t = 0; t < HedyRpm0[0]; t++) {
    buf[t] = HedyRpm0[t + 1];
  }
  buf[4] = rpm >> 8;
  buf[5] = rpm & 0xff;
  OutCrc16(buf, HedyRpm0[0]);
  uart_write_bytes_RS485(uart, port, buf, HedyRpm0[0] + 2);
}

/*********************************************************************
 * Humidiefier retro part
 **********************************************************************/
void humidifierRetro(int uart, int port) {
  if (HedyFan == false) {
    // ESP_LOGI(TAG,"Search Hedy inverter   ");
    uart_write_bytes_RS485(uart, port, writb,
                           prepair_buf(writb, HedySoftVersion));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
    if (InCrc16((char *)readb, f)) {
      int Hedy = readb[3] << 8 | readb[4];
      ESP_LOGI(TAG, "Hedy inverter Found! Softwareversion: %d ", Hedy);
      HedyFan = initHedyFan(uart, port);
    }
  }
  if (HedyFan == true) {
    if (dim1 > 0) {
      uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyRunOn));
      uart_flush_input(uart);
      f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);
    } else {
      uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyRunOff));
      uart_flush_input(uart);
      f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);
    }

    // 50Hz max dim1 from 0 to 1000
    uint32_t rpm = dim1 * 5;

    // skip 47Hz to 49Hz
    if (rpm < 4700 || rpm > 4900) {
      hedy_set_rpm(uart, port, rpm);
      uart_flush_input(uart);
      f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
    }

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedypwrAct));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
    if (InCrc16((char *)readb, f)) {
      HedyPWR = readb[3] << 8 | readb[4];
    } else {
      HedyFan = false;
    }

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyCurAct));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
    if (InCrc16((char *)readb, f)) {
      HedyCUR = readb[3] << 8 | readb[4];
    }

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyRpmAct));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
    if (InCrc16((char *)readb, f)) {
      HedyRPM = readb[3] << 8 | readb[4];
    }

    uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyVccAct));
    uart_flush_input(uart);
    f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
    if (InCrc16((char *)readb, f)) {
      HedyVCC = readb[3] << 8 | readb[4];
    }
    // ESP_LOGI(TAG, "Hedy inverter status RPMset:%.2f RMPact:%.2f VCC:%.2f
    // I:%.2f P:%.2f
    // ",(float)rpm/100,(float)HedyRPM/100,(float)HedyVCC,(float)HedyCUR/100,(float)HedyVCC*(float)HedyCUR/100);
  }
}

/*********************************************************************
 * PUMPFAN part
 **********************************************************************/
void PumpFanInvereter(int uart, int port) {
  PumpFanPid();
  if (HedyFan == false) {
    HedyFan = initHedyFan(uart, port);
  }
  if (HedyFan == true) {
    if (xSemaphoreTake(xSemaphoreSTR, 10 == pdTRUE)) {
      // after cooling
      if (CompressorOn == false && CompressorOnTime > 0) {
        Fan.speedfan = 50;
      }
      if (CompressorOn == false && CompressorOnTime == 0) {
        Fan.speedfan = 0;
      }

#ifdef forceCooldown
      Fan.speedfan = 100;
#endif
      if (Fan.speedfan != 0) {
        if (Fan.speedfan > Fan.speedmaximalfan)
          Fan.speedfan = Fan.speedmaximalfan;
        if (Fan.speedfan < Fan.speedminimalfan)
          Fan.speedfan = Fan.speedminimalfan;
      }
      // if(CompressorOn == true && Fan.speedfan <
      // Fan.speedminimalfan)Fan.speedfan = Fan.speedminimalfan;
      if (Fan.speedfan > 0) {
        uart_write_bytes_RS485(uart, port, writb,
                               prepair_buf(writb, HedyControlOn));
        uart_flush_input(uart);
        f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);
      } else {
        uart_write_bytes_RS485(uart, port, writb,
                               prepair_buf(writb, HedyControlOff));
        uart_flush_input(uart);
        f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);
      }

      // skip 47Hz to 49Hz
      if (Fan.speedfan < 4700 / 50 ||
          Fan.speedfan > 4900 / 50) {  // Bandfilter fan
        hedy_set_rpm(uart, port,
                     (uint16_t)Fan.speedfan *
                         50);  // convert percentage to rmp 100 = 1Hz
        uart_flush_input(uart);
        f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
      }
      xSemaphoreGive(xSemaphoreSTR);

      uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedypwrAct));
      uart_flush_input(uart);
      f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
      if (InCrc16((char *)readb, f)) {
        HedyPWR = readb[3] << 8 | readb[4];
      } else {
        HedyFan = false;
      }
      /*
                              uart_write_bytes_RS485(uart,port,writb,
         prepair_buf(writb, HedyCurAct)); uart_flush_input(uart); f =
         uart_read_bytes(uart,(uint8_t *)readb, 7, 10); if (InCrc16((char
         *)readb, f)) { HedyCUR = readb[3] << 8 | readb[4];
                              }

                              uart_flush_input(uart);
                              uart_write_bytes_RS485(uart,port,writb,
         prepair_buf(writb, HedyRpmAct)); f = uart_read_bytes(uart,(uint8_t
         *)readb, 7, 10); if (InCrc16((char *)readb, f)) { HedyRPM = readb[3] <<
         8 | readb[4]; }else{ HedyRPM = 0;
                              }


                              uart_write_bytes_RS485(uart,port,writb,
         prepair_buf(writb, HedyVccAct)); uart_flush_input(uart); f =
         uart_read_bytes(uart,(uint8_t *)readb, 7, 10); if (InCrc16((char
         *)readb, f)) { HedyVCC = readb[3] << 8 | readb[4];
                              }
      */
      // ESP_LOGI(TAG, "HedyFan  set:%04dRPM act:%04dRPM VCC:%03dV
      // I:%04dmA",rpm,HedyRPM,HedyVCC,HedyCUR*100);
    }
  }
}

/*********************************************************************
 * PUMPPUMP part
 **********************************************************************/

void PumpPumpInvereter(int uart, int port) {
  PumpPumpPid();

  if (HedyPump == false) {
    // ESP_LOGI(TAG,"Search Hedy inverter   ");
    HedyPump = initHedyPump(uart, port);
  } else {
    if (xSemaphoreTake(xSemaphoreSTR, 20 == pdTRUE)) {
      // chek if copmressor is active
#ifdef forceSpeedStart
      if (CompressorOn == true &&
          CompressorOnTime < 60 * 4) {  // short time for speedup testine
        Pump.speedpump = 40;
        Pump.iErrorpump = 0;
        CompressorOnTime = 60 * 4;
      }
#else
      if (CompressorOn == true && CompressorOnTime < 60 * 4) {
        Pump.speedpump = 100;
        Pump.iErrorpump = 0;
        CompressorOnTime++;
      }
#endif
      // after cooling
      if (CompressorOn == false && CompressorOnTime > 0) {
        Pump.speedpump = Pump.speedminimalpump;
        CompressorOnTime--;
      }
      if (CompressorOn == false && CompressorOnTime == 0) {
        Pump.speedpump = 0;
      }
      if (CompressorOn == true && Pump.speedpump < Pump.speedminimalpump) {
        Pump.speedpump = Pump.speedminimalpump;
      }

      // Check pressure
      // Stop pump if alarm is tue for more than 60 seconds
      if (PressLowAlarm == true) {
        if (LowPressureTimer == 0) {
          Pump.speedpump = 0;
        } else {
          LowPressureTimer--;
        }
        ESP_LOGI(TAG, "Low pressure alarm!!!");
      } else {
        LowPressureTimer = 60;
      }
#ifdef forceCooldown
      Pump.speedpump = 100;
#endif

      if (Pump.speedpump > 0) {
        uart_write_bytes_RS485(uart, port, writb,
                               prepair_buf(writb, HedyControlOn));
        uart_flush_input(uart);
        f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);
      } else {
        uart_write_bytes_RS485(uart, port, writb,
                               prepair_buf(writb, HedyControlOff));
        uart_flush_input(uart);
        f = uart_read_bytes(uart, (uint8_t *)readb, 8, 10);
      }

      if (Pump.speedpump != 0) {
        if (Pump.speedpump > Pump.speedmaximalpump) {
          Pump.speedpump = Pump.speedmaximalpump;
        }
        if (Pump.speedpump < Pump.speedminimalpump) {
          Pump.speedpump = Pump.speedminimalpump;
        }
      }

      hedy_set_rpm(
          uart, port,
          Pump.speedpump * 50);  // Correction for inverter 100% -> 50.000
      uart_flush_input(uart);
      f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
      xSemaphoreGive(xSemaphoreSTR);

      uart_write_bytes_RS485(uart, port, writb, prepair_buf(writb, HedyRpmAct));
      uart_flush_input(uart);
      f = uart_read_bytes(uart, (uint8_t *)readb, 7, 10);
      if (InCrc16((char *)readb, f)) {
        HedyRPM = readb[3] << 8 | readb[4];
      } else {
        HedyPump = false;
      }

      /*
      uart_write_bytes_RS485(uart,port,writb, prepair_buf(writb, HedyCurAct));
      uart_flush_input(uart);
      f = uart_read_bytes(uart,(uint8_t *)readb, 7, 10);
      iirf (InCrc16((char *)readb, f)) {
              HedyCUR = readb[3] << 8 | readb[4];
      }else{
              HedyPump = false;
      }

      uart_write_bytes_RS485(uart,port,writb, prepair_buf(writb, HedyVccAct));
      uart_flush_input(uart);
      f = uart_read_bytes(uart,(uint8_t *)readb, 7, 10);
      if (InCrc16((char *)readb, f)) {
              HedyVCC = readb[3] << 8 | readb[4];
      }
      //ESP_LOGI(TAG, "HedyPump set:%04dRPM act:%04dRPM VCC:%03dV
      I:%04dmA",rpm,HedyRPM,HedyVCC,HedyCUR*100);
*/
    }
  }
}

void RS487_init(void) {
  for (int t = 0; t < 6; t++) {
    init_RS485(t);  // setup all RS485 portS
  }
  if (mode == modeHumidifier) {
    init_UART_RS485(UART_NUM_1, 1200);
    init_UART_RS485(UART_NUM_2, 9600);
  }
  if (mode == modeFanPumpController) {
    init_UART_RS485(UART_NUM_1, 9600);
    init_UART_RS485(UART_NUM_2, 9600);
  }
  if (mode == modeFanAuxBoxRetro) {
    init_UART_RS485(UART_NUM_1, 9600);
    init_UART_RS485(UART_NUM_2, 9600);
  }
}

void RS487_task(void *arg) {
  RS487_init();
  vTaskDelay(250);  // give system some time to settle
  ESP_LOGI(TAG, "RS485 task running...");
  TickType_t RS485tamp = xTaskGetTickCount();
  while (1) {
    if (RS485tamp + OneSec < xTaskGetTickCount()) {
      RS485tamp = xTaskGetTickCount();
      if (mode == modeHumidifier) {
        if (!FG6485_Temp_RH_task(UART_NUM_2, port1)) {
          humidifierKabel400(UART_NUM_1, port0);
        }
      }
      if (mode == modeFanAuxBoxRetro) {
        humidifierRetro(UART_NUM_1, port0);
      }
      if (mode == modeFanPumpController) {
        PumpFanInvereter(UART_NUM_1, port2);
        PumpPumpInvereter(UART_NUM_2, port3);
      }
      if (mode == modeFanPumpBoxRetro) {
      }
    }
    vTaskDelay(10);
  }
}
