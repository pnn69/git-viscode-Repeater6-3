/*
    i2c stuff
*/
#include "i2c.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <math.h>
#include <stdbool.h>

#include "RS485.h"
#include "adc.h"
#include "global.h"
#include "menu.h"
#include "ntc.h"
#include "struckt.h"
#include "timer.h"

bool T0, T1, T2, key0, key1, key2;
int TF0, R0, TF1, R1, TF2, R2;
/*
int ADCadress[15][2] = {{ANL1adr, 1}, {ANL2adr, 1}, {ANL3adr, 0}, {ANL4adr, 0},
                        {ANL5adr, 0}, {ANL6adr, 0}, {ANL7adr, 0}, {ANL8adr, 0},
                        {FANadr, 0},  {TEMPadr, 0}, {RHadr, 0},   {OvREF, 0},
                        {v125REF, 0}, {OvREF1, 0},  {v125REF1, 0}};
                        */
int ADCadress[15] = {ANL1adr, ANL2adr, ANL3adr, ANL4adr, ANL5adr,
                     ANL6adr, ANL7adr, ANL8adr, FANadr,  TEMPadr,
                     RHadr,   OvREF,   v125REF, OvREF1,  v125REF1};

extern bool buzzerOnOff;
bool approx = false;
int aproxtimer = 600;
int menu = 0;  // position menu
bool statFan = false;
bool statRH = false;
bool statHeater = false;
static bool FrontOled = false;  // Oled display connected
static bool FrontIO = false;    // IO extener keys front connected

uint8_t PCF8574_I2C_ADDR = 0x20;  //(A version)
#define TAG "I2C"
static int unlock = 0;
xSemaphoreHandle mutexNTC;

void i2c_master_init() {
  int i2c_master_port = I2C_PORT_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_io_num = I2C_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = I2C_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  i2c_driver_install(i2c_master_port, conf.mode, I2C_RX_BUF_DISABLE,
                     I2C_TX_BUF_DISABLE, 0);
}

uint8_t i2c_read_device(uint8_t addres) {
  uint8_t data;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addres << 1) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 10 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return data;
}

void i2c_write_device(uint8_t addres, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addres << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 10 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
}

static void i2cblink(void) {
  int IOstatus = 0;
  bool led;
  IOstatus = i2c_read_device(I2C0_EXPANDER_ADDRESS_MUX);
  // ESP_LOGI(TAG, "IOstatus=%d", IOstatus);
  led = (bool)(IOstatus & (1UL << 7));  // get IO status
  led = !led;
  IOstatus = IOstatus & 0x3F;
  IOstatus ^= (-led ^ IOstatus) & (1UL << 7);  // Changing the nth bit to x
  i2c_write_device(I2C0_EXPANDER_ADDRESS_MUX,
                   IOstatus);  // update IO extender
}

void i2cscanner(void) {
  ESP_LOGD(TAG, ">> i2cScanner");
  int i;
  esp_err_t espRc;
  // printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
  printf("00:         ");
  for (i = 3; i < 0x78; i++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 10 / portTICK_PERIOD_MS);
    if (i % 16 == 0) {
      printf("\n%.2x:", i);
    }
    if (espRc == 0) {
      printf(" %.2x", i);
      if (i == 0x3c) FrontOled = true;
      if (i == 0x24) FrontIO = true;
    } else {
      printf(" --");
    }
    i2c_cmd_link_delete(cmd);
  }
  printf("\n\n");
  if (FrontOled == true) ESP_LOGI(TAG, "Display found!");
  if (FrontIO == true) ESP_LOGI(TAG, "Key extender found!");
}

static uint8_t IOstatus = 0;
static uint8_t adc_select = 0;
float voltageMUX = 0;
void i2c_task(void *arg) {
  TickType_t timestamp = xTaskGetTickCount();
  TickType_t adcstamp = xTaskGetTickCount();
  //  TickType_t unlockstamp = xTaskGetTickCount();
  TickType_t menustamp = xTaskGetTickCount();
  ESP_LOGI(TAG, "I2C tasK running... ");
  i2c_master_init();  // start i2c task
  i2cscanner();
  WaterAlarm = false;
  adc_select = 0;

  IOstatus = i2c_read_device(I2C0_EXPANDER_ADDRESS_MUX);
  IOstatus ^= (-!1 ^ IOstatus) & (1UL << buzzer);  // Changing the nth bit to x
  i2c_write_device(I2C0_EXPANDER_ADDRESS_MUX, IOstatus);  // updat leds
  IOstatus = 1 << buzzer;  // toggel bit 3 (led on pcb)
  i2c_write_device(I2C0_EXPANDER_ADDRESS_MUX, IOstatus);  // updat leds
  IOstatus ^= (-!0 ^ IOstatus) & (1UL << buzzer);         // Buzzer off
  i2c_write_device(I2C0_EXPANDER_ADDRESS_MUX, IOstatus);  // updat leds
  ssd1306_init();
  ssd1306_display_clear();
  // IOstatus = 0xff;
  IOstatus = 0x00;
  i2c_write_device(I2C0_EXPANDER_ADDRESS_NTC, IOstatus);  // update IO extender

  aproxtimer = 600;
  float adcsamp = 0;

#define avrlen 6
  float NTC1avr[avrlen] = {0};
  float NTC0avr[avrlen] = {0};
  float t = 0;
  int pNTC1 = 0;
  int pNTC0 = 0;

  for (;;) {
    i2cblink();
    if (adcstamp + pdMS_TO_TICKS(1) <= xTaskGetTickCount()) {
      adcstamp = xTaskGetTickCount();
      adcsamp = AdcSampRaw();
      ADC[adc_select] = adcsamp;
      adcsamp = ReadVoltage(adcsamp) * 1000;

      switch (adc_select) {
        case 0:
          adcsamp = ADCmV[adc_select] / 1000;
          if (adcsamp > 1.0) {
            WaterAlarm = true;
          } else {
            WaterAlarm = false;
          }
          // ESP_LOGI(TAG, "P3 Vin:%.2fV ",adcsamp);
          break;

        case 1:
          NTC1avr[pNTC1++] = ADCmV[adc_select];
          if (pNTC1 >= avrlen) pNTC1 = 0;
          if (xSemaphoreTake(xSemaphoreNTC, 10 == pdTRUE)) {
            float avr = 0;
            for (int i = 0; i < avrlen; i++) {
              avr += NTC1avr[i];
            }
            avr = avr / avrlen;
            NTC[1] = new_ntc_sample5v(avr) + NTC_offset[1];
            xSemaphoreGive(xSemaphoreNTC);
          }
          // ESP_LOGI(TAG, "P2 Vin:%.2fV ",adcsamp*VgainRJ45/1000);
          break;
        case 2:
          NTC0avr[pNTC0++] = ADCmV[adc_select];
          if (pNTC0 >= avrlen) pNTC0 = 0;
          if (xSemaphoreTake(xSemaphoreNTC, 10 == pdTRUE)) {
            float avr = 0;
            for (int i = 0; i < avrlen; i++) {
              avr += NTC0avr[i];
            }
            avr = avr / avrlen;
            NTC[0] = new_ntc_sample5v(avr) + NTC_offset[0];
            xSemaphoreGive(xSemaphoreNTC);
          }
          // ESP_LOGI(TAG, "P1 Vin:%.2fV ",adcsamp*VgainRJ45/1000);
          break;

        case 3:
          adcsamp = ADCmV[adc_select] / 1000;
          if (adcsamp > 1.0) {
            DayNight = true;
          }
          if (adcsamp < 0.8) {
            DayNight = false;
          }
          // ESP_LOGI(TAG,"P4 Vin:%.2fV %s",adcsamp,(DayNight==1) ?
          // "Day":"Night"); ESP_LOGI(TAG, "P4 Vin:%.2fV ",adcsamp);
          break;
        case 4:
          t = ADCmV[adc_select] / 1000;
          if (t > 0.25) {
            PressLow =
                map(t, 0.5, 4.5, 0, 10);  // inp,Vmin,Vmax,Barrmin,Barrmax
          } else {
            PressLow = NAN;  // no pressure sensor mounted
          }
          // ESP_LOGI(TAG, "P5 Vin:%.2fV ",t);
          break;
        case 5:
          t = ADCmV[adc_select] / 1000;
          if (t > 0.25) {
            PressHigh =
                map(t, 0.5, 4.5, 0, 10);  // inp,Vmin,Vmax,Barrmin,Barrmax
          } else {
            PressHigh = NAN;  // no pressure sensor mounted
          }
          // ESP_LOGI(TAG, "P6 Vin:%.2fV ",t);
          break;
        case 6:
          if (xSemaphoreTake(xSemaphoreVIN, 10 == pdTRUE)) {
            voltageRH = ADCmV[adc_select] * VgainRJ12 / 1000;
            xSemaphoreGive(xSemaphoreVIN);
          }
          break;

        case 7:
          if (xSemaphoreTake(xSemaphoreVIN, 10 == pdTRUE)) {
            voltageHEAT = ADCmV[adc_select] * VgainRJ12 / 1000;  // result in
                                                                 // mV;
            xSemaphoreGive(xSemaphoreVIN);
          }
          break;
        case 8:
          if (xSemaphoreTake(xSemaphoreVIN, 10 == pdTRUE)) {
            // voltageFAN = map(ADC[adc_select], 346, 2303, 0, 12);
            // voltageFAN = map(ADC[adc_select], 55, 1155, 0, 12); //
            // ADC_ATTEN_DB_11
            voltageFAN =  map(ADC[adc_select], 184, 2242, 0, 12);  // ADC_ATTEN_DB_6
            if (voltageFAN > CompressorOnVoltage) {
              CompressorOn = false;
            } else {
              CompressorOn = true;
            }
            xSemaphoreGive(xSemaphoreVIN);
          }
        case 9:
          if (xSemaphoreTake(xSemaphoreVIN, 10 == pdTRUE)) {
            // voltageFAN = map(ADC[adc_select], 346, 2303, 0, 12);
            // voltageFAN = map(ADC[adc_select], 55, 1155, 0, 12); //
            // ADC_ATTEN_DB_11
            voltageHEAT =
                map(ADC[adc_select], 184, 2242, 0, 12);  // ADC_ATTEN_DB_6
          }
        case 10:
          if (xSemaphoreTake(xSemaphoreVIN, 10 == pdTRUE)) {
            // voltageFAN = map(ADC[adc_select], 346, 2303, 0, 12);
            // voltageFAN = map(ADC[adc_select], 55, 1155, 0, 12); //
            // ADC_ATTEN_DB_11
            voltageRH =
                map(ADC[adc_select], 184, 2242, 0, 12);  // ADC_ATTEN_DB_6
            xSemaphoreGive(xSemaphoreVIN);
          }

          break;
      }
      if (adc_select++ > 15) adc_select = 0;
      IOstatus = i2c_read_device(I2C0_EXPANDER_ADDRESS_MUX);
      IOstatus = IOstatus & 0x3F;
      IOstatus = 0;
      IOstatus =
          IOstatus | (ADCadress[adc_select]);  // set mux to selected channel
      i2c_write_device(I2C0_EXPANDER_ADDRESS_MUX,
                       IOstatus);  // update IO extender

      // ESP_LOGI(TAG, "VoltageFAN %f",voltageFAN);
      // ESP_LOGI(TAG, "ADC Raw %4d  %4d %4d %4d %4d
      // %4d",ADC[0],ADC[1],ADC[2],ADC[3],ADC[4],ADC[5]);
    }
    if (timestamp + pdMS_TO_TICKS(100) < xTaskGetTickCount()) {
      timestamp = xTaskGetTickCount();
      if (dim1)
        statFan = true;
      else
        statFan = false;
      if (dim2)
        statRH = true;
      else
        statRH = false;
      if (dim3)
        statHeater = true;
      else
        statHeater = false;
      if (FrontIO) {
        IOstatus = i2c_read_device(I2C0_EXPANDER_ADDRESS_KEY);
        key0 = (bool)(IOstatus & (1UL << POS_KEY0));  // get IO status
        key1 = (bool)(IOstatus & (1UL << POS_KEY1));  // get IO status
        key2 = (bool)(IOstatus & (1UL << POS_KEY2));  // get IO status

        approx = (bool)(IOstatus & (1UL << approxPin));        // get IO status
        IOstatus ^= (-!statFan ^ IOstatus) & (1UL << ledFan);  // set led out1
        IOstatus ^= (-!statRH ^ IOstatus) & (1UL << ledRH);    // set led out2
        IOstatus ^=
            (-!statHeater ^ IOstatus) & (1UL << ledHeater);  // set led out3
        IOstatus ^=
            (-!0 ^ IOstatus) & (1UL << approxPin);  // Changing the nth bit to x
        IOstatus ^= (-!0 ^ IOstatus) & (1UL << POS_KEY0);  // Set bit KEY0
        IOstatus ^= (-!0 ^ IOstatus) & (1UL << POS_KEY1);  // Set bit KEY1
        IOstatus ^= (-!0 ^ IOstatus) & (1UL << POS_KEY2);  // Set bit KEY2
        i2c_write_device(I2C0_EXPANDER_ADDRESS_KEY,
                         IOstatus);  // update IO extender
        if (approx == true) aproxtimer = 600;
        if (FrontOled) {
          if (dispayOnOff == false) {
            ssd1306_display_OnOff(true);
            OLED_homeScreen();
            vTaskDelay(200 / portTICK_PERIOD_MS);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            unlock = 0;

          } else {
            if (dispayOnOff == true && aproxtimer-- == 0) {
              ssd1306_display_OnOff(false);
            }
          }
        }
      }
    }

    if (dispayOnOff && FrontOled) {
      if ((key0 || key1 || key2)) {
        if (buzzerOnOff) {
          IOstatus = i2c_read_device(I2C0_EXPANDER_ADDRESS_MUX);
          IOstatus ^=
              (-!1 ^ IOstatus) & (1UL << buzzer);  // Changing the nth bit to x
          i2c_write_device(I2C0_EXPANDER_ADDRESS_MUX,
                           IOstatus);  // update buzzer
          IOstatus ^=
              (-!0 ^ IOstatus) & (1UL << buzzer);  // Changing the nth bit to x
          i2c_write_device(I2C0_EXPANDER_ADDRESS_MUX,
                           IOstatus);  // update buzzer
        }
        if (key0) {
          ESP_LOGI(TAG, "Key UP");
          if (mode == modeHumidifier)
            LCD_menu_1(UP);
          else if (mode == modeFanAuxBoxRetro)
            LCD_menu_2(UP);
          else if (mode == modeFanPumpController)
            LCD_menu_3(UP);
          else if (mode == modeFanPumpBoxRetro)
            LCD_menu_4(UP);
        }
        if (key1) {
          ESP_LOGI(TAG, "Key DOWN");
          if (mode == modeHumidifier)
            LCD_menu_1(DOWN);
          else if (mode == modeFanAuxBoxRetro)
            LCD_menu_2(DOWN);
          else if (mode == modeFanPumpController)
            LCD_menu_3(DOWN);
          else if (mode == modeFanPumpBoxRetro)
            LCD_menu_4(DOWN);
        }
        if (key2) {
          ESP_LOGI(TAG, "Key ENTER");
          if (mode == modeHumidifier)
            LCD_menu_1(ENTER);
          else if (mode == modeFanAuxBoxRetro)
            LCD_menu_2(ENTER);
          else if (mode == modeFanPumpController)
            LCD_menu_3(ENTER);
          else if (mode == modeFanPumpBoxRetro)
            LCD_menu_4(ENTER);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        key0 = 0;
        key1 = 0;
        key2 = 0;
      } else {
        if (menustamp + pdMS_TO_TICKS(100) < xTaskGetTickCount()) {
          menustamp = xTaskGetTickCount();
          if (mode == modeHumidifier)
            LCD_menu_1(0);
          else if (mode == modeFanAuxBoxRetro)
            LCD_menu_2(0);
          else if (mode == modeFanPumpController)
            LCD_menu_3(0);
          else if (mode == modeFanPumpBoxRetro)
            LCD_menu_4(0);
        }
      }
    }
    vTaskDelay(1);
  }
}
