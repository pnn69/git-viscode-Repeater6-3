/*
 *
 * FanAuxBox retro
 * uses hardware version FanAuxBox 1.7
 *
 */
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/periph_ctrl.h>
#include <driver/timer.h>
#include <driver/uart.h>
#include <esp_adc_cal.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <math.h>
#include <soc/timer_group_struct.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "esp_system.h"
#include "global.h"
#include "menu.h"
#include "ntc.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "timer.h"
#define TAG "Main"
uint8_t mac[6];

bool buzzerOnOff = false;
SemaphoreHandle_t xSemaphoreNTC = NULL;
SemaphoreHandle_t xSemaphoreSTR = NULL;
SemaphoreHandle_t xSemaphoreVIN = NULL;

static float voltage = 0;
static float tmp = 0;
static uint8_t ch;
float datalog=0;
void main_task(void *parameter) {
  ESP_LOGI("Main", "Smartbox HW:%s SW:%s NVM:%d", HW_VERSION, SW_VERSION,
           NVM_VERSION);
  OLED_homeScreen();
  TickType_t secstamp = xTaskGetTickCount();
  vTaskDelay(150);
  enableOutput = true;
  ESP_LOGI(TAG, "Main task running...");
  ESP_LOGI(TAG, "Output enabled");

  while (1) {
    switch (mode) {
        //**************************************************************************************************
        // modeHumidifier 1
        //**************************************************************************************************
      case modeHumidifier:

        if (secstamp + OneSec <= xTaskGetTickCount()) {
          secstamp = xTaskGetTickCount();
          // ESP_LOGI(TAG,"T0:%d[%3d,%03d,%03d] T1:%d[%3d,%03d,%03d]
          // T2:%d[%3d,%03d,%03d] Approx:%d Sec", T0,TF0,R0,TF0-R0,T1,
          // TF1,R1,TF1-R1,T2, TF2,R2,TF2-R2, aproxtimer);
          // ESP_LOGI(TAG,"Dim1=%d",dim1);
        }
        tmp = 0;
        if (xSemaphoreTake(xSemaphoreSTR, pdTRUE)) {
          if (Kabel400.fanspeed > 0) {
            tmp = Kabel400.fanspeed;
            // min max check
            if (tmp < Kabel400.fanspeedminimal) tmp = Kabel400.fanspeedminimal;
            if (tmp > Kabel400.fanspeedmaximal) tmp = Kabel400.fanspeedmaximal;
          } else {
            tmp = 0;
          }
          xSemaphoreGive(xSemaphoreSTR);
          dim1 = (int)(tmp * 10);
        }
        if (xSemaphoreTake(xSemaphoreVIN, 200 == pdTRUE)) {
          if (voltageRH > 4.15) {
            dim2 = 1000;
            statRH = true;
          }
          if (voltageRH < 4.00) {
            dim2 = 0;
            statRH = false;
          }
          if (voltageHEAT > 4.15) {
            dim3 = 1000;
            statHeater = true;
          }
          if (voltageHEAT < 4.00) {
            dim3 = 0;
            statHeater = false;
          }
          xSemaphoreGive(xSemaphoreVIN);
        }
        ch = fgetc(stdin);
        if (ch != 0xFF) {
          if (ch == '1') {
            ESP_LOGI(TAG, "LOG:");
            if (xSemaphoreTake(xSemaphoreSTR, pdTRUE)) {
              ESP_LOGI(TAG, "RH_day:%02d RH_night:%02d RH_act:%02.2f",
                       Kabel400.RH_day, Kabel400.RH_night, hum_FG6485[1]);
              ESP_LOGI(
                  TAG,
                  "dim1=%d fanspeed=%.0f circulateTimer=%d circulateTime=%d",
                  dim1, Kabel400.fanspeed, Kabel400.circulateTimer * 60,
                  Kabel400.circulateTime);
              ESP_LOGI(TAG, "WaterAlarm:%s RAW: %d",
                       (WaterAlarm == 1) ? "Alarm" : "no Alarm", ADC[2]);
              ESP_LOGI(TAG, "LowWaterAlarm:%s RAW: %d",
                       (Kabel400.floatsensorError > 60) ? "Alarm" : "no Alarm",
                       Kabel400.floatsensor);
              ESP_LOGI(TAG, "P5 %s RAW: %d", (DayNight == 1) ? "Day" : "Night",
                       ADC[31]);
              ESP_LOGI(TAG, "VRH   %03.02fV RAW: %d", voltageRH, ADC[6]);
              ESP_LOGI(TAG, "VTMP  %03.02fV RAW: %d", voltageRH, ADC[7]);
              ESP_LOGI(TAG, "FAN   %03.02fV RAW: %d", voltageFAN, ADC[7]);
              ESP_LOGI(TAG, "speedfan   %03.02fV", Fan.speedfan);
              ESP_LOGI(TAG, "speedpump   %03.02fV", Fan.speedpump);
              ESP_LOGI(TAG, "ADC RAW %d %d %d %d %d %d %d %d", ADC[0], ADC[1],
                       ADC[2], ADC[3], ADC[4], ADC[5], ADC[6], ADC[7]);
              xSemaphoreGive(xSemaphoreSTR);
            }
          }
          if (ch == 'N') {
            CalibrateNTC();
          }
          if (ch == 'C') {
            CalibrateV();
          }
        }
        break;
        //**************************************************************************************************
        // modeFanAuxBoxRetro 2
        //**************************************************************************************************

      case modeFanAuxBoxRetro:
        if (secstamp + OneSec / 100 <= xTaskGetTickCount()) {
          secstamp = xTaskGetTickCount();
          voltage = (int)(voltageFAN * 100) * 1024 / 1000;
          voltage = pwr_to_time(voltage) * 1000 / 4096;  // scale power
          if (voltage > 1000) voltage = 1000;
          dim1 = 1000 - voltage;
        }
        if (voltageRH > 5.05) {
          dim2 = 1000;
          statRH = true;
        }
        if (voltageRH < 4.95) {
          dim2 = 0;
          statRH = false;
        }
        if (voltageHEAT > 5.05) {
          dim3 = 1000;
          statHeater = true;
        }
        if (voltageHEAT < 4.95) {
          dim3 = 0;
          statHeater = false;
        }
        ch = fgetc(stdin);
        if (ch != 0xFF) {
          if (ch == '1') {
            ESP_LOGI(TAG, "dim1=%d dim2=%d dim3=%d", dim1, dim2, dim3);
            ESP_LOGI(TAG, "LastCNT=%d", lstcnt);
          }
          if (ch == '2') {
            ESP_LOGI(TAG, "P1 RAW %04d  %0.2fV", ADC[0],
                     ADC[0] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "P2 RAW %04d  %0.2fV", ADC[1],
                     ADC[1] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "P3 RAW %04d  %0.2fV", ADC[2],
                     ADC[2] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "P4 RAW %04d  %0.2fV", ADC[3],
                     ADC[3] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "P5 RAW %04d  %0.2fV", ADC[4],
                     ADC[4] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "P6 RAW %04d  %0.2fV", ADC[5],
                     ADC[5] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "PR RAW %04d  %0.2fV", ADC[6],
                     ADC[6] * NVMsystem.NVMVgain * VgainRJ12 / 1000);
            ESP_LOGI(TAG, "PH RAW %04d  %0.2fV", ADC[7],
                     ADC[7] * NVMsystem.NVMVgain * VgainRJ12 / 1000);
            ESP_LOGI(TAG, "FAN   %03.02fV ", voltageFAN);
            ESP_LOGI(TAG, "log   %03.02fV ", datalog);
          }

          if (ch == 'N') {
            CalibrateNTC();
          }
          if (ch == 'C') {
            CalibrateV();
          }
        }
        break;

        //**************************************************************************************************
        // modeFanPumpController 3
        //**************************************************************************************************
      case modeFanPumpController:
        if (xSemaphoreTake(xSemaphoreSTR, pdTRUE)) {
          if (Fan.speedfan > 0) {
            tmp = Fan.speedfan;
            if (tmp > Fan.speedmaximalfan)
              tmp = Fan.speedmaximalfan;  // man fanspeed
            if (tmp < Fan.speedminimalfan)
              tmp = Fan.speedminimalfan;  // min fanspeed
          } else {
            tmp = 0;  // fan off
          }
          dim1 = (int)(tmp * 10);
          xSemaphoreGive(xSemaphoreSTR);
        }

        // check High and low pressure values
        if (PressLow < 0.3 && PressLow != NAN) {
          PressLowAlarm = true;
        } else {
          PressLowAlarm = false;
        }
        if (PressHigh < 0.3 && PressHigh != NAN) {
          PressHighAlarm = true;
        } else {
          PressHighAlarm = false;
        }

        // calculate pump On/Off
        // PressLow < 0  if input voltage drops below 0.5V Indicating no sensor
        // connected if so skip low pressure protection
        if (xSemaphoreTake(xSemaphoreVIN, 10 == pdTRUE)) {
          if (voltageFAN > 2 || PressLowAlarm == true ||
              PressHighAlarm == true) {
            dim2 = 0;
          } else {
            dim2 = 1000;
          }
          if (voltageFAN > 2 && CompressorOnTime > 0) {
            dim2 = 1000;
          }
          xSemaphoreGive(xSemaphoreVIN);
        }

        // Debug info
        if (secstamp + OneSec <= xTaskGetTickCount()) {
          secstamp = xTaskGetTickCount();
          if (xSemaphoreTake(xSemaphoreNTC, pdTRUE)) {
            if (xSemaphoreTake(xSemaphoreSTR, pdTRUE)) {
              // ESP_LOGI(TAG, "Tout:%03.02fC Fan:%03.0f%% Tint:%03.02fC
              // Tset:%02.0fC DeltaT:%04.02fC Pump:%03.0f%%
              // C:%s",NTC[1],Fan.speedfan,NTC[0],Fan.Tout,NTC[0]-NTC[1],Pump.speedpump,(CompressorOn
              // == true) ? "ON":"OFF"); ESP_LOGI(TAG, " Tout:%03.02fC
              // Fan:%06.02f%% ErrFan:%06.02f%%   Tin:%03.02fC TDelta:%05.02fC
              // Pump:%06.02f%% ErrPump:%06.02f%%
              // C:%s",NTC[ToutPos],Fan.speedfan,FanError,NTC[TinPos],NTC[TinPos]-NTC[ToutPos],Pump.speedpump,PumpError,(CompressorOn
              // == true) ? "ON":"OFF"); ESP_LOGI(TAG, " Tout:%03.02fC
              // Fan:%06.02f%% ErrFan:%06.02f%%   Tin:%03.02fC TDelta:%05.02fC
              // Pump:%06.02f%% ErrPump:%06.02f%%  %s
              // I:%06.02f",NTC[ToutPos],Fan.speedfan,FanError,NTC[TinPos],NTC[TinPos]
              // -Fan.Tout,Pump.speedpump,PumpError,(CompressorOn == true) ?
              // "ON":"OFF", Fan.iErrorfan);

              xSemaphoreGive(xSemaphoreSTR);
            }
            xSemaphoreGive(xSemaphoreNTC);
          }
        }
        ch = fgetc(stdin);
        if (ch != 0xFF) {
          if (ch == '1') {
            if (xSemaphoreTake(xSemaphoreNTC, 10 == pdTRUE)) {
              ESP_LOGI(TAG, "Temp1 %03.02fC", NTC[0]);
              ESP_LOGI(TAG, "Temp2 %03.02fC", NTC[1]);
              xSemaphoreGive(xSemaphoreNTC);
            }
            ESP_LOGI(TAG, "Plow  %03.02fBar RAW: %.2fV", PressLow,
                     ADC[4] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "Phigh %03.02fBar RAW: %.2fV", PressHigh,
                     ADC[5] * NVMsystem.NVMVgain / 1000);
            if (xSemaphoreTake(xSemaphoreVIN, 10 == pdTRUE)) {
              ESP_LOGI(TAG, "VRH   %03.02fV RAW: %d", voltageRH, ADC[6]);
              ESP_LOGI(TAG, "VTMP  %03.02fV RAW: %d", voltageHEAT, ADC[7]);
              ESP_LOGI(TAG, "FAN   %03.02fV ", voltageFAN);
              xSemaphoreGive(xSemaphoreVIN);
            }
            if (xSemaphoreTake(xSemaphoreSTR, 10 == pdTRUE)) {
              ESP_LOGI(TAG, "speedfan   %03.02f", Fan.speedfan);
              ESP_LOGI(TAG, "speedpump   %03.02f", Pump.speedpump);
              xSemaphoreGive(xSemaphoreSTR);
            }
            ESP_LOGI(TAG, "ADC RAW %d %d %d %d %d %d %d %d", ADC[0], ADC[1],
                     ADC[2], ADC[3], ADC[4], ADC[5], ADC[6], ADC[7]);
          }
          if (ch == '2') {
            ESP_LOGI(
                TAG,
                "Menustatus: menuPos[0] = %d menuPos[1] = %d menuPos[2] = %d",
                menuPos[0], menuPos[1], menuPos[2]);
          }
          if (ch == '3') {
            ESP_LOGI(TAG, "P1 RAW %04d  %0.2fV", ADC[0],
                     ADC[0] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "P2 RAW %04d  %0.2fV", ADC[1],
                     ADC[1] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "P3 RAW %04d  %0.2fV", ADC[2],
                     ADC[2] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "P4 RAW %04d  %0.2fV", ADC[3],
                     ADC[3] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "P5 RAW %04d  %0.2fV", ADC[4],
                     ADC[4] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "P6 RAW %04d  %0.2fV", ADC[5],
                     ADC[5] * NVMsystem.NVMVgain / 1000);
            ESP_LOGI(TAG, "PR RAW %04d  %0.2fV", ADC[6],
                     ADC[6] * NVMsystem.NVMVgain * VgainRJ12 / 1000);
            ESP_LOGI(TAG, "PH RAW %04d  %0.2fV", ADC[7],
                     ADC[7] * NVMsystem.NVMVgain * VgainRJ12 / 1000);
            ESP_LOGI(TAG, "P5 Press LOW  %0.2f Barr", PressLow);
            ESP_LOGI(TAG, "P6 Press High %0.2f Barr", PressHigh);
          }

          if (ch == 'D') {
            ESP_LOGI(TAG, "Storing defauts");
            if (xSemaphoreTake(xSemaphoreSTR, 10 == pdTRUE)) {
              Pump.speedminimalpump = 30;
              Pump.speedmaximalpump = 100;
              Pump.TempDelta = 5;
              Pump.iErrorfanmax = 50;
              Pump.dEerrorfanmax = 60;
              Pump.Thigh = 30;
              Pump.Ppump = 15;
              Pump.Ipump = 6;
              Pump.Dpump = 0;
              Pump.Tout = 30;

              Fan.speedminimalfan = 10;
              Fan.speedmaximalfan = 100;
              Fan.iErrorfanmax = 1;
              Fan.dEerrorfanmax = 100;
              Fan.Tout = 30;
              Fan.Pfan = 25;
              Fan.Ifan = 2;
              Fan.Dfan = 0;

              xSemaphoreGive(xSemaphoreSTR);
            }
          }

          if (ch == 'N') {
            CalibrateNTC();
          }

          if (ch == 'C') {
            CalibrateV();
          }

          if (ch == '7' || ch == '9' || ch == '4' || ch == '6') {
            if (ch == '7') {
              printf("P fan ++");
              Fan.Pfan++;
            }
            if (ch == '9') {
              printf("P fan --");
              if (Fan.Pfan) Fan.Pfan++;
            }
            if (ch == '6') {
              printf("P pump ++");
              Fan.Pfan++;
            }
            if (ch == '4') {
              printf("P pump --");
              if (Fan.Pfan) Fan.Pfan++;
            }
          }
        }

        break;

        //**************************************************************************************************
        // modeFanPumpBoxRetro 4
        //**************************************************************************************************
      case modeFanPumpBoxRetro:
        if (xSemaphoreTake(xSemaphoreVIN, 10 == pdTRUE)) {
          voltage = (fan_calcPercentageToVoltage(voltageFAN * 10)) * 100;
          if (voltage > 1000) voltage = 1000;
          if (voltage >= 1000) {
            voltage = 0;
          } else {
            voltage = 1000 - voltage;
          }
          dim1 = nextValneg(dim1, (int)voltage);
          if (voltageFAN < 4.00) {
            dim1 = 1000;
            statFan = true;
          }
          if (voltageRH > 4.15) {
            dim2 = 0;
            statRH = false;
          }
          if (voltageRH < 4.00) {
            dim2 = 1000;
            statRH = true;
          }
          if (voltageHEAT > 4.15) {
            dim3 = 0;
            statHeater = false;
          }
          if (voltageHEAT < 4.00) {
            dim3 = 1000;
            statHeater = true;
          }
          xSemaphoreGive(xSemaphoreVIN);
        }

        ch = fgetc(stdin);
        if (ch != 0xFF) {
          if (ch == 'N') {
            CalibrateNTC();
          }
          if (ch == 'C') {
            CalibrateV();
          }
        }

        break;

      default:
        vTaskDelay(pdMS_TO_TICKS(1));
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void initSystemNVM(void) {
  NVMsystem.NVMVgain = 2.3894;
  NVMsystem.dProbeA5v = 9.5503329E-04;   // value B from spreadsheet
  NVMsystem.dProbeB5v = 2.7349626E-04;   // value B from spreadsheet
  NVMsystem.dProbeC5v = -2.0025608E-07;  // value C from spreadsheet

  store_struckt_name("NVMsystem", &NVMsystem, sizeof(NVMsystem));
}

void app_main() {
  xSemaphoreNTC = xSemaphoreCreateMutex();
  xSemaphoreSTR = xSemaphoreCreateMutex();
  xSemaphoreVIN = xSemaphoreCreateMutex();
  ESP_LOGI(TAG, "Smartbox booting...");
  vTaskDelay(10 / portTICK_PERIOD_MS);  // give system some time
  xTaskCreate(i2c_task, "i2c_task", 1024 * 5, (void *)0, 0, NULL);

  // setup flash drive
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  // Get current stored mode
  ESP_LOGI(TAG, "NVM version hard coded: %d", NVM_VERSION);
  nvm_version = read_nvmint32("NVM_VERSION");
  ESP_LOGI(TAG, "NVM version eeprom: %d", nvm_version);
  if (nvm_version != NVM_VERSION) {
    ESP_LOGI(TAG, "Writing default values");
    initKabel();
    initFan();
    initPump();
    initNTC_offset();
    initSystemNVM();
    store_struckt_name("Kabel400", &Kabel400, sizeof(Kabel400));
    store_struckt_name("Fan", &Fan, sizeof(inverter));
    store_struckt_name("Pump", &Pump, sizeof(inverter));
    write_nvmint32("NVM_VERSION", NVM_VERSION);
    write_nvmint32("MODE", modeFanAuxBoxRetro);
    write_nvmint32("BEEP", true);
  }

  // Get current stored mode
  mode = read_nvmint32("MODE");
  buzzerOnOff = read_nvmint32("BEEP");
  ESP_LOGI(TAG, "MODE set to %d", mode);
  restore_struckt_name("NTC_offset", &NTC_offset, sizeof(NTC_offset));
  restore_struckt_name("NVMsystem", &NVMsystem, sizeof(NVMsystem));
  // Get stored setting
  if (mode == modeHumidifier) {
    err = restore_struckt_name("Kabel400", &Kabel400, sizeof(Kabel400));
    if (NVM_VERSION != Kabel400.nvm_version || err != ESP_OK) {
      ESP_LOGI(TAG, "False data in memory so store default data");
      initKabel();
      err = store_struckt_name("Kabel400", &Kabel400, sizeof(Kabel400));
      if (err == ESP_OK) {
        write_nvmint32("NVM_VERSION", NVM_VERSION);
        ESP_LOGI(TAG, "Version NVM now up to date");
      }
    }
  }

  if (mode == modeFanPumpController) {
    err = restore_struckt_name("Pump", &Pump, sizeof(inverter));
    err = restore_struckt_name("Fan", &Fan, sizeof(inverter));
    if (NVM_VERSION != Pump.nvm_version || err != ESP_OK) {
      ESP_LOGI(TAG, "False data in memory so store default data");
      initPump();
      err = store_struckt_name("Pump", &Pump, sizeof(Pump));
      if (err == ESP_OK) {
        write_nvmint32("NVM_VERSION", NVM_VERSION);
        ESP_LOGI(TAG, "Version NVM now up to date");
      }
    }
  }
  ESP_LOGI(TAG, "Reading setup from NVS done!");
  ESP_LOGI(TAG, "NVMVgain %f", NVMsystem.NVMVgain);
  adc_config();
  init_zerocross();  // set up zero cross detection
  vTaskDelay(50);
  init_timer(period);  // start timer
  xTaskCreate(RS487_task, "RS487_task", 2024 * 2, NULL, 5, NULL);
#ifdef HardwareKey
  xTaskCreate(touch_task_key, "touch_task_key", 2024, NULL, 0, NULL);
#else
  xTaskCreate(touch_task, "touch_task", 2024, NULL, 0, NULL);
#endif
  ESP_LOGI(TAG, "Setup done!");
  xTaskCreate(main_task, "main_task", 2024, NULL, 0, NULL);
}
