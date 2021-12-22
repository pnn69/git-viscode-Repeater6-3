
/*
 * global.h
 *
 */
#ifndef GLOBAL_H_
#define GLOBAL_H_

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
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <soc/timer_group_struct.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "display.h"
#include "i2c.h"
#include "rs485.h"
#include "sdkconfig.h"
#include "struckt.h"

#define HW_VERSION "6/3V1.1aUSB"
#define SW_VERSION "1.4"

#define ZerroCrossPin 35

#define OneSec pdMS_TO_TICKS(1000)

#define UP 1
#define DOWN 2
#define ENTER 3

#define POS_KEY0 (5)
#define POS_KEY1 (4)
#define POS_KEY2 (3)

#define ledFan 0    // pos led Fan IO extender
#define ledRH 1     // pos led RH IO extender
#define ledHeater 2 // pos led Heater IO extender

#define ThyristorON 1
#define BuzzerOn 0
#define LedIO 7

#define AC_pin1 32 // Output to Opto Triac1
#define AC_pin2 33 // Output to Opto Triac2
#define AC_pin3 23 // Output to Opto Triac3

#define ANL1adr 0x10  // Analog input P1
#define ANL2adr 0x20  // Analog input P2
#define ANL3adr 0x30  // Analog input P3
#define ANL4adr 0x01  // Analog input P4
#define ANL5adr 0x02  // Analog input P5
#define ANL6adr 0x04  // Analog input P6
#define ANL7adr 0x05  // Analog input P7
#define ANL8adr 0x07  // Analog input P8
#define FANadr 0x08   // Analog Fan
#define TEMPadr 0x00  // Analog Temp
#define RHadr 0x18    // Analog RH
#define OvREF 0x03    // 0V Ref
#define v125REF 0x06  // 1.25V Ref
#define OvREF1 0x28   // 0V Ref
#define v125REF1 0x38 // 1.25V Ref

#define modeHumidifier 1
#define modeFanAuxBoxRetro 2
#define modeFanPumpController 3
#define modeFanPumpBoxRetro 4

#define VgainFan 4.556
#define VgainRJ12 1.68
#define CompressorOnVoltage 2
#define LoAnaCount10V 370
#define HiAnaCount10V 2645
#define LoAnaCount12V 373
#define HiAnaCount12V 3195
#define HiAnaCount15V 3895
#define LoAnaCount15V 337
#define HiAnaCount5V 1515
#define LoAnaCount5V 337

#define ToutPos 1
#define TinPos 0
extern volatile bool onesec;
extern bool buzzerOnOff;
extern float PressHigh;
extern float PressLow;
extern float datalog;
extern int menuPos[3];
extern SemaphoreHandle_t xSemaphoreNTC;
extern SemaphoreHandle_t xSemaphoreSTR;
extern SemaphoreHandle_t xSemaphoreVIN;
extern bool approx;
TimerHandle_t tmr;

#endif /* GLOBAL_H_ */
