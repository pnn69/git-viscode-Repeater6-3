
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
#include "touch.h"

#define HW_VERSION "6.3A"
#define SW_VERSION "1.0"

#define ZerroCrossPin 35

#define OneSec pdMS_TO_TICKS(1000)

#define HardwareKey 1  // Use switches mounted at the front

#define UP 1
#define DOWN 2
#define ENTER 3

#define TOUCH_PAD_0 5
#define TOUCH_PAD_1 4
#define TOUCH_PAD_2 3

#define KEY_PAD_0 12
#define KEY_PAD_1 13
#define KEY_PAD_2 15

#define ledFan 0     // pos led Fan IO extender
#define ledRH 1      // pos led RH IO extender
#define ledHeater 2  // pos led Heater IO extender

#define AC_pin1 33  // Output to Opto Triac1
#define AC_pin2 25  // Output to Opto Triac2
#define AC_pin3 32  // Output to Opto Triac3

#define modeHumidifier 1
#define modeFanAuxBoxRetro 2
#define modeFanPumpController 3
#define modeFanPumpBoxRetro 4

#define VgainFan 4.556
#define VgainRJ12 1.68
//#define VgainRJ45	2.03894
//#define VgainRJ45	2.393122
#define CompressorOnVoltage 2

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

TimerHandle_t tmr;

#endif /* GLOBAL_H_ */
