/*
 * Touch stuff
 */
#include <stdbool.h>
#include <stdio.h>

#include "esp_log.h"
#include "global.h"

#define TAG "Touch"

uint16_t TF0, R0, TF1, R1, TF2, R2;
bool T0, T1, T2, key0, key1, key2;

void key_pad_init(void) {
  gpio_config_t io_conf;
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  // bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  // disable pull-down mode
  io_conf.pull_down_en = 0;
  // disable pull-up mode
  io_conf.pull_up_en = 1;
  // configure GPIO with the given settings
  gpio_config(&io_conf);
}

// Switch
// https://nl.farnell.com/te-connectivity-alcoswitch/fsm4jrt/through-hole-tactile/dp/3406905?st=fsm4j
static bool KeyConfig = false;
void touch_task_key(void *pvParameter) {
  while (gpio_get_level(KEY_PAD_0) != gpio_get_level(KEY_PAD_1) ||
         gpio_get_level(KEY_PAD_0) != gpio_get_level(KEY_PAD_2) ||
         gpio_get_level(KEY_PAD_1) != gpio_get_level(KEY_PAD_2))
    ;
  KeyConfig = (bool)gpio_get_level(KEY_PAD_0);
  if (KeyConfig == true) {
    while (1) {
      if (!gpio_get_level(KEY_PAD_0)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!gpio_get_level(KEY_PAD_0)) key0 = true;
      }
      if (!gpio_get_level(KEY_PAD_1)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!gpio_get_level(KEY_PAD_1)) key1 = true;
      }
      if (!gpio_get_level(KEY_PAD_2)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!gpio_get_level(KEY_PAD_2)) key2 = true;
      }
      vTaskDelay(1);
    }
  } else {
    while (1) {
      if (gpio_get_level(KEY_PAD_0)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (gpio_get_level(KEY_PAD_0)) key0 = true;
      }
      if (gpio_get_level(KEY_PAD_1)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (gpio_get_level(KEY_PAD_1)) key1 = true;
      }
      if (gpio_get_level(KEY_PAD_2)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (gpio_get_level(KEY_PAD_2)) key2 = true;
      }
      vTaskDelay(1);
    }
  }
}
