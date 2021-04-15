/*
    timer stuff
*/
#include <stdbool.h>

#include "adc.h"
#include "esp_timer.h"
#include "global.h"
#define TAG "timer"
#define PULSE_OFF 4000
#define GPIO_INPUT_PIN_SEL (1ULL << ZerroCrossPin)
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_OUTPUT_PIN_SEL \
  ((1ULL << AC_pin1) | (1ULL << AC_pin2) | (1ULL << AC_pin3))

static DRAM_ATTR portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static intr_handle_t s_timer_handle;
SemaphoreHandle_t xSemaphore = NULL;

volatile char tenth;  // timing parameter
volatile int dim1 = 0;
volatile int dim2 = 0;
volatile int dim3 = 0;

static DRAM_ATTR volatile uint64_t samp, psamp, halfperiod, skew = 0;
static DRAM_ATTR volatile int16_t nRes, pRes, offsetcorrection = 0;
volatile DRAM_ATTR int16_t lstcnt, period;
static DRAM_ATTR const int nchannel = 3;
//static DRAM_ATTR const int treshold = 150;
static DRAM_ATTR const int treshold = 200;
static volatile uint16_t executeChnn[3];
static volatile int channel[3];
static DRAM_ATTR volatile int loopcnt;
volatile bool enableOutput = false;
static DRAM_ATTR volatile bool OddEven = true;

void IRAM_ATTR ZerroCrossISR(void *arg);
void IRAM_ATTR timer_isr_1(void *para);

/*
 * Stuff to make the phase cut output linear
 */

// Power to time conversion, via lookup table
// input is a power level  [0...1000] = [0..100%]
// output is the phase [0..4096] == [0..180deg]
// see spreadsheet
// n 0 1 2 3 ...
// p 0 50 100 150
// [n] 0 10 40 100
//
// 125 -> n=3
// 40 + (100-40) * (125-100)/50 = 40 + 60 * 25/50 = 70

// static word pwr_to_time(int pwr) {
// static int pwr_to_time(int pwr) {
int pwr_to_time(int pwr) {
  // static const int len = 50; // lookup table angle [0..49] -> power [0..1000]
  // static int16_t tbl[len] = {
  // 0,2,6,12,20,29,41,54,69,86,105,125,146,169,193,219,245,273,301,330,360,391,421,453,484,515,547,
  // 578,609,639,669,698,727,754,781,806,830,853,875,895,913,930,945,959,970,980,988,994,998,1000
  // };
  static const int len = 32;  // lookup table angle [0..31] -> power [0..1000]
  // static int16_t tbl[len] =
  // {0,5,14,29,47,71,98,129,164,202,242,286,331,378,426,475,524,573,621,668,714,757,798,836,870,902,929,952,971,985,995,1000
  // };
  static int16_t tbl[] = {0,   5,   14,  29,  47,  71,  98,  129, 164, 202, 242,
                          286, 331, 378, 426, 475, 524, 573, 621, 668, 714, 757,
                          798, 836, 870, 902, 929, 952, 971, 985, 995, 1000};
  int out = 4096;                  // max value
  for (int n = 0; n < len; n++) {  // 50 steps * 20
    if (tbl[n] >= pwr) {
      if (n == 0) {
        out = 0;  // min value
        break;
      }
      // integreren. deling / 32 is een shift
      int step = 4096 / len;
      out =
          ((n - 1) * step) + (pwr - tbl[n - 1]) * step / (tbl[n] - tbl[n - 1]);
      break;
    }
  }
  // ESP_LOGI(TAG,"pwr_to_time(%d) = %d", pwr, out*1000/4096);
  return out;  // too high
}

float fan_lineairCompFan(double measP, double botvalueP, double topV,
                         double nextValueV) {
  double diffV = topV - nextValueV;
  double perVP = diffV / 5.0;
  double diffP = measP - botvalueP;
  return topV - (diffP * perVP);
}

float fan_calcPercentageToVoltage(float measP)  // percentage in voltage out
{
  float returvalue = 10.0;
  if (measP <= 10)
    returvalue = fan_lineairCompFan(measP, 10, 9.79, 9.49);

  else if (measP <= 15)
    returvalue = fan_lineairCompFan(measP, 15, 9.49, 9.19);

  else if (measP <= 20)
    returvalue = fan_lineairCompFan(measP, 20, 9.19, 8.79);

  else if (measP <= 25)
    returvalue = fan_lineairCompFan(measP, 25, 8.79, 8.39);

  else if (measP <= 30)
    returvalue = fan_lineairCompFan(measP, 30, 8.39, 7.99);

  else if (measP <= 35)
    returvalue = fan_lineairCompFan(measP, 35, 7.99, 7.59);

  else if (measP <= 40)
    returvalue = fan_lineairCompFan(measP, 40, 7.59, 7.19);

  else if (measP <= 45)
    returvalue = fan_lineairCompFan(measP, 45, 7.19, 6.79);

  else if (measP <= 50)
    returvalue = fan_lineairCompFan(measP, 50, 6.79, 6.39);

  else if (measP <= 55)
    returvalue = fan_lineairCompFan(measP, 55, 6.39, 5.99);

  else if (measP <= 60)
    returvalue = fan_lineairCompFan(measP, 60, 5.99, 5.59);

  else if (measP <= 65)
    returvalue = fan_lineairCompFan(measP, 65, 5.59, 5.14);

  else if (measP <= 70)
    returvalue = fan_lineairCompFan(measP, 70, 5.14, 4.64);

  else if (measP <= 75)
    returvalue = fan_lineairCompFan(measP, 75, 4.64, 4.07);

  else if (measP <= 80)
    returvalue = fan_lineairCompFan(measP, 80, 4.07, 3.40);

  else if (measP <= 85)
    returvalue = fan_lineairCompFan(measP, 85, 3.40, 2.63);

  else if (measP <= 90)
    returvalue = fan_lineairCompFan(measP, 85, 2.63, 1.80);

  else if (measP <= 95)
    returvalue = fan_lineairCompFan(measP, 95, 1.80, 0.93);

  else if (measP <= 97)
    returvalue = fan_lineairCompFan(measP, 99, 0.93, 0.13);

  else if (measP == 100)
    returvalue = 0.12;
  else
    returvalue = 0;  // just to be sure fan never off  0.12;
  return returvalue = (returvalue);
}

/*
 * end Stuff to make the phase cut output linear
 */

/*
static int IRAM_ATTR nextVal(int curr, int max) {
        if (curr >= max) {
                if (max<0){
                        return 0;
                }
                return max;
        }
        if (max - curr > 10) {
                return curr + 10;
        }
        return curr + 1;
}
*/

/*
 * Map parameter
 */
int32_t IRAM_ATTR DimMap(int32_t x, int32_t in_min, int32_t in_max,
                         int32_t out_min, int32_t out_max) {
  x = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (x < out_min) return out_min;
  if (x > out_max) return out_max;
  return x;
}

int IRAM_ATTR nextValneg(int curr, int max) {
  if (1000 - max <= curr) {
    if (1000 - max < 0) {
      return 0;
    }
    return 1000 - max;
  }
  if (1000 - max - curr > 10) {
    return curr + 10;
  }
  return curr + 1;
}

void init_zerocross(void) {
  gpio_config_t io_conf;
  // io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.pin_bit_mask = (1ULL << ZerroCrossPin);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = (gpio_pullup_t)0;
  gpio_config(&io_conf);
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add((gpio_num_t)ZerroCrossPin, ZerroCrossISR,
                       (void *)ZerroCrossPin);
}

void IRAM_ATTR ZerroCrossISR(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  if (gpio_num == ZerroCrossPin) {
    portENTER_CRITICAL(&mux);  // used in int handler
    if (esp_timer_get_time() - samp > 9000) {
      halfperiod = samp;
      samp = esp_timer_get_time();
      OddEven = !OddEven;
      if (OddEven) {
        pRes = (int16_t)(samp - halfperiod);
        period = ((int16_t)(samp - psamp)) / 2;
        psamp = samp;
        if (OddEven) skew = offsetcorrection / 2;
      } else {
        nRes = (int16_t)(samp - halfperiod);
        if (!OddEven) skew = offsetcorrection / 2;
      }
      if (pRes < nRes) {
        offsetcorrection = (nRes - pRes) / 10;
      } else {
        offsetcorrection = (pRes - nRes) / 10;
      }

      lstcnt = (pRes + nRes) / 2;

      if (enableOutput == false) {
        gpio_set_level((gpio_num_t)AC_pin1, 0);
        gpio_set_level((gpio_num_t)AC_pin2, 0);
        gpio_set_level((gpio_num_t)AC_pin3, 0);
        TIMERG0.hw_timer[0].config.alarm_en = 0;
      } else {
        if (dim1 != 0){
          executeChnn[0] =
              (uint16_t)DimMap(dim1, 1000, 0, offsetcorrection, 1000 - treshold) + skew;
              //datalog = executeChnn[0];
        }else {
          gpio_set_level((gpio_num_t)AC_pin1, 0);
          executeChnn[0] = PULSE_OFF;
        }
        channel[0] = AC_pin1;

        if (dim2 != 0) {
          //          executeChnn[1] = DimMap(dim2, 1000, 0, offsetcorrection,
          //          1000 - treshold + skew);
          executeChnn[1] = PULSE_OFF;
          if (dim2 > 500)
            gpio_set_level((gpio_num_t)AC_pin2, 1);
          else
            gpio_set_level((gpio_num_t)AC_pin2, 0);
        } else {
          gpio_set_level((gpio_num_t)AC_pin2, 0);
          executeChnn[1] = PULSE_OFF;
        }
        channel[1] = AC_pin2;

        if (dim3 != 0) {
          // executeChnn[2] = DimMap(dim3, 1000, 0, offsetcorrection, 1000 -
          // treshold + skew);
          executeChnn[2] = PULSE_OFF;
          if (dim3 > 500)
            gpio_set_level((gpio_num_t)AC_pin3, 1);
          else
            gpio_set_level((gpio_num_t)AC_pin3, 0);
        } else {
          gpio_set_level((gpio_num_t)AC_pin3, 0);
          executeChnn[2] = PULSE_OFF;
        }
        channel[2] = AC_pin3;
/*
        for (int ch = 0; ch < nchannel - 1;
             ch++) {  // bubble sort by time, PULSE_OFF is always biggest
          for (int ch = 0; ch < nchannel - 1; ch++) {
            if (executeChnn[ch] > executeChnn[ch + 1]) {
              uint16_t tmp0 = executeChnn[ch];
              executeChnn[ch] = executeChnn[ch + 1];
              executeChnn[ch + 1] = tmp0;

              int tmp1 = channel[ch];
              channel[ch] = channel[ch + 1];
              channel[ch + 1] = tmp1;
            }
          }
        }
*/
        loopcnt = 0;
        if (executeChnn[loopcnt] < PULSE_OFF) {
          timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
          timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, executeChnn[loopcnt]);
          TIMERG0.hw_timer[0].config.alarm_en = 1;
        }
      }
    }
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void IRAM_ATTR timer_isr_1(void *para) {
  uint64_t timerstamp, wait;
  int16_t next = 0;
  TIMERG0.int_clr_timers.t0 = 1;
  timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timerstamp);
  portENTER_CRITICAL_ISR(&mux);
  next = loopcnt;
  if (next == nchannel) {
    next = next - nchannel;
  }
  gpio_set_level((gpio_num_t)channel[next], 1);
  // timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timerstamp);
  // Delay 10 us
  wait = esp_timer_get_time();
  while (wait + 10 > esp_timer_get_time())
    ;
  gpio_set_level((gpio_num_t)channel[next], 0);

  next = 0;
  switch (loopcnt) {
    case 0:
        next = 1000;
        loopcnt = 3;

//      if (executeChnn[1] < PULSE_OFF) {
//        next = executeChnn[1] - executeChnn[0];
//        loopcnt = 1;
//      } else {
//        next = 1000;
//        loopcnt = 3;
//      }
      break;
    case 1:
      if (executeChnn[2] < PULSE_OFF) {
        next = executeChnn[2] - executeChnn[1];
        loopcnt = 2;
      } else {
        next = 1000 + executeChnn[0];
        loopcnt = 3;
      }
      break;
    case 2:
      next = 1000 + executeChnn[0];
      loopcnt = 3;
      break;

    case 3:
      if (executeChnn[1] < PULSE_OFF) {
        next = executeChnn[1] - executeChnn[0];
        loopcnt = 4;
      }
      break;
    case 4:
      if (executeChnn[2] < PULSE_OFF) {
        next = executeChnn[2] - executeChnn[1];
        loopcnt = 5;
      }
      break;
    default:
      TIMERG0.hw_timer[0].config.alarm_en = 0;
      break;
  }

  if (next > 0) {
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timerstamp + next);
    TIMERG0.hw_timer[0].config.alarm_en = 1;
  } else {
    TIMERG0.hw_timer[0].config.alarm_en = 0;
  }
  portEXIT_CRITICAL_ISR(&mux);
}
/*
    setup timer auto reload
    100us interval at 50Hz
    83us  interval at 60Hz
*/

int init_timer(uint16_t set) {
  uint16_t div = 0;
  if (set <= 10200 && set >= 9800) {
    div = 1000;
    ESP_LOGI(TAG, "50Hz");
  }
  if (set >= 8100 && set <= 9400) {
    div = 833;
    ESP_LOGI(TAG, "60Hz");
  }
  gpio_pad_select_gpio(AC_pin1);
  gpio_set_direction((gpio_num_t)AC_pin1, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)AC_pin1, 0);
  gpio_pad_select_gpio(AC_pin2);
  gpio_set_direction((gpio_num_t)AC_pin2, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)AC_pin2, 0);
  gpio_pad_select_gpio(AC_pin3);
  gpio_set_direction((gpio_num_t)AC_pin3, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)AC_pin3, 0);

  // timer_spinlock_take(TIMER_GROUP_0);
  timer_config_t config = {.alarm_en = TIMER_ALARM_DIS,
                           .counter_en = TIMER_PAUSE,
                           .intr_type = TIMER_INTR_LEVEL,
                           .counter_dir = TIMER_COUNT_UP,
                           .auto_reload = TIMER_AUTORELOAD_EN,
                           .divider = 800};
  // timer_config_t config = {.alarm_en = TIMER_ALARM_EN, .counter_en = false,
  // .intr_type = TIMER_INTR_LEVEL, .counter_dir = TIMER_COUNT_UP, .auto_reload
  // = TIMER_AUTORELOAD_EN, .divider = 80};
  if (div != 0) {
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, div);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr_1, NULL, 1,
                       &s_timer_handle);
    timer_start(TIMER_GROUP_0, TIMER_0);
  } else {
    ESP_LOGI(TAG, "No mains detected");
    return 1;
  }
  // timer_spinlock_give(TIMER_GROUP_0);
  return 0;
}
