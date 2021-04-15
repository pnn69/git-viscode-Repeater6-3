/*
* Touch stuff
*/
#include <stdio.h>
#include <stdbool.h>
#include "global.h"
#include "driver/touch_pad.h"
#include "esp_log.h"

#define TAG "Touch"


#define GPIO_INPUT_PIN_SEL ((1ULL << KEY_PAD_0) | (1ULL << KEY_PAD_1) | (1ULL << KEY_PAD_2))

#define TOUCH_PAD_NO_CHANGE   (-1)
#define TOUCH_THRESH_NO_USE   (1)
#define TOUCH_FILTER_MODE_EN  (1)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (15)
//#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)
#define FILTER_TIME pdMS_TO_TICKS(150)

#define Tsens

#ifdef Tsens
#define tres0 15
#define tres1 15
#define tres2 15
#else
#define tres0 75
#define tres1 75
#define tres2 75
#endif


uint16_t TF0,R0,TF1,R1,TF2,R2;
bool T0,T1,T2,key0,key1,key2;
bool  calibrate = false;
void calibrateTouch(void){
	ESP_LOGI(TAG,"Calibrate touch");
    touch_pad_read_filtered(TOUCH_PAD_0, &TF0);
    touch_pad_read_filtered(TOUCH_PAD_1, &TF1);
    touch_pad_read_filtered(TOUCH_PAD_2, &TF2);
    R0 = TF0-tres0;
    R1 = TF1-tres1;
    R2 = TF2-tres2;
}

/*
    Read values sensed at all available touch pads.
    Print out values in a loop on a serial monitor.
*/
static bool calc = false;


void key_pad_init(void){
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 1;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

}

void touch_task(void *pvParameter)
{
	TickType_t touchstamp = xTaskGetTickCount();
    //alternative touch sensor
	//TTP223 https://lcsc.com/products/Touch-Screen-Controller-ICs_902.html?q=TTP223
	ESP_LOGI(TAG,"Init soft key");
    touch_pad_init();
    vTaskDelay(500/ portTICK_PERIOD_MS);
    //touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V7, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    //touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V5);
    //TOUCH_PAD_SLOPE_1
    touch_pad_config(TOUCH_PAD_0, TOUCH_THRESH_NO_USE);
    touch_pad_config(TOUCH_PAD_1, TOUCH_THRESH_NO_USE);
    touch_pad_config(TOUCH_PAD_2, TOUCH_THRESH_NO_USE);

    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    vTaskDelay(500/ portTICK_PERIOD_MS);
    touch_pad_read_filtered(TOUCH_PAD_0, &TF0);
    touch_pad_read_filtered(TOUCH_PAD_1, &TF1);
    touch_pad_read_filtered(TOUCH_PAD_2, &TF2);
    vTaskDelay(500/ portTICK_PERIOD_MS);
    calibrateTouch();
    ESP_LOGI(TAG,"Init done!");
    TickType_t touchcalc = xTaskGetTickCount();

    while (1) {
        // If open the filter mode, please use this API to get the touch pad count.
    	if(calibrate == true){
    		vTaskDelay(10/ portTICK_PERIOD_MS);
    		calibrateTouch();
    		calibrate = false;
    	}
        touch_pad_read_filtered(TOUCH_PAD_0, &TF0);
        touch_pad_read_filtered(TOUCH_PAD_1, &TF1);
        touch_pad_read_filtered(TOUCH_PAD_2, &TF2);

    	if (touchcalc + FILTER_TIME < xTaskGetTickCount()) {
        	touchcalc = xTaskGetTickCount();
        	calc = true;
    	}else{
    		calc =  false;
    	}

    	if(R0 > TF0){
            T0 = true;
            key0 = true;
            if(R0-TF0 >= tres0 + 2 ){
            	R0--;
            }
        }else{
        	T0 = false;
        	if(calc){
        		if(TF0-tres0 > R0)R0=TF0-tres0;
              	if(TF0-tres0 < R0)R0=R0-1;
        	}
        }

    	if(R1 > TF1){
            T1 = true;
            key1 = true;
            if(R1-TF1 > tres1 + 2){
            	R1--;
            }
        }else{
        	T1 = false;
        	if(calc){
        		if(TF1-tres1 >= R1) R1=TF1-tres1;
        		if(TF1-tres1 < R1) R1=R1-1;
        	}
        }

    	if(R2 > TF2){
            T2 =  true;
            key2 = true;
            if(R2-TF2 > tres2 + 2){
            	R2--;
            }
        }else{
            T2 =  false;
            if(calc){
            	if(TF2-tres2 >= R2) R2=TF2-tres2;
            	if(TF2-tres2 < R2) R2=R2-1;
            }
        }

    	if (touchstamp + pdMS_TO_TICKS(500) < xTaskGetTickCount()) {
        	touchstamp = xTaskGetTickCount();
        	//ESP_LOGI(TAG,"T0:%d[%3d,%03d] T1:%d[%3d,%03d] T2:%d[%3d,%03d] Approx:%dSec", T0,TF0,TF0-R0,T1, TF1,TF1-R1,T2, TF2,TF2-R2, aproxtimer);
        }
        vTaskDelay(1);
    }
}
//Switch
//https://nl.farnell.com/te-connectivity-alcoswitch/fsm4jrt/through-hole-tactile/dp/3406905?st=fsm4j
static bool KeyConfig = false;
void touch_task_key(void *pvParameter){
    ESP_LOGI(TAG,"Init hard key");
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 1;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

    while(gpio_get_level(KEY_PAD_0) != gpio_get_level(KEY_PAD_1) || gpio_get_level(KEY_PAD_0) != gpio_get_level(KEY_PAD_2)|| gpio_get_level(KEY_PAD_1) != gpio_get_level(KEY_PAD_2));
    KeyConfig =(bool)gpio_get_level(KEY_PAD_0);
    if(KeyConfig == true){
		while(1){
			if(!gpio_get_level(KEY_PAD_0)){
				vTaskDelay(pdMS_TO_TICKS(10));
				if(!gpio_get_level(KEY_PAD_0)) key0 = true;
			}
			if(!gpio_get_level(KEY_PAD_1)){
				vTaskDelay(pdMS_TO_TICKS(10));
				if(!gpio_get_level(KEY_PAD_1)) key1 = true;
			}
			if(!gpio_get_level(KEY_PAD_2)){
				vTaskDelay(pdMS_TO_TICKS(10));
				if(!gpio_get_level(KEY_PAD_2)) key2 = true;

			}
			vTaskDelay(1);
		}
    }else{
		while(1){
			if(gpio_get_level(KEY_PAD_0)){
				vTaskDelay(pdMS_TO_TICKS(10));
				if(gpio_get_level(KEY_PAD_0)) key0 = true;
			}
			if(gpio_get_level(KEY_PAD_1)){
				vTaskDelay(pdMS_TO_TICKS(10));
				if(gpio_get_level(KEY_PAD_1)) key1 = true;
			}
			if(gpio_get_level(KEY_PAD_2)){
				vTaskDelay(pdMS_TO_TICKS(10));
				if(gpio_get_level(KEY_PAD_2)) key2 = true;

			}
			vTaskDelay(1);
		}
    }
}
