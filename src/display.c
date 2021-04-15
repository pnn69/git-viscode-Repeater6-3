/*
    display stuff
*/
#include "global.h"
#include "i2c.h"
#include "ssd1366.h"
#include "font8x8_basic.h"
#include <stdbool.h>

// SDA - GPIO21
#define PIN_SDA 22
// SCL - GPIO22
#define PIN_SCL 19




#define SSD1306_HEIGHT 32
#define SSD1306_WIDTH 128

bool dispayOnOff = false;

void I2C_start_p(i2c_cmd_handle_t cmd){
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, 0x00, true);
}

void I2C_stop_p(i2c_cmd_handle_t cmd){
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

void I2C_select_rows( uint8_t beginRow, uint8_t endRow){
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	I2C_start_p(cmd);
	i2c_master_write_byte(cmd, OLED_CMD_SET_COLUMN_RANGE, true);
	i2c_master_write_byte(cmd, 0, true);
	i2c_master_write_byte(cmd, SSD1306_WIDTH-1, true);
	I2C_stop_p(cmd);
	cmd = i2c_cmd_link_create();
	I2C_start_p(cmd);
	i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_RANGE, true);
	i2c_master_write_byte(cmd, beginRow, true);
	i2c_master_write_byte(cmd, endRow, true);
	I2C_stop_p(cmd);
}

void ssd1306_init() {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	//vTaskDelay(pdMS_TO_TICKS(100));
	I2C_start_p(cmd);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_CLK_DIV, true);
	i2c_master_write_byte(cmd, 0x80, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_MUX_RATIO, true);
	I2C_stop_p(cmd);
	cmd = i2c_cmd_link_create();
	I2C_start_p(cmd);
	i2c_master_write_byte(cmd, SSD1306_HEIGHT-1, true);
	I2C_stop_p(cmd);
	cmd = i2c_cmd_link_create();
    I2C_start_p(cmd);
	i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_OFFSET, true);
	i2c_master_write_byte(cmd, 0x0, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_START_LINE | 0x00, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
	I2C_stop_p(cmd);
	cmd = i2c_cmd_link_create();
	I2C_start_p(cmd);
	i2c_master_write_byte(cmd, 0x14, true);
	I2C_stop_p(cmd);
	cmd = i2c_cmd_link_create();
    I2C_start_p(cmd);
	i2c_master_write_byte(cmd, OLED_CMD_SET_MEMORY_ADDR_MODE, true);
	i2c_master_write_byte(cmd, 0x00, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true);
	I2C_stop_p(cmd);
	cmd = i2c_cmd_link_create();
    I2C_start_p(cmd);
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_PIN_MAP, true);
	i2c_master_write_byte(cmd, 0x02, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true);
	i2c_master_write_byte(cmd, 0x8F, true);
	I2C_stop_p(cmd);
	cmd = i2c_cmd_link_create();
	I2C_start_p(cmd);
	i2c_master_write_byte(cmd, OLED_CMD_SET_PRECHARGE, true);
	I2C_stop_p(cmd);
	cmd = i2c_cmd_link_create();
	I2C_start_p(cmd);
	i2c_master_write_byte(cmd, 0xf1, true);
	I2C_stop_p(cmd);
	cmd = i2c_cmd_link_create();
	I2C_start_p(cmd);
	i2c_master_write_byte(cmd, OLED_CMD_SET_PRECHARGE, true);
	i2c_master_write_byte(cmd, 0x40, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_RAM, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NO_SCROLL, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
	I2C_stop_p(cmd);
	dispayOnOff = true;
}

void ssd1306_display_clear() {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	I2C_select_rows(0,7);
	uint16_t count = (uint16_t)SSD1306_WIDTH << 3;
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, 0x40, true);
	while (count--)
    {
        i2c_master_write_byte(cmd, 0x0, true);
    }
    I2C_stop_p(cmd);
}

void ssd1306_display_text(char *text) {
	uint8_t text_len = strlen(text);
	i2c_cmd_handle_t cmd;
	uint8_t cur_page = 0;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, 0x00, true); // reset column
	i2c_master_write_byte(cmd, 0x10, true);
	i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page
	for (uint8_t i = 0; i < text_len; i++) {
		i2c_master_start(cmd);
		if (text[i] == '\n') {
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
			i2c_master_write_byte(cmd, 0x00, true); // reset column
			i2c_master_write_byte(cmd, 0x10, true);
			i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page
		} else {
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);
		}
	}
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 10);
	i2c_cmd_link_delete(cmd);
}


void ssd1306_DrawStrd(uint8_t r,uint8_t p,char *text){
    uint8_t text_len = strlen(text);
	i2c_cmd_handle_t cmd;
	uint8_t cur_page = 0;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, 0x00, true); // reset column
	i2c_master_write_byte(cmd, 0x10, true);
	i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page
    for (; r > 0; r--) {
    	printf("r = %d",r);
        i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
        i2c_master_write_byte(cmd, 0x00, true); // reset column
        i2c_master_write_byte(cmd, 0x10, true);
        i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page
    }
    for (; p > 0; p--) {
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write(cmd, font8x8_basic_tr[' '], 8, true);
		}
    ssd1306_display_text(text);
	for (uint8_t i = 0; i < text_len; i++) {
		if (text[i] == '\n') {
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
			i2c_master_write_byte(cmd, 0x00, true); // reset column
			i2c_master_write_byte(cmd, 0x10, true);
			i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page
		} else {
  			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);
		}
	}
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 10/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}


void ssd1306_display_OnOff(bool on) {
	if(on){
		//ESP_LOGI("display", "Oled on... ");
		ssd1306_init();
		dispayOnOff = true;
	}else{
		if(dispayOnOff == true){
			//ESP_LOGI("display", "Oled off... ");
			i2c_cmd_handle_t cmd = i2c_cmd_link_create();
			I2C_start_p(cmd);
			i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);
			i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_CLK_DIV, true);
			i2c_master_write_byte(cmd, 0x80, true);
			i2c_master_write_byte(cmd, OLED_CMD_SET_MUX_RATIO, true);
			I2C_stop_p(cmd);
			dispayOnOff =  false;
		}
	}
}
