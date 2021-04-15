/*
 * display.h
 *
 */
#ifndef DISPLAY_H_
#define DISPLAY_H_

void ssd1306_init();
void ssd1306_display_clear();
void ssd1306_display_text(char *text);
void ssd1306_display_OnOff(bool);
extern bool dispayOnOff;

#endif /* DISPLAY_ */
