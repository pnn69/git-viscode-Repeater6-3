/*
 * touch.h
 *
 */

#ifndef TOUCH_H_
#define TOUCH_H_

void touch_task(void *pvParameter);
void touch_task_key(void *pvParameter);
void calibrateTouch(void);
extern bool T0,T1,T2,key0,key1,key2;
extern bool  calibrate;
extern uint16_t TF0,R0,TF1,R1,TF2,R2;
#endif /* TIMER_H_ */
