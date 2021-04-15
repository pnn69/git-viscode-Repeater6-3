/*
 *rs458.h
 *
 */
#ifndef RS485_H_
#define RS485_H_

#include <stdint.h>
#include <stdbool.h>
void RS487_init(void);
void RS487_task(void* arg);

extern volatile int nrFG645;
extern volatile float hum_FG6485[];
extern volatile float tmp_FG6485[];
extern volatile uint16_t CompressorOnTime;
extern volatile bool CompressorOn;
extern volatile float FanError;
extern volatile float PumpError;



#endif /* RS485_H_ */
