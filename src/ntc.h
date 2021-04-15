/*
 * ntc.h
 *
 */
#ifndef NTC_H_
#define NTC_H_

extern float NTC[6];
double new_ntc_sample(int);
float new_ntc_sample5v(float);
void initNTC_offset(void);
void CalibrateNTC(void);
void calibrate(int countl, int counth);


#endif /* NTC_H_ */
