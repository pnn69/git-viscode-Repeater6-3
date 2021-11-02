/*
 * i2c.h
 *
 */
#include <stdbool.h>
#ifndef I2C_H_
#define I2C_H_

#define A_4051 3
#define B_4051 4
#define C_4051 5
#define buzzer 6    // buzzer on pcb IO extender
#define approxPin 6 // presence detection

extern bool T0, T1, T2, key0, key1, key2;
extern int TF0, R0, TF1, R1, TF2, R2;

#define I2C0_EXPANDER_ADDRESS_MUX (0x20)
#define I2C0_EXPANDER_ADDRESS_NTC (0x21)
#define I2C0_EXPANDER_ADDRESS_KEY (0x24)
#define I2C0_DISPLAY_ADDRESS (0x3C)

#define I2C_SCL_IO 21      //			 /*!< gpio number for I2C master clock */
#define I2C_SDA_IO 22      //			 /*!< gpio number for I2C master data  */
#define I2C_FREQ_HZ 400000 /*!< I2C master clock frequency */
//#define I2C_FREQ_HZ				400000           /*!< I2C master
// clock frequency */
#define I2C_PORT_NUM I2C_NUM_1     /*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE 0       /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE 0       /*!< I2C master do not need buffer */
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

extern int aproxtimer;
extern bool approx;
int ADC_256[15];
float VP1, VP2, VP3, VP4, VP5, VP6;

void i2c_task(void *arg);
float fan_calcPercentageToVoltage(float measP);

#endif /* I2C_H_ */
