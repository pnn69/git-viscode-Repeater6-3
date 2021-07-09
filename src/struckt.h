/*
 * struckt.h
 *
 *  Created on: 26 Aug 2020
 *      Author: Peter
 */
#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef MAIN_STRUCKT_H_
#define MAIN_STRUCKT_H_
#define NVM_VERSION 7

typedef struct {
    uint8_t nvm_config;
    uint8_t nvm_version;
    bool connected;
    bool purifyvalveOn;
    bool drainOn;
    bool extventOn;
    bool pumpOn;
    bool reverse;
    bool fanOn;
    bool fanManual;
    bool ldr;
    bool cleanManual;
    float fanspeedinverter;
    float fanspeed;
    uint8_t fanspeedmanual;
    uint8_t fanspeed_set;
    uint8_t fanspeedminimal;
    uint8_t fanspeedmaximal; // 60Hz max
    uint8_t pumpspeedminimal;
    uint8_t pumpspeedmaximal;
    uint8_t floatsensor;      // 0xFF means full 0xB4 means empty
    uint8_t floatsensorError; // 0xFF means full 0xB4 means empty
    uint8_t RHmimimal;
    uint8_t RHmaximal;
    uint8_t RH_day;
    uint8_t RH_night;
    uint8_t pump_max;
    uint8_t pump_min;
    uint8_t cleantimeset;
    uint16_t circulateTimer;
    uint8_t circulateSpeed;
    uint16_t circulateTime;
    float temp_set;
    float tout;
    float current;
    unsigned long pumpTime;
    unsigned long onTime;
    unsigned long offTime;
    float P;
    float I;
    float D;
} sys;

typedef struct {
    uint8_t nvm_config;
    uint8_t nvm_version;
    bool connected;
    float Pset;
    float Tempset;
    float TempDelta;
    uint8_t speedminimalfan;
    uint8_t speedmaximalfan;
    uint8_t speedminimalpump;
    uint8_t speedmaximalpump;
    float speedfan;
    float Pfan;
    float Ifan;
    float Dfan;
    float iErrorfan;
    float iErrorfanmax;
    float dEerrorfan;
    float dEerrorfanmax;

    float speedpump;
    float Ppump;
    float Ipump;
    float Dpump;
    float iErrorpump;
    float iErrorpumpmax;
    float dEerrorpump;
    float dEerrorpumpmax;

    float Phigh;
    float Plow;
    float current;

    float Thigh;
    float Tlow;
    float Tin;
    float Tout;

} inverter;

typedef struct {
    float NVMVgain;
    float dProbeA5v;
    float dProbeB5v;
    float dProbeC5v;
} sysparm;

int mode;
uint8_t nvm_version;
float NTC_offset[6];

esp_err_t store_struckt_name(char *nameStore, void *name, size_t strucktSize);
esp_err_t restore_struckt_name(char *nameStore, void *name, size_t strucktSize);
int32_t read_nvmint32(char *nameStore);
esp_err_t write_nvmint32(char *nameStore, int32_t b);
void initKabel(void);
void initFan(void);
void initPump(void);

sys Kabel400;
sys OuleMould;
sys WeifangMuhe;
sys setup;
inverter Pump;
inverter Fan;
sysparm NVMsystem;
#endif /* MAIN_STRUCKT_H_ */
