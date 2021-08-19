/*
 *NTC stuff
 *
 * NTC connected to 5V
 * retuns temp in degrees celcius
 * Uses Steinhart-Hart correction
 * Steinhart-Hart Coefficient Calculator:
 *https://docs.google.com/spreadsheets/d/1loZOUvEIKg3FN3XMkUbffdzAgFlPnjNALKsVrhj87Dw/edit?ts=5efea640#gid=0
 */
#include "ntc.h"

#include <esp_log.h>
#include <math.h>

#include "adc.h"
#include "global.h"
#include "struckt.h"

#define TAG "NTC"
float NTC[8];

// Steinhart-Hart coeffecients from the spreadsheet.
double dKelvin = 273.15; // degrees kelvin
double dDegreesC = 0.0;  // calculated degrees Ca

double dResistor = 1000; // in ohms
// double  					dProbeA = -2.7867643E-03;
// double                      dProbeB = 7.6048367E-04;              // value B
// from spreadsheet double                      dProbeC = -1.5659170E-06; //
// value C from spreadsheet
double dProbeA = 2.8816881E+03;
double dProbeB = -4.8241469E+02; // value B from spreadsheet
double dProbeC = 2.0000000E+00;  // value C from spreadsheet
uint32_t cnt = 4095;

void initNTC_offset(void) {
    for (int i = 0; i < 6; i++) {
        NTC_offset[i] = 0;
    }
    store_struckt_name("NTC_offset", &NTC_offset, sizeof(NTC_offset));
}

double new_ntc_sample(int ntc) {
    double dThermistor = dResistor * (((double)cnt / (double)(cnt - ntc)) - 1.0);
    // The Steinhart-Hart equation uses log(dRThermistor) four times, so calculate
    // it once.
    double dLogdRThermistor = log(dThermistor);
    // Then calculate degrees C.
    return (1.0 / (dProbeA + (dProbeB * dLogdRThermistor) + (dProbeC * dLogdRThermistor * dLogdRThermistor * dLogdRThermistor)) - dKelvin);
}
float new_ntc_sample5v(float ntc) {
    float dThermistor = dResistor * ((5000 / ntc) - 1.0);
    // The Steinhart-Hart equation uses log(dRThermistor) four times, so calculate
    // it once.
    float dLogdRThermistor = log(dThermistor);
    // Then calculate degrees C.
    return (1.0 / (NVMsystem.dProbeA5v + (NVMsystem.dProbeB5v * dLogdRThermistor) + (NVMsystem.dProbeC5v * dLogdRThermistor * dLogdRThermistor * dLogdRThermistor)) - dKelvin);
}

#define Resistor 20000
#define ADCcount 5000
#define T0 0.42
#define T1 25
#define T2 100.13

void CalibrateNTC(void) {
    printf("\r\n\r\n  ***Calibrating NTC***\r\n\r\n  Continue y\\n ...");
    char ch;
    ch = fgetc(stdin);
    while (ch == 0xFF) {
        ch = fgetc(stdin);
    }
    printf("\r\n");
    if (ch == 'Y' || ch == 'y') {
        for (int c = 0; c < 6; c++) { // clear offsets
            NTC_offset[c] = 0;
        }

        printf("  Put 0.42C ref resistor in P1 and hit a key\r\n");
        ch = fgetc(stdin);
        while (ch == 0xFF) {
            ch = fgetc(stdin);
        }
        puts("  Wait");
        for (ch = 0; ch < 5; ch++) {
            vTaskDelay(100);
            puts("  .");
        }
        // float C8 = ADCmV[2];  // take voltage value
        float C8 = cnt - ADC[0];
        printf("  ADCvalue %04d\r\n\r\n", ADC[0]);

        printf("  Put 100.13C ref resistor in P1 and hit a key\r\n");
        ch = fgetc(stdin);
        while (ch == 0xFF) {
            ch = fgetc(stdin);
        }
        puts("  Wait");
        for (ch = 0; ch < 5; ch++) {
            vTaskDelay(100);
            puts("  .");
        }
        float C10 = cnt - ADC[0]; // take voltage value
        printf("  ADCvalue %04d\r\n\r\n", ADC[0]);

        // take voltage value
        printf("  Put 25.00C ref resistor in P1 and hit a key\r\n");
        ch = fgetc(stdin);
        while (ch == 0xFF) {
            ch = fgetc(stdin);
        }
        puts("  Wait");
        for (ch = 0; ch < 5; ch++) {
            vTaskDelay(100);
            puts("  .");
        }
        float C9 = cnt - ADC[0]; // take voltage value
        printf("  ADCvalue %04d\r\n\r\n", ADC[0]);
        // take voltage value
        float E8 = T0 + 273.15;
        float E9 = T1 + 273.15;
        float E10 = T2 + 273.15;
        float G8 = Resistor * ((ADCcount / C8) - 1);
        float G9 = Resistor * ((ADCcount / C9) - 1);
        float G10 = Resistor * ((ADCcount / C10) - 1);
        float H8 = log(G8);
        float H9 = log(G9);
        float H10 = log(G10);
        float I8 = 1 / E8;
        float I9 = 1 / E9;
        float I10 = 1 / E10;
        float I13 = (I10 - I8) / (H10 - H8);
        float I12 = (I9 - I8) / (H9 - H8);
        float K10 = ((I13 - I12) / (H10 - H9)) / (H8 + H9 + H10);
        float K9 = I12 - K10 * (H8 * H8 + H8 * H9 + H9 * H9);
        float K8 = I8 - (K9 + H8 * H8 * K10) * H8;
        NVMsystem.NVMVgain = 1;
        NVMsystem.dProbeA5v = K8;
        NVMsystem.dProbeB5v = K9;
        NVMsystem.dProbeC5v = K10;
        printf("  Coefficient A:%E\r\n  Coefficient B:%E\r\n  Coefficient "
               "C:%E\r\n\n\r",
               NVMsystem.dProbeA5v, NVMsystem.dProbeB5v, NVMsystem.dProbeC5v);
        printf("  If 25.00C ref resistor still connected you should see 25.0C on "
               "Tin\r\n\r\n");
        printf("  Store new parametes? y\\n ...");
        ch = fgetc(stdin);
        while (ch == 0xFF) {
            ch = fgetc(stdin);
        }
        if (ch == 'y' || ch == 'Y') {
            printf("\r\n");
            store_struckt_name("NVMsystem", &NVMsystem, sizeof(NVMsystem));
        }
        printf("\r\n");
        printf("  ***Calibrating NTC Done***\r\n");
    } else {
        return;
    }
}

// input is adc counts*256
// output is mv*10
// this is the voltage on the mux inputs
// it is divided/2 before it goes to the adc, but that is all calibrated away
static float A;
static float B;

void calibrate(int countl, int counth) {
    const float refl = 3811.8;          // 0,382
    const float refh = (24950.0) / 2.0; // 1,25
    A = (refh - refl) / (counth - countl);
    B = refh - A * counth;
    //ESP_LOGI(TAG, "Calibrate %d-%d A=%f, B=%f", countl, counth, A, B);
}

// NTC lookup table, see spreadsheet
// Input is mv on the analog circuit *10
// Output is degc*1000

const int lutlen = 19;
const int lut[19 * 3] = {
    // T, R, V (x10000)
    -20000, 87430, 22628, -10000, 51820, 21476, 0,     31770, 20131, 5000,  24940,  19398, 10000, 19680,  18662, 15000, 15620,  17946, 20000,
    12470,  17266, 25000, 10000,  16633, 30000, 8064,  16060, 35000, 6538,  15548,  40000, 5327,  15099,  50000, 3592,  14374,  60000, 2472,
    13847,  70000, 1735,  13471,  80000, 1243,  13205, 90000, 908,   13017, 100000, 674,   12882, 120000, 384,   12710, 150000, 177,   12584};

int ntcLookup(int mvi, int *pt, int *pr) { // returns temp and resistance
    // a disconnected ntc computes as -9degc, this is undetected here!
    if (mvi > lut[0 + 2]) { // max mv
        *pt = lut[0];
        *pr = 0;
        //ESP_LOGI(TAG,"Err to low");
        return -1; // too low
    }
    if (mvi < lut[(lutlen - 1) * 3 + 2]) { // min mv
        *pt = lut[(lutlen - 1) * 3];       // 150C
        *pr = 0;
        //ESP_LOGI(TAG,"Err to high");
        return 1; // too high
    }

    int n;
    for (n = 0; n < lutlen * 3; n += 3) {
        if (lut[n + 2] <= mvi)
            break;
    }
    int t1 = lut[n - 3 + 0], t0 = lut[n + 0]; // t1 < t0
    int r1 = lut[n - 3 + 1], r0 = lut[n + 1]; // r1 > r0
    int v1 = lut[n - 3 + 2], v0 = lut[n + 2]; // v1 > v0
    *pt = t0 + (t1 - t0) * (mvi - v0) / (v1 - v0);
    *pr = r0 + (r1 - r0) * (mvi - v0) / (v1 - v0);
    //ESP_LOGI(TAG,"v=%d v1=%d t0=%d t1=%d t=%d r=%d", v0, v1, t0, t1, *pt, *pr);
    return 0;
}