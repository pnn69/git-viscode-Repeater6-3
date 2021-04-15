/*
*NTC stuff
*
* NTC connected to 5V
* retuns temp in degrees celcius
* Uses Steinhart-Hart correction
* Steinhart-Hart Coefficient Calculator:  https://docs.google.com/spreadsheets/d/1loZOUvEIKg3FN3XMkUbffdzAgFlPnjNALKsVrhj87Dw/edit?ts=5efea640#gid=0
*/
#include <math.h>
#include <esp_log.h>
#include "global.h"
#include "ntc.h"
#include "struckt.h"
#include "adc.h"

#define TAG "NTC"



float NTC[6];


// Steinhart-Hart coeffecients from the spreadsheet.
double                      dKelvin = 273.15;                     // degrees kelvin
double                     	dDegreesC = 0.0;                      // calculated degrees Ca

double                      dResistor = 3200;                      // in ohms
//double  					dProbeA = -2.7867643E-03;
//double                      dProbeB = 7.6048367E-04;              // value B from spreadsheet
//double                      dProbeC = -1.5659170E-06;              // value C from spreadsheet
double  					dProbeA = -3.9974457E-03;
double                      dProbeB = 9.2112399E-04;              // value B from spreadsheet
double                      dProbeC = -1.9562966E-06;              // value C from spreadsheet


void initNTC_offset(void){
	for(int i=0;i<6;i++){
		NTC_offset[i]=0;
	}
	store_struckt_name("NTC_offset",&NTC_offset,sizeof(NTC_offset));
}

double new_ntc_sample(int ntc){
	double  dThermistor = dResistor * ((4095.0 / (double)ntc) - 1.0);
    // The Steinhart-Hart equation uses log(dRThermistor) four times, so calculate it once.
    double dLogdRThermistor = log(dThermistor);
    // Then calculate degrees C.
    return (1.0 / (dProbeA + (dProbeB * dLogdRThermistor) + (dProbeC * dLogdRThermistor * dLogdRThermistor * dLogdRThermistor)) - dKelvin);
}
float new_ntc_sample5v(float ntc){
	float  dThermistor = dResistor * ((5000 / ntc) - 1.0);
    // The Steinhart-Hart equation uses log(dRThermistor) four times, so calculate it once.
    float dLogdRThermistor = log(dThermistor);
    // Then calculate degrees C.
    return (1.0 / (NVMsystem.dProbeA5v + (NVMsystem.dProbeB5v * dLogdRThermistor) + (NVMsystem.dProbeC5v * dLogdRThermistor * dLogdRThermistor * dLogdRThermistor)) - dKelvin);
}


#define Resistor 3200
#define ADCcount 5000
#define T0 0.42
#define T1 25
#define T2 100.13


void CalibrateNTC(void){
	printf("\r\n\r\n  ***Calibrating NTC***\r\n\r\n  Continue y\\n ...");
	char ch;
	ch = fgetc(stdin);
	while(ch == 0xFF){
		ch = fgetc(stdin);
	}
	printf("\r\n");
	if(ch == 'Y' || ch =='y'){
		for(int c = 0;c < 6;c++){	//clear offsets
			NTC_offset[c] = 0;
		}
		printf("  Put 5V loop in P1 and hit a key\r\n");
		ch = fgetc(stdin);
		while(ch == 0xFF){
			ch = fgetc(stdin);
		}
		puts("  Wait");
		for(ch = 0;ch < 10; ch++){
			vTaskDelay(100);
			puts("  .");
		}
		NVMsystem.NVMVgain = 5000/(float)ADC[0];
		printf("  ADCvalue %.0f\r\n\r\n",(float)ADC[0]);

		printf("  Put 0.42C ref resistor in P1 and hit a key\r\n");
		ch = fgetc(stdin);
		while(ch == 0xFF){
			ch = fgetc(stdin);
		}
		puts("  Wait");
		for(ch = 0;ch < 10; ch++){
			vTaskDelay(100);
			puts("  .");
		}
		float C8=ADCmV[2]; 		//take voltage value
		printf("  ADCvalue %f\r\n\r\n",ADCmV[2]);

		printf("  Put 100.13C ref resistor in P1 and hit a key\r\n");
		ch = fgetc(stdin);
		while(ch == 0xFF){
			ch = fgetc(stdin);
		}
		puts("  Wait");
		for(ch = 0;ch < 10; ch++){
			vTaskDelay(100);
			puts("  .");
		}
		float C10=ADCmV[2]; 		//take voltage value
		printf("  ADCvalue %f\r\n\r\n",ADCmV[2]);

		//take voltage value
		printf("  Put 25.00C ref resistor in P1 and hit a key\r\n");
		ch = fgetc(stdin);
		while(ch == 0xFF){
			ch = fgetc(stdin);
		}
		puts("  Wait");
		for(ch = 0;ch < 10; ch++){
			vTaskDelay(100);
			puts("  .");
		}
		float C9=ADCmV[2]; 		//take voltage value
		printf("  ADCvalue %f\r\n\r\n",ADCmV[2]);
		//take voltage value
		float E8 = T0 + 273.15;
		float E9 = T1 + 273.15;
		float E10 = T2 + 273.15;
		float G8 = Resistor*((ADCcount/C8)-1);
		float G9 = Resistor*((ADCcount/C9)-1);
		float G10 = Resistor*((ADCcount/C10)-1);
		float H8 = log(G8);
		float H9 = log(G9);
		float H10 = log(G10);
		float I8 = 1/E8;
		float I9 = 1/E9;
		float I10 = 1/E10;
		float I13 = (I10 - I8) / (H10 - H8);
		float I12 = (I9 - I8) / (H9 - H8);
		float K10 = ((I13 - I12) / (H10 - H9)) / (H8 + H9 + H10);
		float K9 = I12 - K10*(H8*H8 + H8 * H9 + H9*H9);
		float K8 = I8 - (K9 + H8*H8*K10)*H8;

		NVMsystem.dProbeA5v = K8;
		NVMsystem.dProbeB5v = K9;
		NVMsystem.dProbeC5v = K10;
		printf("  Coefficient A:%E\r\n  Coefficient B:%E\r\n  Coefficient C:%E\r\n\n\r",NVMsystem.dProbeA5v,NVMsystem.dProbeB5v,NVMsystem.dProbeC5v);
		printf("  If 25.00C ref resistor still connected you should see 25.0C on Tin\r\n\r\n");
		printf("  Store new parametes? y\\n ...");
		ch = fgetc(stdin);
		while(ch == 0xFF){
			ch = fgetc(stdin);
		}
		if(ch == 'y' || ch =='Y'){
			printf("\r\n");
			store_struckt_name("NVMsystem",&NVMsystem,sizeof(NVMsystem));
			store_struckt_name("NTC_offset",&NTC_offset,sizeof(NTC_offset));
		}
		printf("\r\n");
		printf("  ***Calibrating NTC Done***\r\n");
	}else{
		return;
	}



}
