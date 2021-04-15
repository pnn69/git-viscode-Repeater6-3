
/*
 * menu.c
 *
 *  Created on: 24 Jun 2020
 *      Author: Peter
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "global.h"
#include "display.h"
#include "esp_log.h"
#include "struckt.h"
#include "ntc.h"
#include "timer.h"
#include "adc.h"
#include "rs485.h"

#define TAG "menu"

#define menuhome 0
#define menuLDR 1
#define menuRH 2
#define menuFan 3
#define menuClean 4
#define menuMode 5
#define menuPID 6
#define menuInfo 7
#define menuPump 8
#define menuRHset 9
#define menuRHday 10
#define menuRHnight 11
#define menuExit 12
#define menuTout 13
#define menuBeep 14
#define menuPIDPump 15
#define menuPIDFan 16
#define menuTdelta 17
#define menuNTC 18

int menuPos[3]={0,0,0,};
static char tbuf[80];
static int t=0;
static bool blink = false;
TickType_t menustamp;

//**********************************************************************
//Home screen
//**********************************************************************

void OLED_homeScreen() {
	//ssd1306_display_clear();
	float Fanspeed = voltageFAN*10;
	if(Fanspeed > 100) Fanspeed = 100;
	if(Fanspeed < 0.85) Fanspeed = 0;
	if(xSemaphoreTake(xSemaphoreSTR,10 == pdTRUE)){
		if(mode == modeHumidifier){
			if(menustamp + 50 <= xTaskGetTickCount()){
				menustamp = xTaskGetTickCount();
				if(WaterAlarm == false && Kabel400.floatsensorError <60){
					if(!blink){
						ssd1306_display_OnOff(true);
						blink =true;
					}
				}
				else{
						if(blink) ssd1306_display_OnOff(blink);
						blink = !blink;
					}
				}
				if(WaterAlarm){
					if(Kabel400.ldr == true && DayNight == false){
						sprintf(tbuf,"Water Alarm!!!! \nFan:      %04.1f%%\nRH night: %2d.0%%\nRH act:   %4.1f%%    ",Kabel400.fanspeedinverter,Kabel400.RH_night,hum_FG6485[1]);
					}else{
						sprintf(tbuf,"Water Alarm!!!! \nFan:      %04.1f%%\nRH day:   %2d.0%%\nRH act:   %4.1f%%    ",Kabel400.fanspeedinverter,Kabel400.RH_day,hum_FG6485[1]);
					}
				}else if (Kabel400.floatsensorError >60){
					if(Kabel400.ldr == true && DayNight == false){
						sprintf(tbuf,"ShortWaterAlarm!\nFan:      %04.1f%%\nRH night: %2d.0%%\nRH act:   %4.1f%%    ",Kabel400.fanspeedinverter,Kabel400.RH_night,hum_FG6485[1]);
					}else{
						sprintf(tbuf,"ShortWaterAlarm!\nFan:      %04.1f%%\nRH day:   %2d.0%%\nRH act:   %4.1f%%    ",Kabel400.fanspeedinverter,Kabel400.RH_day,hum_FG6485[1]);
					}

				}else{
					if(Kabel400.ldr == true && DayNight == false){
						sprintf(tbuf,"Humidifier      \nFan:      %04.1f%%\nRH night: %2d.0%%\nRH act:   %4.1f%%    ",Kabel400.fanspeedinverter,Kabel400.RH_night,hum_FG6485[1]);
					}else{
						sprintf(tbuf,"Humidifier      \nFan:      %04.1f%%\nRH day:   %2d.0%%\nRH act:   %4.1f%%    ",Kabel400.fanspeedinverter,Kabel400.RH_day,hum_FG6485[1]);
					}
				}
			}
		else if(mode == modeFanAuxBoxRetro){   	sprintf(tbuf,"FanAuxBoxRetro  \nFan: %03d%%     \nOut2 %s         \nOut3 %s           ",(int)(100-Fanspeed),(dim2>500) ? "On":"Off",(dim3>500) ? "On":"Off");}
		else if(mode == modeFanPumpController){
			if(menustamp + 50 <= xTaskGetTickCount()){
				menustamp = xTaskGetTickCount();
				if(PressLowAlarm == false && PressHighAlarm == false){
					if(!blink){
						ssd1306_display_OnOff(true);
						blink =true;
					}
				}
				else{
						if(blink) ssd1306_display_OnOff(blink);
						blink = !blink;
					}
				}
				if(xSemaphoreTake(xSemaphoreNTC,10 == pdTRUE)){
					if(PressLowAlarm == false && PressHighAlarm == false){
														sprintf(tbuf,"FanPumpControl  \nTin:  %2.1f     \nTout: %2.1f     \nFan:%03d%% Pump:%d",NTC[TinPos],NTC[ToutPos],(dim1/10),(dim2<10) ? 0:1);
					}else{
														//sprintf(tbuf,"FanPumpControl  \n!!!! ALARM !!!!\nPressure<0,3bar\nFan:%03d%% Pump:%d",(dim1/10),(dim2<10) ? 0:1);
														sprintf(tbuf,"FanPumpControl  \n!!!! ALARM !!!!\nPl:%.1f Ph:%.1f  \nInput:%s           ",PressLow,PressHigh,(voltageFAN > 2  ) ? "open":"closed");
					}
					xSemaphoreGive( xSemaphoreNTC );
				}
			}
		else if(mode == modeFanPumpBoxRetro){  	sprintf(tbuf,"FanPumpBoxRetro \nFan: %03d%%     \n                \n                  ",(int)(100-Fanspeed));}
		else 									sprintf(tbuf,"No setup        \nHW:%s           \nSW:%s           \nMEM:%d            ",HW_VERSION,SW_VERSION,NVM_VERSION);
		xSemaphoreGive(xSemaphoreSTR);
	}
	ssd1306_display_text(tbuf);
    menuPos[0] = 0;
    menuPos[1] = 0;
    menuPos[2] = 0;
}

//**********************************************************************
//Info screen
//**********************************************************************

void OLED_infoScreen() {
	ssd1306_display_clear();
	if(mode == modeHumidifier) 				sprintf(tbuf,"Humidifier %s   \n%s.%d           \nFG645:%d Inv:%s \n                ",HW_VERSION,SW_VERSION,NVM_VERSION,nrFG645,(Kabel400.connected) ? "On":"Off" );
	else if(mode== modeFanAuxBoxRetro)		sprintf(tbuf,"FanAuxBoxRetro  \nHW:%s           \nSW:%s           \nMEM:%d          ",HW_VERSION,SW_VERSION,NVM_VERSION);
	else if(mode == modeFanPumpController)	sprintf(tbuf,"PumpControl     \nHW:%s           \nSW:%s           \nMEM:%d          ",HW_VERSION,SW_VERSION,NVM_VERSION);
	else if(mode == modeFanPumpBoxRetro)	sprintf(tbuf,"FanPumpBoxRetro \nHW:%s           \nSW:%s           \nMEM:%d          ",HW_VERSION,SW_VERSION,NVM_VERSION);
	else 									sprintf(tbuf,"No setup        \nHW:%s           \nSW:%s           \nMEM:%d          ",HW_VERSION,SW_VERSION,NVM_VERSION);
	ssd1306_display_text(tbuf);
    menuPos[0] = 0;
    menuPos[1] = 0;
    menuPos[2] = 0;
}
void OLED_UnlockScreen(int inp) {
	if(inp == 0){
		ssd1306_display_clear();
		sprintf(tbuf,"SWIPE TO UNLOCK\n------>");
	}
	if(inp == 1)sprintf(tbuf,"SWIPE TO UNLOCK\n\n*");
	if(inp == 2)sprintf(tbuf,"SWIPE TO UNLOCK\n\n*      *");
	if(inp == 3)sprintf(tbuf,"SWIPE TO UNLOCK\n\n*      *       *");
	ssd1306_display_text(tbuf);
}





//**********************************************************************
//NTC setup
//**********************************************************************
int setNTC(int inp){
    if(menuPos[1] == 0 && menuPos[2] == 0){
    	ssd1306_display_text("NTC         \nSETUP        ");
    	if(inp == UP){
            menuPos[2]++;
            inp=0;
        }
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 0;
            inp=0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 1){
    	ssd1306_display_text("NTC         \nEXIT         ");
        if(inp == UP){
        	ssd1306_display_text("NTC         \nSETUP        ");
        	menuPos[1] = 0;
        	menuPos[2] = 0;
            inp = 0;
        }
    	if(inp == ENTER){
    		ssd1306_display_text("NTC    SETUP      \n            ");
    		menuPos[0] = 0;
    		menuPos[1] = menuNTC;
	        menuPos[2] = 0;
            inp = 0;
            return 1;
        }
    }
	if(menuPos[1] == 1 && menuPos[2] == 0){
		if(inp == UP && NTC_offset[0] < 10){
			NTC_offset[0] += 0.1;
			NTC_offset[1] += 0.1;
		}
		if(inp == DOWN && NTC_offset[0] > -10){
			NTC_offset[0] -= 0.1 ;
			NTC_offset[1] -= 0.1;
		}
		if(xSemaphoreTake(xSemaphoreNTC,10 == pdTRUE)){
			sprintf(tbuf,"NTC offset \n%02.1fC       \nNTC act \n%02.1fC       \n",NTC_offset[0],NTC[ToutPos]);
			xSemaphoreGive(xSemaphoreNTC);
		}
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("NTC             \nSETUP                \n                              \n                              ");
			menuPos[1] = 0;
			menuPos[2] = 0;
			vTaskDelay(pdMS_TO_TICKS(250));
			return 0;
		}
	}
	return 0;
}




//**********************************************************************
//Temp setup Pump
//**********************************************************************
int setTOUTpump(inverter *name,int inp){
    if(menuPos[1] == 0 && menuPos[2] == 0){
    	ssd1306_display_text("Tout        \nSETUP        ");
    	if(inp == UP){
            menuPos[2]++;
            inp=0;
        }
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 0;
            inp=0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 1){
    	ssd1306_display_text("Tout        \nEXIT         ");
        if(inp == UP){
        	ssd1306_display_text("Tout        \nSETUP        ");
        	menuPos[1] = 0;
        	menuPos[2] = 0;
            inp = 0;
        }
    	if(inp == ENTER){
    		ssd1306_display_text("Tout SETUP      \n            ");
    		menuPos[0] = 0;
    		menuPos[1] = menuTout;
	        menuPos[2] = 0;
            inp = 0;
            return 1;
        }
    }
	if(menuPos[1] == 1 && menuPos[2] == 0){
		if(inp == UP && name->Tout < 50){
			name->Tout++;
		}
		if(inp == DOWN && name->Tout > 20){
			name->Tout--;
		}
		sprintf(tbuf,"Tout   \n%02.0fC      ",name->Tout);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("Tout          \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 0;
			vTaskDelay(pdMS_TO_TICKS(250));
			return 0;
		}
	}
	return 0;
}
int setTdeltapump(inverter *name,int inp){
    if(menuPos[1] == 0 && menuPos[2] == 0){
    	ssd1306_display_text("Tdelta      \nSETUP        ");
    	if(inp == UP){
            menuPos[2]++;
            inp=0;
        }
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 0;
            inp=0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 1){
    	ssd1306_display_text("Tdelta      \nEXIT         ");
        if(inp == UP){
        	ssd1306_display_text("Tdelta      \nSETUP        ");
        	menuPos[1] = 0;
        	menuPos[2] = 0;
            inp = 0;
        }
    	if(inp == ENTER){
    		ssd1306_display_text("Tdelta SETUP      \n            ");
    		menuPos[0] = 0;
    		menuPos[1] = menuTout;
	        menuPos[2] = 0;
            inp = 0;
            return 1;
        }
    }
	if(menuPos[1] == 1 && menuPos[2] == 0){
		if(inp == UP && name->TempDelta < 10){
			name->TempDelta += 0.5;
		}
		if(inp == DOWN && name->TempDelta > 3){
			name->TempDelta -=0.5 ;
		}
		sprintf(tbuf,"Tdelta \n%02.1fC      ",name->TempDelta);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("Tdelta          \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 0;
			vTaskDelay(pdMS_TO_TICKS(250));
			return 0;
		}
	}
	return 0;
}

//**********************************************************************
//Temp setup Fan
//**********************************************************************
int setTOUT(sys *name,int inp){
    if(menuPos[1] == 0 && menuPos[2] == 0){
    	ssd1306_display_text("Tout        \nSETUP        ");
    	if(inp == UP){
            menuPos[2]++;
            inp=0;
        }
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 0;
            inp=0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 1){
    	ssd1306_display_text("Tout        \nEXIT         ");
        if(inp == UP){
        	ssd1306_display_text("Tout        \nSETUP        ");
        	menuPos[1] = 0;
        	menuPos[2] = 0;
            inp = 0;
        }
    	if(inp == ENTER){
    		ssd1306_display_text("Tout SETUP      \n            ");
    		menuPos[0] = 0;
    		menuPos[1] = menuTout;
	        menuPos[2] = 0;
            inp = 0;
            return 1;
        }
    }
	if(menuPos[1] == 1 && menuPos[2] == 0){
		if(inp == UP && name->tout < 50){
			name->tout++;
		}
		if(inp == DOWN && name->tout > 20){
			name->tout--;
		}
		sprintf(tbuf,"Tout   \n%02.0fC      ",name->tout);
		//sprintf(tbuf,"P    \n%2f%%      ",name->P);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("Tout          \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 0;
			vTaskDelay(pdMS_TO_TICKS(250));
			return 0;
		}
	}
	return 0;
}
int setPIDpumpFan(inverter *name,int inp){
    if(menuPos[1] == 0 && menuPos[2] == 0){
    	ssd1306_display_text("P Fan        \nSETUP        ");
    	if(inp == UP){
            menuPos[2]++;
            inp=0;
        }
    	if(inp == DOWN){
            menuPos[2]=3;
            inp=0;
    	}
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 0;
            inp=0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 1){
        ssd1306_display_text("I Fan        \nSETUP        ");
        if(inp == UP){
            menuPos[2]++;
            inp = 0;
        }
    	if(inp == DOWN){
        	ssd1306_display_text("P Fan       \nSETUP        ");
            menuPos[2]=0;
            inp=0;
    	}
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 1;
            inp = 0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 2){
        ssd1306_display_text("D Fan      \nSETUP        ");
        if(inp == UP){
            menuPos[2]++;
            inp = 0;
        }
    	if(inp == DOWN){
    		ssd1306_display_text("I Fan      \nSETUP        ");
    		menuPos[2]=1;
            inp=0;
    	}
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 2;
            inp = 0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 3){
    	ssd1306_display_text("PID Fan     \nEXIT         ");
        if(inp == UP){
        	ssd1306_display_text("P Fan       \nSETUP        ");
        	menuPos[1] = 0;
        	menuPos[2] = 0;
            inp = 0;
        }
    	if(inp == DOWN){
    		ssd1306_display_text("D Fan      \nSETUP        ");
            menuPos[2]=2;
            inp=0;
    	}

        if(inp == ENTER){
    		ssd1306_display_text("PID Fan SETUP     \n            ");
    		menuPos[0] = 0;
    		menuPos[1] = menuPIDFan;
	        menuPos[2] = 0;
            inp = 0;
            return 1;
        }
    }
	if(menuPos[1] == 1 && menuPos[2] == 0){
		//ESP_LOGI(TAG,"Menu P");
		if(inp == UP && name->Pfan < 100){
			name->Pfan++;
		}
		if(inp == DOWN && name->Pfan > 1){
			name->Pfan--;
		}
		sprintf(tbuf,"P fan    \n%02.0f      ",name->Pfan);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("P fan      \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 0;
			vTaskDelay(pdMS_TO_TICKS(250));
			return 0;
		}
	}
	if(menuPos[1] == 1 && menuPos[2] == 1){
		if(inp == UP && name->Ifan < 100){
			name->Ifan++;
		}
		if(inp == DOWN && name->Ifan > 0){
			name->Ifan--;
		}
		sprintf(tbuf,"I fan    \n%02.0f      ",name->Ifan);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("I fan       \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 1;
			vTaskDelay(pdMS_TO_TICKS(250));
			return 0;
		}
	}
	if(menuPos[1] == 1 && menuPos[2] == 2){
		if(inp == UP && name->Dfan < 100){
			name->Dfan++;
		}
		if(inp == DOWN && name->Dfan > 0){
			name->Dfan--;
		}
		sprintf(tbuf,"D  fan   \n%02.0f      ",name->Dfan);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("D fan       \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 2;
			return 0;
		}
	}
    return 0;

}

//**********************************************************************
//PID Pump setup
//**********************************************************************
int setPIDpumpPump(inverter *name,int inp){
    if(menuPos[1] == 0 && menuPos[2] == 0){
    	ssd1306_display_text("P Pump           \nSETUP        ");
    	if(inp == UP){
            menuPos[2]++;
            inp=0;
        }
    	if(inp == DOWN){
            menuPos[2]=3;
            inp=0;
    	}
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 0;
            inp=0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 1){
        ssd1306_display_text("I Pump     \nSETUP        ");
        if(inp == UP){
            menuPos[2]++;
            inp = 0;
        }
    	if(inp == DOWN){
        	ssd1306_display_text("P Pump      \nSETUP        ");
            menuPos[2]=0;
            inp=0;
    	}
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 1;
            inp = 0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 2){
        ssd1306_display_text("D Pump     \nSETUP        ");
        if(inp == UP){
            menuPos[2]++;
            inp = 0;
        }
    	if(inp == DOWN){
    		ssd1306_display_text("I Pump     \nSETUP        ");
    		menuPos[2]=1;
            inp=0;
    	}
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 2;
            inp = 0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 3){
    	ssd1306_display_text("PID Pump    \nEXIT         ");
        if(inp == UP){
        	ssd1306_display_text("P Pump      \nSETUP        ");
        	menuPos[1] = 0;
        	menuPos[2] = 0;
            inp = 0;
        }
    	if(inp == DOWN){
    		ssd1306_display_text("D Pump     \nSETUP        ");
            menuPos[2]=2;
            inp=0;
    	}

        if(inp == ENTER){
    		ssd1306_display_text("PID SETUP       \n            ");
    		menuPos[0] = 0;
    		menuPos[1] = menuPIDPump;
	        menuPos[2] = 0;
            inp = 0;
            return 1;
        }
    }
	if(menuPos[1] == 1 && menuPos[2] == 0){
		//ESP_LOGI(TAG,"Menu P");
		if(inp == UP && name->Ppump < 100){
			name->Ppump++;
		}
		if(inp == DOWN && name->Ppump > 1){
			name->Ppump--;
		}
		sprintf(tbuf,"P Pump        \n%02.0f      ",name->Ppump);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("P Pump      \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 0;
			vTaskDelay(pdMS_TO_TICKS(250));
			return 0;
		}
	}
	if(menuPos[1] == 1 && menuPos[2] == 1){
		if(inp == UP && name->Ipump < 100){
			name->Ipump++;
		}
		if(inp == DOWN && name->Ipump > 0){
			name->Ipump--;
		}
		sprintf(tbuf,"I Pump   \n%02.0f      ",name->Ipump);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("I           \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 1;
			vTaskDelay(pdMS_TO_TICKS(250));
			return 0;
		}
	}
	if(menuPos[1] == 1 && menuPos[2] == 2){
		if(inp == UP && name->Dpump < 100){
			name->Dpump++;
		}
		if(inp == DOWN && name->Dpump > 0){
			name->Dpump--;
		}
		sprintf(tbuf,"D Pump   \n%02.0f      ",name->Dpump);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("D           \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 2;
			return 0;
		}
	}
    return 0;

}

//**********************************************************************
//PID Humidifier setup
//**********************************************************************
int setPID(sys *name,int inp){
    if(menuPos[1] == 0 && menuPos[2] == 0){
    	ssd1306_display_text("P           \nSETUP        ");
    	if(inp == UP){
            menuPos[2]++;
            inp=0;
        }
    	if(inp == DOWN){
            menuPos[2]=3;
            inp=0;
    	}
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 0;
            inp=0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 1){
        ssd1306_display_text("I          \nSETUP        ");
        if(inp == UP){
            menuPos[2]++;
            inp = 0;
        }
    	if(inp == DOWN){
        	ssd1306_display_text("P           \nSETUP        ");
            menuPos[2]=0;
            inp=0;
    	}
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 1;
            inp = 0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 2){
        ssd1306_display_text("D          \nSETUP        ");
        if(inp == UP){
            menuPos[2]++;
            inp = 0;
        }
    	if(inp == DOWN){
    		ssd1306_display_text("I          \nSETUP        ");
    		menuPos[2]=1;
            inp=0;
    	}
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 2;
            inp = 0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 3){
    	ssd1306_display_text("PID         \nEXIT         ");
        if(inp == UP){
        	ssd1306_display_text("P           \nSETUP        ");
        	menuPos[1] = 0;
        	menuPos[2] = 0;
            inp = 0;
        }
    	if(inp == DOWN){
    		ssd1306_display_text("D          \nSETUP        ");
            menuPos[2]=2;
            inp=0;
    	}

        if(inp == ENTER){
    		ssd1306_display_text("PID SETUP       \n            ");
    		menuPos[0] = 0;
    		menuPos[1] = menuPID;
	        menuPos[2] = 0;
            inp = 0;
            return 1;
        }
    }
	if(menuPos[1] == 1 && menuPos[2] == 0){
		//ESP_LOGI(TAG,"Menu P");
		if(inp == UP && name->P < 100){
			name->P++;
		}
		if(inp == DOWN && name->P > 1){
			name->P--;
		}
		sprintf(tbuf,"P    \n%02.0f      ",name->P);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("P           \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 0;
			vTaskDelay(pdMS_TO_TICKS(250));
			return 0;
		}
	}
	if(menuPos[1] == 1 && menuPos[2] == 1){
		if(inp == UP && name->I < 100){
			name->I++;
		}
		if(inp == DOWN && name->I > 0){
			name->I--;
		}
		sprintf(tbuf,"I    \n%02.0f      ",name->I);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("I           \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 1;
			vTaskDelay(pdMS_TO_TICKS(250));
			return 0;
		}
	}
	if(menuPos[1] == 1 && menuPos[2] == 2){
		if(inp == UP && name->D < 100){
			name->D++;
		}
		if(inp == DOWN && name->D > 0){
			name->D--;
		}
		sprintf(tbuf,"D    \n%02.0f      ",name->D);
		ssd1306_display_text(tbuf);
		if(inp == ENTER){
			ssd1306_display_text("D           \nSETUP                 ");
			menuPos[1] = 0;
			menuPos[2] = 2;
			return 0;
		}
	}
    return 0;
}

//**********************************************************************
//Mode setup
//**********************************************************************
void setMODE(int inp){
	if(inp == 0){
		t=mode;
	}
	if(inp == UP){
		if(t < modeFanPumpBoxRetro){
			t++;
		}else{
			t = modeHumidifier;
		}
	}
	ESP_LOGI(TAG,"Mode %d",t);
	switch (t){
	case modeHumidifier:
		ssd1306_display_text("Humidifier         \n                  \nSelect push DOWN");
		break;
	case modeFanAuxBoxRetro:
		ssd1306_display_text("FanAuxBoxRetro    \n                   \nSelect push DOWN");
		break;
	case modeFanPumpController:
		ssd1306_display_text("FanPumpControl    \n                   \nSelect push DOWN");
		break;
	case modeFanPumpBoxRetro:
		ssd1306_display_text("PumpBoxRetro      \n                   \nSelect push DOWN");
		break;
	}
	if(inp == DOWN){
    	if( read_nvmint32("MODE") != t){
    		ESP_LOGI(TAG,"MODE set to %d",t);
    		write_nvmint32("MODE",t);
    		ssd1306_display_text("Store new data    \n                    \n                  ");
    		vTaskDelay(pdMS_TO_TICKS(250));
    		ssd1306_display_text("\n*");
    		vTaskDelay(pdMS_TO_TICKS(250));
    		ssd1306_display_text("\n**");
    		vTaskDelay(pdMS_TO_TICKS(250));
    		ssd1306_display_text("\n***");
    		vTaskDelay(pdMS_TO_TICKS(250));
    		ssd1306_display_text("\n****");
    		vTaskDelay(pdMS_TO_TICKS(250));
    		ssd1306_display_text("\n*****\nRebooting...");
    		vTaskDelay(pdMS_TO_TICKS(1000));
    		esp_restart();
    	}
	}
	if(inp == ENTER ){
    	ssd1306_display_text("MODE SETUP      \n                  ");
    	menuPos[0] = 0;
        menuPos[1] = 5;
	}
}


//**********************************************************************
//Clean setup
//**********************************************************************
void setCLEAN(int inp){
	if(inp == 0){
		menuPos[1] = 0;
		menuPos[2] = 0;
	}
	if(menuPos[1] == 0 && menuPos[2] == 0){
		ssd1306_display_text("CLEAN AUTO/MAN    \nSETUP         ");
		if(inp == ENTER){
			menuPos[1] = 1;
			menuPos[2] = 0;
			inp = 0;
		}
		if(inp == UP){
			menuPos[2] = 1;
			inp = 0;
		}
		if(inp == DOWN){
			menuPos[2] = 3;
			inp = 0;
		}
	}
	if(menuPos[1] == 0 && menuPos[2] == 1){
		ssd1306_display_text("CLEAN PERIOD   \nSETUP           ");
		if(inp == ENTER){
			menuPos[1] = 1;
			menuPos[2] = 1;
			inp = 0;
		}
		if(inp == UP){
			menuPos[2] = 2;
			inp = 0;
		}
		if(inp == DOWN){
			ssd1306_display_text("CLEAN AUTO/MAN    \nSETUP         ");
			menuPos[2] = 0;
			inp = 0;
		}
	}
	if(menuPos[1] == 0 && menuPos[2] == 2){
		ssd1306_display_text("CLEAN PERIOD   \nMANUAL SETUP     ");
		if(inp == ENTER){
			menuPos[1] = 1;
			menuPos[2] = 2;
			inp = 0;
		}
		if(inp == UP){
			menuPos[2] = 3;
			inp = 0;
		}
		if(inp == DOWN){
			ssd1306_display_text("CLEAN PERIOD   \nSETUP           ");
			menuPos[2] = 1;
			inp = 0;
		}
	}
	if(menuPos[1] == 0 && menuPos[2] == 3){
		ssd1306_display_text("CLEAN SETUP   \nEXIT           \n                    \n             ");
		if(inp == ENTER){
			ssd1306_display_text("CLEAN SETUP                                 ");
			menuPos[0] = 0;
			menuPos[1] = 4;
			menuPos[2] = 0;
			store_struckt_name("Kabel400",&Kabel400,sizeof(Kabel400));
			return;
		}
		if(inp == UP){
			ssd1306_display_text("CLEAN AUTO/MAN   \nSETUP                     ");
			menuPos[1] = 0;
			menuPos[2] = 0;
			inp = 0;
		}
		if(inp == DOWN){
			ssd1306_display_text("CLEAN PERIOD   \nMANUAL SETUP     ");
			menuPos[2] = 2;
			inp = 0;
		}
	}
	if(menuPos[1] == 1 && menuPos[2] == 0){
		if(inp == ENTER){
			ssd1306_display_text("CLEAN AUTO/MAN   \nSETUP                    ");
			menuPos[1] = 0;
			menuPos[2] = 0;
			return;
		}
		if(inp == UP){Kabel400.cleanManual =! Kabel400.cleanManual;}
		if(inp == DOWN){Kabel400.cleanManual =! Kabel400.cleanManual;}
		if(Kabel400.cleanManual){sprintf(tbuf,"CLEAN:AUTO                       ");}
		else{sprintf(tbuf,"CLEAN:MANUAL                         ");}
		ssd1306_display_text(tbuf);
	}
	if(menuPos[1] == 1 && menuPos[2] == 1){
		if(inp == ENTER){
			ssd1306_display_text("CLEAN PERIOD     \nSETUP                      ");
			menuPos[1] = 0;
			menuPos[2] = 1;
			return;
		}
		if(inp == UP){
			if(Kabel400.cleantimeset == 24)
				Kabel400.cleantimeset = 3;
			else if(Kabel400.cleantimeset == 12)
				Kabel400.cleantimeset = 24;
			else if(Kabel400.cleantimeset == 6)
				Kabel400.cleantimeset = 12;
			else if(Kabel400.cleantimeset == 6)
				Kabel400.cleantimeset = 12;
			else if(Kabel400.cleantimeset == 3)
				Kabel400.cleantimeset = 6;
			else Kabel400.cleantimeset = 6;
		}
		sprintf(tbuf,"CLEAN PERIOD    \n%d-Hrs   ",Kabel400.cleantimeset);
		ssd1306_display_text(tbuf);
	}
	if(menuPos[1] == 1 && menuPos[2] == 2){
		if(inp == ENTER){
			ssd1306_display_text("CLEAN AUTO/MAN  \nSETUP         ");
			menuPos[1] = 0;
			menuPos[2] = 2;
			return;
		}
		if(inp == DOWN){
			if(Kabel400.cleantimeset > 1)
			Kabel400.cleantimeset--;
		}
		if(inp == UP){
			Kabel400.cleantimeset++;
			if(Kabel400.cleantimeset > 72)
				Kabel400.cleantimeset =72;
		}
		sprintf(tbuf,"CLEAN PERIOD       \n%d-Hr MANUAL    ",Kabel400.cleantimeset);
		ssd1306_display_text(tbuf);
	}
	return;
}

//**********************************************************************
//Fan Pump setup
//**********************************************************************
int setFANpump(inverter *name,int inp){
	if(inp == 0){
        menuPos[1] = 0;
        menuPos[2] = 0;
    }
    if(menuPos[1] == 0 && menuPos[2] == 0){
    	ssd1306_display_text("FAN MAX    \nSETUP       ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 1;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = 0;
            menuPos[2] = 3;
            inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 1;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    //Level1
    if(menuPos[1] == 0 && menuPos[2] == 1){
    	ssd1306_display_text("FAN MIN      \nSETUP     ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 2;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("FAN MAX    \nSETUP       ");
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 1;
        	menuPos[2] = 1;
            inp = 0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 2){
    	ssd1306_display_text("FAN MANUAL   \nSETUP                  ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 3;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("FAN MIN      \nSETUP     ");
            menuPos[1] = 0;
            menuPos[2] = 1;
            inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 1;
        	menuPos[2] = 3;
            inp = 0;
        }
    }

    if(menuPos[1] == 0 && menuPos[2] == 3){
    	ssd1306_display_text("FAN SETUP   \nEXIT         \n                ");
        if(inp == UP){
        	menuPos[1] = 0;
        	menuPos[2] = 0;
            ssd1306_display_text("FAN MAX        \nSETUP               ");
        }
        if(inp == DOWN){
        	ssd1306_display_text("FAN MANUAL   \nSETUP                  ");
            menuPos[1] = 0;
            menuPos[2] = 2;
            inp = 0;
        }
        if(inp == ENTER){
        	ssd1306_display_text("FAN SETUP         \n           ");
            menuPos[0] = 0;
            menuPos[1] = 3;
            menuPos[2] = 0;
            return 1;
        }
    }
    if(menuPos[1] == 1 && menuPos[2] == 0){
        if(inp == UP && name->speedmaximalfan < 100){
        	name->speedmaximalfan++;
        }
        if(inp == DOWN && name->speedmaximalfan > 30 && name->speedmaximalfan > name->speedminimalfan){
        	name->speedmaximalfan--;
        }
        sprintf(tbuf,"FANSPEED MAX    \n%d%%      ",name->speedmaximalfan);
        ssd1306_display_text(tbuf);
        if(inp == ENTER){
        	ssd1306_display_text("FAN MAX     \nSETUP                 ");
            menuPos[1] = 0;
            menuPos[2] = 0;
            return 0;
        }
    }
    if(menuPos[1] == 1 && menuPos[2] == 1){
        if(inp == UP && name->speedminimalfan < 40 && name->speedminimalfan <name->speedmaximalfan){
        	name->speedminimalfan++;
        }
        if(inp == DOWN && name->speedminimalfan != 0){
        	name->speedminimalfan--;
        }

        sprintf(tbuf,"FANSPEED MIN  \n%d%%        ",name->speedminimalfan);
        ssd1306_display_text(tbuf);
        if(inp == ENTER){
        	ssd1306_display_text("FAN MIN      \nSETUP           ");
            menuPos[1] = 0;
            menuPos[2] = 1;
            return 0;
        }
    }

    if(menuPos[1] == 1 && menuPos[2] == 4){
        if(inp == 0){
        	sprintf(tbuf,"Circulate time \nSETUP    ");
        	ssd1306_display_text(tbuf);
        }
        if(inp == UP){
        	menuPos[2] = 5;
        	inp = 0;
        }
        if(inp ==  DOWN){
        	menuPos[2] = 6;
        	inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 2;
        	menuPos[2] = 0;
        	inp = 0;
        }
    }

    if(menuPos[1] == 1 && menuPos[2] == 5){
		if(inp == 0){
			sprintf(tbuf,"Circulate speed\nSETUP    ");
			ssd1306_display_text(tbuf);
		}
		if(inp == UP){
			menuPos[2] = 6;
			inp = 0;
		}
		if(inp ==  DOWN){
			sprintf(tbuf,"Circulate time \nSETUP    ");
			ssd1306_display_text(tbuf);
			menuPos[2] = 4;
			inp = 0;
		}
        if(inp == ENTER){
        	menuPos[1] = 2;
        	menuPos[2] = 1;
        	inp = 0;
        }

    }
    if(menuPos[1] == 1 && menuPos[2] == 6){
		if(inp == 0){
			sprintf(tbuf,"Circulate      \nEXIT    ");
			ssd1306_display_text(tbuf);
		}
		if(inp == UP){
        	sprintf(tbuf,"Circulate Time\nSETUP    ");
			ssd1306_display_text(tbuf);
			menuPos[2] = 4;
			inp = 0;
		}
		if(inp ==  DOWN){
			sprintf(tbuf,"Circulate speed\nSETUP    ");
			ssd1306_display_text(tbuf);
			menuPos[2] = 5;
			inp = 0;
		}
		if(inp == ENTER){
			menuPos[1] = 0;
			menuPos[2] = 4;
			ssd1306_display_text("Circulate   \nSETUP                  ");
		}

    }

	return 0;
}

//**********************************************************************
//FAN setup
//**********************************************************************
int setFAN(sys *name,int inp){
	if(inp == 0){
        menuPos[1] = 0;
        menuPos[2] = 0;
    }
    if(menuPos[1] == 0 && menuPos[2] == 0){
    	ssd1306_display_text("FAN MAX    \nSETUP       ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 1;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = 0;
            menuPos[2] = 4;
            inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 1;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    //Level1
    if(menuPos[1] == 0 && menuPos[2] == 1){
    	ssd1306_display_text("FAN MIN      \nSETUP     ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 2;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("FAN MAX    \nSETUP       ");
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 1;
        	menuPos[2] = 1;
            inp = 0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 2){
    	ssd1306_display_text("FAN SELECT   \nAUTO/MANUAL         ");
        if(inp ==UP){
            menuPos[1] = 0;
            menuPos[2] = 3;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("FAN MIN      \nSETUP             ");
            menuPos[1] = 0;
            menuPos[2] = 1;
            inp = 0;
        }
        if(inp == ENTER){
            menuPos[1] = 1;
        	menuPos[2] = 2;
            inp = 0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 3){
    	ssd1306_display_text("FAN MANUAL   \nSETUP                  ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 4;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("FAN SELECT   \nAUTO/MANUAL         ");
            menuPos[1] = 0;
            menuPos[2] = 2;
            inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 1;
        	menuPos[2] = 3;
            inp = 0;
        }
    }

    if(menuPos[1] == 0 && menuPos[2] == 4){
    	ssd1306_display_text("Circulate   \nSETUP                  ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 5;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("FAN MANUAL   \nSETUP              ");
            menuPos[1] = 0;
            menuPos[2] = 3;
            inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 1;
        	menuPos[2] = 4;
            inp = 0;
        }
    }

    if(menuPos[1] == 0 && menuPos[2] == 5){
    	ssd1306_display_text("FAN SETUP   \nEXIT         \n                ");
        if(inp == UP){
        	menuPos[1] = 0;
        	menuPos[2] = 0;
            ssd1306_display_text("FAN MAX        \nSETUP               ");
        }
        if(inp == DOWN){
        	ssd1306_display_text("Circulate   \nSETUP                  ");
            menuPos[1] = 0;
            menuPos[2] = 4;
            inp = 0;
        }
        if(inp == ENTER){
        	ssd1306_display_text("FAN SETUP         \n           ");
            menuPos[0] = 0;
            menuPos[1] = 3;
            menuPos[2] = 0;
            return 1;
        }
    }
    if(menuPos[1] == 1 && menuPos[2] == 0){
        if(inp == UP && name->fanspeedmaximal < 100){
        	name->fanspeedmaximal++;
        }
        if(inp == DOWN && name->fanspeedmaximal > 30){
        	name->fanspeedmaximal--;
        }
        sprintf(tbuf,"FANSPEED MAX    \n%d%%      ",name->fanspeedmaximal);
        ssd1306_display_text(tbuf);
        if(inp == ENTER){
        	ssd1306_display_text("FAN MAX     \nSETUP                 ");
            menuPos[1] = 0;
            menuPos[2] = 0;

            return 0;
        }
    }
    if(menuPos[1] == 1 && menuPos[2] == 1){
        if(inp == UP && name->fanspeedminimal < 40){
        	name->fanspeedminimal++;
        }
        if(inp == DOWN && name->fanspeedminimal != 0){
        	name->fanspeedminimal--;
        }

        sprintf(tbuf,"FANSPEED MIN  \n%d%%        ",name->fanspeedminimal);
        ssd1306_display_text(tbuf);
        if(inp == ENTER){
        	ssd1306_display_text("FAN MIN      \nSETUP           ");
            menuPos[1] = 0;
            menuPos[2] = 1;
            return 0;
        }
    }
    if(menuPos[1] == 1 && menuPos[2] == 2){
        if(inp == UP){name->fanManual = !name->fanManual;}
        if(inp == DOWN){name->fanManual = !name->fanManual;}
        if(name->fanManual){sprintf(tbuf,"FAN          \nFAN:AUTO    ");}
        else{sprintf(tbuf,"FAN          \nFAN:MANUAL  ");}
        ssd1306_display_text(tbuf);
        if(inp == ENTER){
        	ssd1306_display_text("FAN SELECT        \nAUTO/MANUAL    ");
            menuPos[1] = 0;
            menuPos[2] = 2;
            return 0;
        }
    }
    if(menuPos[1] == 1 && menuPos[2] == 3){
        if(inp == ENTER){
        	ssd1306_display_text("FAN MANUAL      \nSETUP     ");
            menuPos[1] = 0;
            menuPos[2] = 3;
            return 0;
        }
        if(inp == DOWN){
            if(name->fanspeedmanual > name->fanspeedminimal){
            	name->fanspeedmanual--;
            }
        }
        if(inp == UP){
        	name->fanspeedmanual++;
            if(name->fanspeedmanual > name->fanspeedmaximal ){
            	name->fanspeedmanual = name->fanspeedmaximal;
            }
        }
        sprintf(tbuf,"FANSPEED MAN\n%d%%    ",name->fanspeedmanual);
        ssd1306_display_text(tbuf);
    }

    if(menuPos[1] == 1 && menuPos[2] == 4){
        if(inp == 0){
        	sprintf(tbuf,"Circulate time \nSETUP    ");
        	ssd1306_display_text(tbuf);
        }
        if(inp == UP){
        	menuPos[2] = 5;
        	inp = 0;
        }
        if(inp ==  DOWN){
        	menuPos[2] = 6;
        	inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 2;
        	menuPos[2] = 0;
        	inp = 0;
        }
    }

    if(menuPos[1] == 1 && menuPos[2] == 5){
		if(inp == 0){
			sprintf(tbuf,"Circulate speed\nSETUP    ");
			ssd1306_display_text(tbuf);
		}
		if(inp == UP){
			menuPos[2] = 6;
			inp = 0;
		}
		if(inp ==  DOWN){
			sprintf(tbuf,"Circulate time \nSETUP    ");
			ssd1306_display_text(tbuf);
			menuPos[2] = 4;
			inp = 0;
		}
        if(inp == ENTER){
        	menuPos[1] = 2;
        	menuPos[2] = 1;
        	inp = 0;
        }

    }
    if(menuPos[1] == 1 && menuPos[2] == 6){
		if(inp == 0){
			sprintf(tbuf,"Circulate      \nEXIT    ");
			ssd1306_display_text(tbuf);
		}
		if(inp == UP){
        	sprintf(tbuf,"Circulate Time\nSETUP    ");
			ssd1306_display_text(tbuf);
			menuPos[2] = 4;
			inp = 0;
		}
		if(inp ==  DOWN){
			sprintf(tbuf,"Circulate speed\nSETUP    ");
			ssd1306_display_text(tbuf);
			menuPos[2] = 5;
			inp = 0;
		}
		if(inp == ENTER){
			menuPos[1] = 0;
			menuPos[2] = 4;
			ssd1306_display_text("Circulate   \nSETUP                  ");
		}

    }

    if(menuPos[1] == 2 && menuPos[2] == 0){
        if(inp == UP){
        	name->circulateTimer++;
        }
        if(inp == DOWN && name->circulateTimer != 0){
        	name->circulateTimer--;
        }
        sprintf(tbuf,"Circulate time  \n%d minutes delay",name->circulateTimer);
        ssd1306_display_text(tbuf);
        if(inp == ENTER){
        	ssd1306_display_text("Circulate time \nSETUP                      ");
            menuPos[1] = 1;
            menuPos[2] = 4;
            inp = 0;
        }
    }

    if(menuPos[1] == 2 && menuPos[2] == 1){
        if(inp == UP && name->circulateSpeed < 100){
        	name->circulateSpeed++;
        }
        if(inp == DOWN && name->circulateSpeed != 0){
        	name->circulateSpeed--;
        }
        sprintf(tbuf,"Circulate speed  \n%d%%       ",name->circulateSpeed);
        ssd1306_display_text(tbuf);
        if(inp == ENTER){
        	ssd1306_display_text("Circulate speed\nSETUP    ");
            menuPos[1] = 1;
            menuPos[2] = 5;
            inp = 0;
        }
    }
    return 0;
}

int setPUMPpump(inverter *name,int inp){
	if(inp == 0){
        menuPos[1] = 0;
        menuPos[2] = 0;
    }
    if(menuPos[1] == 0 && menuPos[2] == 0){
    	ssd1306_display_text("PUMP MAX    \nSETUP       ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 1;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = 0;
            menuPos[2] = 2;
            inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 1;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    //Level1
    if(menuPos[1] == 0 && menuPos[2] == 1){
    	ssd1306_display_text("PUMP MIN      \nSETUP     ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 2;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("PUMP MAX    \nSETUP       ");
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 1;
        	menuPos[2] = 1;
            inp = 0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 2){
    	ssd1306_display_text("PUMP SETUP   \nEXIT         \n                ");
        if(inp == UP){
        	menuPos[1] = 0;
        	menuPos[2] = 0;
            ssd1306_display_text("PUMP MAX        \nSETUP               ");
        }
        if(inp == ENTER){
        	ssd1306_display_text("PUMP SETUP         \n           ");
            menuPos[0] = 0;
            menuPos[1] = menuPump;
            menuPos[2] = 0;
            return 1;
        }
    }
    if(menuPos[1] == 1 && menuPos[2] == 0){
        if(inp == UP && name->speedmaximalpump < 100){
                name->speedmaximalpump++;
        }
        if(inp == DOWN && name->speedmaximalpump > 30 &&  name->speedmaximalpump > name->speedminimalpump){
                name->speedmaximalpump--;
        }
        sprintf(tbuf,"PUMPSPEED MAX    \n%d%%      ",name->speedmaximalpump);
        ssd1306_display_text(tbuf);
        if(inp == ENTER){
        	ssd1306_display_text("PUMP MAX     \nSETUP                 ");
            menuPos[1] = 0;
            menuPos[2] = 0;
        }
    }
    if(menuPos[1] == 1 && menuPos[2] == 1){
        if(inp == UP && name->speedminimalpump < 50 && name->speedminimalpump < name->speedmaximalpump ){
                name->speedminimalpump++;
        }
        if(inp == DOWN && name->speedminimalpump != 0){
                name->speedminimalpump--;
        }

        sprintf(tbuf,"PUMPSPEED MIN  \n%d%%        ",name->speedminimalpump);
        ssd1306_display_text(tbuf);
        if(inp == ENTER){
        	ssd1306_display_text("PUMP MIN      \nSETUP           ");
            menuPos[1] = 0;
            menuPos[2] = 1;
        }
    }
   return 0;
}


int setPUMP(sys *name,int inp){
	if(inp == 0){
        menuPos[1] = 0;
        menuPos[2] = 0;
    }
    if(menuPos[1] == 0 && menuPos[2] == 0){
    	ssd1306_display_text("PUMP MAX    \nSETUP       ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 1;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = 0;
            menuPos[2] = 2;
            inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 1;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    //Level1
    if(menuPos[1] == 0 && menuPos[2] == 1){
    	ssd1306_display_text("PUMP MIN      \nSETUP     ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 2;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("PUMP MAX    \nSETUP       ");
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
        if(inp == ENTER){
        	menuPos[1] = 1;
        	menuPos[2] = 1;
            inp = 0;
        }
    }
    if(menuPos[1] == 0 && menuPos[2] == 2){
    	ssd1306_display_text("PUMP SETUP   \nEXIT         \n                ");
        if(inp == UP){
        	menuPos[1] = 0;
        	menuPos[2] = 0;
            ssd1306_display_text("PUMP MAX        \nSETUP               ");
        }
        if(inp == ENTER){
        	ssd1306_display_text("PUMP SETUP         \n           ");
            menuPos[0] = 0;
            menuPos[1] = menuPump;
            menuPos[2] = 0;
            return 1;
        }
    }
    if(menuPos[1] == 1 && menuPos[2] == 0){
        if(inp == UP && name->pumpspeedmaximal < 60){
                name->pumpspeedmaximal++;
        }
        if(inp == DOWN && name->pumpspeedmaximal > 20 &&  name->pumpspeedmaximal > name->pumpspeedminimal){
                name->pumpspeedmaximal--;
        }
        sprintf(tbuf,"PUMPSPEED MAX    \n%d%%      ",name->pumpspeedmaximal);
        ssd1306_display_text(tbuf);
        if(inp == ENTER){
        	ssd1306_display_text("PUMP MAX     \nSETUP                 ");
            menuPos[1] = 0;
            menuPos[2] = 0;
        }
    }
    if(menuPos[1] == 1 && menuPos[2] == 1){
        if(inp == UP && name->pumpspeedminimal < 40 && name->pumpspeedminimal < name->pumpspeedmaximal ){
                name->pumpspeedminimal++;
        }
        if(inp == DOWN && name->pumpspeedminimal != 0){
                name->pumpspeedminimal--;
        }

        sprintf(tbuf,"PUMPSPEED MIN  \n%d%%        ",name->pumpspeedminimal);
        ssd1306_display_text(tbuf);
        if(inp == ENTER){
        	ssd1306_display_text("PUMP MIN      \nSETUP           ");
            menuPos[1] = 0;
            menuPos[2] = 1;
        }
    }
    return 0;
}

void setRH(int inp){
	if(inp == 0){
		ssd1306_display_text("RH DAY   \nSETUP        ");
        menuPos[1] = 0;
        menuPos[2] = 0;
        return;
    }
    if(menuPos[1] == 0 && menuPos[2] == 0){
        ssd1306_display_text("RH DAY   \nSETUP        ");
        if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 1;
            inp=0;
        }
        if(inp == DOWN){
        	menuPos[1] = 0;
        	menuPos[2] = 2;
        	inp=0;
        }
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 0;
            inp=0;
        }
    }

    if(menuPos[1] == 0 && menuPos[2] == 1){
    	ssd1306_display_text("RH NIGHT      \nSETUP      ");
    	if(inp == UP){
            menuPos[1] = 0;
            menuPos[2] = 2;
            inp=0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("RH DAY   \nSETUP        ");
        	menuPos[1] = 0;
        	menuPos[2] = 0;
        	inp=0;
        }
        if(inp == ENTER){
            menuPos[1] = 1;
            menuPos[2] = 1;
            inp=0;
        }
    }


    if(menuPos[1] == 0 && menuPos[2] == 2){
        ssd1306_display_text("RH SETUP      \nEXIT          \n          ");
        if(inp == UP){
        	ssd1306_display_text("RH DAY   \nSETUP       ");
            menuPos[1] = 0;
            menuPos[2] = 0;
            return;
        }
        if(inp == DOWN){
            ssd1306_display_text("RH NIGHT        \nSETUP      ");
        	menuPos[1] = 0;
        	menuPos[2] = 1;
        	inp=0;
        }
        if(inp == ENTER){
        	ssd1306_display_text("RH SETUP     \n             ");
            menuPos[0] = 0;
            menuPos[1] = 2;
            menuPos[2] = 0;
            store_struckt_name("Kabel400",&Kabel400,sizeof(Kabel400));
            return;
        }
    }


//level 2
    if(menuPos[1] == 1 && menuPos[2] == 0){
        if(inp == UP && Kabel400.RH_day < Kabel400.RHmaximal){
                Kabel400.RH_day += 1;
        }
        if(inp == DOWN && Kabel400.RH_day > Kabel400.RHmimimal){
                Kabel400.RH_day -= 1;
        }
        if(inp == ENTER){
        	ssd1306_display_text("RH DAY      \nSETUP         ");
            menuPos[1] = 0;
            menuPos[2] = 0;
            return;
        }
        sprintf(tbuf,"RH DAY\n%d%%         ",Kabel400.RH_day);
        ssd1306_display_text(tbuf);
    }

    if(menuPos[1] == 1 && menuPos[2] == 1){
        if(inp == UP && Kabel400.RH_night < Kabel400.RHmaximal){
                Kabel400.RH_night += 1;
        }
        if(inp == DOWN && Kabel400.RH_night > Kabel400.RHmimimal){
                Kabel400.RH_night -= 1;
        }
        if(inp == ENTER){
        	ssd1306_display_text("RH NIGHT   \nSETUP           ");
            menuPos[1] = 0;
            menuPos[2] = 1;
            return;
        }
        sprintf(tbuf,"RH NIGHT     \n%d%%     ",Kabel400.RH_night);
        ssd1306_display_text(tbuf);
    }
}

void setLDR(int inp){
	if(inp == 0){
		vTaskDelay(pdMS_TO_TICKS(250));
	}
	if(inp == UP || inp == DOWN){
        Kabel400.ldr = !Kabel400.ldr;
    }
    if(inp == ENTER  ){
    	ssd1306_display_clear();
    	ssd1306_display_text("LDR SETUP");
        menuPos[0] = 0;
        menuPos[1] = 1;
        store_struckt_name("Kabel400",&Kabel400,sizeof(Kabel400));
        return;
    }
    if(Kabel400.ldr == true){ssd1306_display_text("LDR ON      \nON/OFF    ");}
    else{ssd1306_display_text("LDR OFF      \nON/OFF    ");}
    return;
}

void setINFO(int inp){
	if(inp == UP || inp == ENTER){
		ssd1306_display_clear();
		OLED_homeScreen();
	}
	if(inp == DOWN){
		ssd1306_display_clear();
		if(mode == modeHumidifier) sprintf(tbuf,"Humidifier\nHW:%s\nSW:%s\nMEM:%d",HW_VERSION,SW_VERSION,NVM_VERSION);
		else if(mode== modeFanAuxBoxRetro)sprintf(tbuf,"FanAuxBoxRetro\nHW:%s\nSW:%s\nMEM:%d",HW_VERSION,SW_VERSION,NVM_VERSION);
		else if(mode == modeFanPumpController)sprintf(tbuf,"PumpControl\nHW:%s\nSW:%s\nMEM:%d",HW_VERSION,SW_VERSION,NVM_VERSION);
		else if(mode == modeFanPumpBoxRetro)sprintf(tbuf,"FanPumpBoxRetro\nHW:%s\nSW:%s\nMEM:%d",HW_VERSION,SW_VERSION,NVM_VERSION);
		else sprintf(tbuf,"No setup\nHW:%s\nSW:%s\nMEM:%d",HW_VERSION,SW_VERSION,NVM_VERSION);
		ssd1306_display_text(tbuf);
	}
}

void setBeep(int inp){
	if(inp == 0){
		vTaskDelay(pdMS_TO_TICKS(250));
	}
	if(inp == UP || inp == DOWN){
		buzzerOnOff = !buzzerOnOff;
    }
    if(inp == ENTER  ){
    	ssd1306_display_clear();
    	ssd1306_display_text("Beep SETUP");
        menuPos[0] = 0;
        menuPos[1] = 1;
        write_nvmint32("BEEP",buzzerOnOff);
        return;
    }
    if(buzzerOnOff == true){ssd1306_display_text("Beep ON      \nON/OFF    ");}
    else{ssd1306_display_text("Beep OFF      \nON/OFF    ");}
}


//**************************************************************************************************
// modeHumidifier 1
//**************************************************************************************************
void LCD_menu_1(int inp){
	if(inp == 0 && menuPos[0] == 0 && menuPos[1] == 0 && menuPos[2] == 0 ){
		OLED_homeScreen();
		return;
	}
	if(inp == 0){
		return;
	}
    if(menuPos[0] == 0 && menuPos[1] == 0){
        if(inp == ENTER){
            OLED_homeScreen();
            return;
        }
        if(inp == UP){
        	ssd1306_display_clear();
            menuPos[1] = menuLDR;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_clear();
        	menuPos[1] = menuExit;
        	inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuLDR){
    	ssd1306_display_text("LDR SETUP         ");
        if(inp == UP){ //LDR
            menuPos[1] = menuRH;
            inp = 0;
        }
        if(inp == DOWN){
        	menuPos[1] = menuExit;
        	inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuLDR;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuRH){
    	ssd1306_display_text("RH SETUP            ");
        if(inp == UP){ //RH
            menuPos[1]++;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("LDR SETUP         ");
        	menuPos[1] = menuLDR;
        	inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuRH;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuFan){
    	ssd1306_display_text("FAN SETUP         ");
        if(inp == UP){ //FAN
            menuPos[1] = menuClean;
            inp = 0;
        }
        if(inp == DOWN){

        	ssd1306_display_text("RH SETUP            ");
        	menuPos[1] = menuRH;
        	inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuFan;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuClean){
    	ssd1306_display_text("CLEAN SETUP        ");
        if(inp == UP){ //CLEAN
            menuPos[1] = menuMode;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("FAN SETUP         ");
        	menuPos[1] = menuFan;
        	inp = 0;
        }
        if(inp == ENTER){
        	ssd1306_display_text("LDR SETUP       ");
            menuPos[0] = menuClean;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }

    if(menuPos[0] == 0 && menuPos[1] == menuMode){
    	ssd1306_display_text("MODE SETUP          ");
        if(inp == UP){
            menuPos[1] = menuPID;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("CLEAN SETUP        ");
        	menuPos[1] = menuClean;
        	inp = 0;
        }
        if(inp == ENTER){
        	ssd1306_display_clear();
            menuPos[0] = menuMode;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }

    if(menuPos[0] == 0 && menuPos[1] == menuPID){
    	ssd1306_display_text("PID SETUP         ");
        if(inp == UP){
            menuPos[1] = menuBeep;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("MODE SETUP          ");
        	menuPos[1] = menuMode;
        	inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuPID;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }

    if(menuPos[0] == 0 && menuPos[1] == menuBeep){
    	ssd1306_display_text("Beep SETUP        ");
        if(inp == UP){
            menuPos[1] = menuInfo;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("PID SETUP         ");
        	menuPos[1] = menuPID;
        	inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuBeep;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }

    if(menuPos[0] == 0 && menuPos[1] == menuInfo){
    	ssd1306_display_text("SYS INFO          ");
        if(inp == UP){
            menuPos[1] = menuExit;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("Beep SETUP        ");
        	menuPos[1] = menuBeep;
        	inp = 0;
        }
        if(inp == ENTER){
        	OLED_infoScreen();
            menuPos[0] = menuInfo;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }

    if(menuPos[0] == 0 && menuPos[1] == menuExit){
    	ssd1306_display_text("EXIT           ");
        if(inp == UP){
        	ssd1306_display_text("LDR SETUP         ");
            menuPos[1] = menuLDR;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("SYS INFO          ");
        	menuPos[1] = menuInfo;
        	inp = 0;
        }
        if(inp == ENTER){
            menuPos[1] = 0;
            OLED_homeScreen();
        }
    }
    //Show menu's
    if(menuPos[0] == menuLDR ){
    	setLDR(inp);
    }
    if(menuPos[0] == menuRH ){ //main 2,0,0
    	setRH(inp);
    }
    if(menuPos[0] == menuFan ){
    	if( setFAN(&Kabel400,inp)){
    		store_struckt_name("Kabel400",&Kabel400,sizeof(Kabel400));
    	}
    }
    if(menuPos[0] == menuClean ){
    	setCLEAN(inp);
    }
    if(menuPos[0] == menuMode ){
    	setMODE(inp);
    }
    if(menuPos[0] == menuPID ){
    	if(setPID(&Kabel400,inp)){
    		store_struckt_name("Kabel400",&Kabel400,sizeof(Kabel400));
    	}
    }
    if(menuPos[0] == menuInfo ){
    	setINFO(inp);
    }
    if(menuPos[0] == menuBeep){
    	setBeep(inp);
    }
    inp=0;
}



//**********************************************************************
// define modeFanAuxBoxRetro 2
//**********************************************************************
void LCD_menu_2(int inp){
	if(inp == 0 && menuPos[0] == 0 && menuPos[1] == 0 && menuPos[2] == 0 ){
		OLED_homeScreen();
		return;
	}
	if(inp == 0){
		return;
	}
    if(menuPos[0] == 0 && menuPos[1] == 0){
        if(inp == ENTER){
            OLED_homeScreen();
            return;
        }
        if(inp == UP){
        	ssd1306_display_clear();
            menuPos[1] = menuMode;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_clear();
        	menuPos[1] = menuExit;
        	inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuMode){
    	ssd1306_display_text("MODE SETUP          ");
        if(inp == UP){
            menuPos[1] = menuExit;
            inp = 0;
        }
        if(inp == DOWN){
        	ssd1306_display_text("MODE SETUP          ");
            menuPos[1] = menuMode;
            inp = 0;
        }
        if(inp == ENTER){
        	ssd1306_display_clear();
            menuPos[0] = menuMode;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuExit){
    	ssd1306_display_text("EXIT           ");
        if(inp == UP){
        	ssd1306_display_text("MODE SETUP          ");
            menuPos[1] = menuMode;
            inp = 0;
        }
        if(inp == ENTER){
            menuPos[1] = 0;
            OLED_homeScreen();
        }
    }
    if(menuPos[0] == menuMode ){
    	setMODE(inp);
    }
	vTaskDelay(pdMS_TO_TICKS(250));
}




//**********************************************************************
// modeFanPumpController 3
//**********************************************************************
void LCD_menu_3(int inp){
	if(inp == 0 && menuPos[0] == 0 && menuPos[1] == 0 && menuPos[2] == 0 ){
		OLED_homeScreen();
		return;
	}

	if(inp == 0){
		return;
	}
    if(menuPos[0] == 0 && menuPos[1] == 0){
        if(inp == ENTER){
            OLED_homeScreen();

            return;
        }
        if(inp == UP){
        	ssd1306_display_clear();
            menuPos[1] = menuTout;
            inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuTout){
    	ssd1306_display_text("Tout SETUP        \n               ");
        if(inp == UP){
            menuPos[1] = menuTdelta;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = menuExit;
            inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuTout;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }

    if(menuPos[0] == 0 && menuPos[1] == menuTdelta){
    	ssd1306_display_text("Tdelta SETUP        \n               ");
        if(inp == UP){
            menuPos[1] = menuNTC;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = menuTout;
            ssd1306_display_text("Tout SETUP        \n               ");
            inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuTdelta;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }

    if(menuPos[0] == 0 && menuPos[1] == menuNTC){
    	ssd1306_display_text("NTC SETUP        \n               ");
        if(inp == UP){
            menuPos[1] = menuFan;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = menuTdelta;
        	ssd1306_display_text("Tdelta SETUP        \n               ");
            inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuNTC;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }


    if(menuPos[0] == 0 && menuPos[1] == menuFan){
    	ssd1306_display_text("FAN SETUP        \n               ");
        if(inp == UP){ //FAN
            menuPos[1] = menuPump;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = menuNTC;
            ssd1306_display_text("NTC SETUP        \n               ");
            inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuFan;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuPump){
    	ssd1306_display_text("Pump SETUP       \n              ");
        if(inp == UP){
            menuPos[1] = menuPIDFan;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = menuFan;
            ssd1306_display_text("FAN SETUP        \n               ");
            inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuPump;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuPIDFan){
    	ssd1306_display_text("PID Fan SETUP     \n            ");
        if(inp == UP){
            menuPos[1]= menuPIDPump;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = menuPump;
            ssd1306_display_text("Pump SETUP       \n              ");
            inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuPIDFan;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }
      if(menuPos[0] == 0 && menuPos[1] == menuPIDPump){
    	  ssd1306_display_text("PID Pump SETUP     \n            ");
          if(inp == UP){
        	  menuPos[1]= menuMode;
              inp = 0;
           }
          if(inp == DOWN){
              menuPos[1] = menuPIDFan;
          	ssd1306_display_text("PID Fan SETUP     \n            ");
              inp = 0;
          }
           if(inp == ENTER){
                menuPos[0] = menuPIDPump;
                menuPos[1] = 0;
                menuPos[2] = 0;
                inp = 0;
            }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuMode){
    	ssd1306_display_text("MODE SETUP         \n             ");
        if(inp == UP){
            menuPos[1] = menuBeep;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = menuPIDPump;
            ssd1306_display_text("PID Pump SETUP     \n            ");
            inp = 0;
        }
        if(inp == ENTER){
        	ssd1306_display_clear();
            menuPos[0] = menuMode;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }

    }
    if(menuPos[0] == 0 && menuPos[1] == menuBeep){
    	ssd1306_display_text("Beep SETUP        ");
        if(inp == UP){
            menuPos[1] = menuInfo;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = menuMode;
            ssd1306_display_text("MODE SETUP         \n             ");
            inp = 0;
        }
        if(inp == ENTER){
            menuPos[0] = menuBeep;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }

    if(menuPos[0] == 0 && menuPos[1] == menuInfo){
    	ssd1306_display_text("SYS INFO          \n               ");
        if(inp == UP){
            menuPos[1] = menuExit;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = menuBeep;
            ssd1306_display_text("Beep SETUP        ");
            inp = 0;
        }
        if(inp == ENTER){
        	OLED_infoScreen();
            menuPos[0] = menuInfo;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuExit){
    	ssd1306_display_text("EXIT          \n                ");
        if(inp == UP){
        	ssd1306_display_text("Tout SETUP        \n               ");
            menuPos[1] = menuTout;
            inp = 0;
        }
        if(inp == DOWN){
            menuPos[1] = menuInfo;
            ssd1306_display_text("SYS INFO          \n               ");
            inp = 0;
        }
        if(inp == ENTER){
            menuPos[1] = 0;
            OLED_homeScreen();

            return;
        }
    }

    //Show menu's here
    if(menuPos[0] == menuLDR ){
    	setLDR(inp);
    }
    if(menuPos[0] == menuRH ){
    	setRH(inp);
    }
    if(menuPos[0] == menuTout ){
    	if( setTOUTpump(&Fan,inp)){
    		store_struckt_name("Fan",&Fan,sizeof(Fan));
    	}
    }

    if(menuPos[0] == menuNTC ){
    	if(setNTC(inp)){
    		store_struckt_name("NTC_offset",&NTC_offset,sizeof(NTC_offset));
    	}
    }

    if(menuPos[0] == menuTdelta ){
    	if( setTdeltapump(&Pump,inp)){
    		store_struckt_name("Pump",&Pump,sizeof(Pump));
    	}
    }
    if(menuPos[0] == menuFan ){
    	if( setFANpump(&Fan,inp)){
    		store_struckt_name("Fan",&Fan,sizeof(Fan));
    	}
    }
    if(menuPos[0] == menuPump ){
    	if( setPUMPpump(&Pump,inp)){
    		store_struckt_name("Pump",&Pump,sizeof(Pump));
    	}
    }
    if(menuPos[0] == menuMode ){
		setMODE(inp);
    }
    if(menuPos[0] == menuPIDPump ){
    	if( setPIDpumpPump(&Pump,inp)){
    		store_struckt_name("Pump",&Pump,sizeof(Pump));
    	}
    }
    if(menuPos[0] == menuPIDFan ){
    	if( setPIDpumpFan(&Fan,inp)){
    		store_struckt_name("Fan",&Fan,sizeof(Fan));
    	}
    }
    if(menuPos[0] == menuInfo ){
    	setINFO(inp);
    }
    if(menuPos[0] == menuBeep){
    	setBeep(inp);
    }
    inp = 0;
	vTaskDelay(pdMS_TO_TICKS(250));
}









//**********************************************************************
//modeFanPumpBoxRetro 4
//**********************************************************************
void LCD_menu_4(int inp){
	if(inp == 0 && menuPos[0] == 0 && menuPos[1] == 0 && menuPos[2] == 0 ){
		OLED_homeScreen();
		return;
	}
	if(inp == 0){
		return;
	}
    if(menuPos[0] == 0 && menuPos[1] == 0){
        if(inp == ENTER){
            OLED_homeScreen();

            return;
        }
        if(inp == UP){
        	ssd1306_display_clear();
            menuPos[1] = menuMode;
            inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuMode){
    	ssd1306_display_text("MODE SETUP          ");
        if(inp == UP){
            menuPos[1] = menuExit;
            inp = 0;
        }
        if(inp == ENTER){
        	ssd1306_display_clear();
            menuPos[0] = menuMode;
            menuPos[1] = 0;
            menuPos[2] = 0;
            inp = 0;
        }
    }
    if(menuPos[0] == 0 && menuPos[1] == menuExit){
    	ssd1306_display_text("EXIT           ");
        if(inp == UP){
        	ssd1306_display_text("MODE SETUP          ");
            menuPos[1] = menuMode;
            inp = 0;
        }
        if(inp == ENTER){
            menuPos[1] = 0;
            OLED_homeScreen();
        }
    }
    if(menuPos[0] == menuMode ){
    	setMODE(inp);
    }
	vTaskDelay(pdMS_TO_TICKS(250));
}
