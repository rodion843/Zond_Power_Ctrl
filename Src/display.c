/*
 * display.c
 *
 *  Created on: Jun 24, 2021
 *      Author: user0051
 */
#include "main.h"
extern int TEMPVALUE;
void debug_disp(void){
	char str[25];
//		PWR_CTRL.ADC_busy = true;
//		Get_Values();

//		Enable_5V();
	PWR_CTRL.Five_V_On = true;
//		HAL_Delay(150);
	// инициализация после включения


	// Отображаем результаты измерений напряжения
	char debug_string[] = "Li Pb CC RC Cb Lib";
	Display_Info(debug_string, 1, 0, Font_7x10);
	sprintf(str, "%2d %2d %2d %2d %2d %2d", (int)(PWR_CTRL.Voltage_Li_Ion + 0.5),
											(int)(PWR_CTRL.Voltage_Pb + 0.5),
											(int)(PWR_CTRL.Charge_Current + 0.5),
											(int)(PWR_CTRL.Rack_Current + 0.5),
											(int)(PWR_CTRL.Charger_On),
											(int)(PWR_CTRL.Li_Ion_On)
			);

	Display_Info(str, 1, 12, Font_7x10);

	sprintf(debug_string, "Rb Bb T");
	Display_Info(debug_string, 1, 27, Font_7x10);
	sprintf(str, "%2d %2d %2d", (int)PWR_CTRL.Rack_On,
								(int)PWR_CTRL.Beep_On,
								(int)TEMPVALUE
		   );
	Display_Info(str, 1, 39, Font_7x10);

	SSD1306_UpdateScreen();

	//	SSD1306_OFF();
//		Disable_5V();
//		PWR_CTRL.Five_V_On = false;
}

