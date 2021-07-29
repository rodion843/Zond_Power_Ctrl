/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "display.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int TEMPVALUE = 0;

//uint16_t adc[4];

struct {
    uint16_t Li_Ion_Voltage;
    uint16_t Pb_Voltage;
    uint16_t Charge_Current;
    uint16_t Rack_Current;
} adc;
uint8_t UART_Buf[2];

int8_t devices;
uint8_t sensor;
OneWire ow;
Temperature t;
char *crcOK;



bool BTN_Event_Fl = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Device_Init(void);
void Get_Values(void);
void Check_Status(void);
void Sleep_Time(void);
void Beep_Beep(void);
void Display_Info(char* str, uint8_t x, uint8_t y, FontDef_t font);
int16_t Get_Temperature(void);

void BTN_Event(void);
Pwr_ctrl_t* Power_Ctrl_Init(Pwr_ctrl_t* s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Enable_Rack();
  while(1);





	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); // ��������� ���������� �� ������ �� UART1
//	Beep_Off();
  Power_Ctrl_Init(&PWR_CTRL);
	HAL_GPIO_WritePin(ON_LED_DISP_GPIO_Port, ON_LED_DISP_Pin, GPIO_PIN_SET);

	get_ROMid();

	Device_Init();
	PWR_CTRL.First_Time = true;

	SSD1306_ON();
	SSD1306_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
//		HAL_GPIO_WritePin(RACK_ON_OFF_GPIO_Port, RACK_ON_OFF_Pin, GPIO_PIN_SET);
		Get_Values();	// получаем значение основных параметров
		Check_Status();	// выполняем проверку по параметрам
		if(BTN_Event_Fl == true)
		{
			BTN_Event_Fl = false;
			BTN_Event();
		}else {

		}
		debug_disp();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
Pwr_ctrl_t* Power_Ctrl_Init(Pwr_ctrl_t* s){
	Pwr_ctrl_t temp = {
			.Charger_On = false,
			.Ingition_On = false,
			.Li_Ion_On = false,
			.Rack_On = false,
			.Five_V_On = false,
			.Twelve_V_On = false,
			.Fan_On = false,
	};
	memcpy(s, &temp, sizeof(Pwr_ctrl_t));
	return s;
}
void Device_Init(void)
{

	PWR_CTRL.ADC_busy = true;
	//включаем 12 Вольт
	Enable_12V();

	//выключаем 5 Вольт
//	Disable_5V();
	Enable_5V();

	//выключаем вентилятор
	Disable_Fan();

	//выключаем ЗУ
	Ignition_Off();
	PWR_CTRL.Charger_On = false;

	// значения АЦП не готовы
	PWR_CTRL.ADC_busy = true;
	PWR_CTRL.Voltage_Pb = 0;

	HAL_Delay(10000);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 2);


	// запускаем таймер включения ото сна
	HAL_TIM_Base_Start_IT(&htim1);
	PWR_CTRL.ADC_busy = false;
}

void Get_Values(void)
{
	// стартуем АЦП
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 4);
//	PWR_CTRL.ADC_busy = true;
	//запрашиваем температуру
	TEMPVALUE = Get_Temperature();
}

void Check_Status(void)
{
	char str[50];

	if (is_Rack_OK())
	{
		Disable_Rack();
		sprintf(str, "CURRENT OVERFLOW");
		Display_Info(str, 10, 50, Font_7x10);
		Beep_Beep();
	}
	else
	{
		Beep_Off();
		Enable_Rack();
	}

	if (is_Charge_OK())
	{
		Ignition_Off();
		PWR_CTRL.Charger_On = false;
		if (is_Charge_Alarm())
		{
			sprintf(str, "CHECK CHARGE!");
			Display_Info(str, 10, 50, Font_7x10);
			Beep_Beep();
		}
		else
		{
			Beep_Off();
		}
	}
	else
	{
		Ignition_On();
		PWR_CTRL.Charger_On = true;
	}

	if  (is_Li_Ion_OK())
	{
		Disable_Li_Ion();
	}
	else
	{
		Enable_Li_Ion();
	}

	if ((TEMPVALUE > 25) && (PWR_CTRL.Twelve_V_On == true)) Enable_Fan();
	else if ((TEMPVALUE < 20) && (PWR_CTRL.Twelve_V_On == true)) Disable_Fan();
	else if ((TEMPVALUE > 25) && (PWR_CTRL.Twelve_V_On == false))
	{
		Enable_12V();
		HAL_Delay(1);
		Enable_Fan();
	}
}

void Display_Info(char* str, uint8_t x, uint8_t y, FontDef_t font)
{
#ifdef OLED_DISPLAY
	SSD1306_GotoXY(x, y); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
	SSD1306_Puts(str, &font, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом
#elif defined (LCD_DISPLAY)
	LCD_Init();
	LCD_SetPos(x, y);
	LCD_String(str);
#endif
}

void BTN_Event(void)
{
	char Li_Ion[] = "Li-Ion";
	char Temp[] = "Temp";
	char Current[] = "Current";
	char Charge[] = "Charge";
	if (PWR_CTRL.ADC_busy == false)
	{
		char str[25];
//		PWR_CTRL.ADC_busy = true;
//		Get_Values();

//		Enable_5V();
//		HAL_Delay(150);
		// инициализация после включения
		SSD1306_ON();
		SSD1306_Init();

		// Отображаем результаты измерений напряжения

		Display_Info(Li_Ion, 5, 5, Font_16x26);
		if(PWR_CTRL.Voltage_Li_Ion < LI_ION_MIN_VOLTAGE){
			sprintf(str, "OFF");
		}else{
			sprintf(str, "%dV", (int)PWR_CTRL.Voltage_Li_Ion);
		}
		Display_Info(str, 5, 30, Font_16x26);
		SSD1306_UpdateScreen();

		// Ожидаем и после затираем экран
		HAL_Delay(3000);
		SSD1306_Fill(SSD1306_COLOR_BLACK);

		// Отображаем результаты измерений температуры
		//sprintf(str, %3d.%dC",t.inCelsus, t.frac);
		sprintf(str, "%dC", TEMPVALUE);
		Display_Info(Temp, 5, 5, Font_16x26);
		Display_Info(str, 5, 30, Font_16x26);
		SSD1306_UpdateScreen();

		// Ожидаем и после затираем экран
		HAL_Delay(3000);
		SSD1306_Fill(SSD1306_COLOR_BLACK);

		// Rack power
		//sprintf(str, %3d.%dC",t.inCelsus, t.frac);
		Display_Info(Current, 5, 5, Font_16x26);
		if(PWR_CTRL.Rack_Current < 2.0){
			sprintf(str, "Rack off");
		}else{
			sprintf(str, "%dA", (int)PWR_CTRL.Rack_Current);
		}
		Display_Info(str, 5, 30, Font_11x18);
		SSD1306_UpdateScreen();

		// Ожидаем и после затираем экран
		HAL_Delay(3000);
		SSD1306_Fill(SSD1306_COLOR_BLACK);

		// Отображаем результаты измерений тока заряда
		Display_Info(Charge, 5, 5, Font_16x26);
		if(PWR_CTRL.Charge_Current < 2.0){
			sprintf(str, "OFF");
		}else{
			sprintf(str, "%dA", (int)PWR_CTRL.Charge_Current);
		}
		Display_Info(str, 5, 30, Font_16x26);
		SSD1306_UpdateScreen();

		HAL_Delay(3000);
		// Стираем результаты измерений
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_UpdateScreen();
		SSD1306_OFF();
//		Disable_5V();
		HAL_Delay(5000);
	}

}

void Beep_Beep(void)
{
	Beep_On();
}

void Sleep_Time(void)
{
	//HAL_PWR_DisableSleepOnExit(); // очистить
	HAL_SuspendTick(); //HAL_ResumeTick();
	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

//-----------------------------------------------------------------------------
//                               DS18B20
//-----------------------------------------------------------------------------

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xffff);
	return ch;
}

void get_ROMid (void){
	if (owResetCmd() != ONEWIRE_NOBODY) {    // is anybody on the bus?
		devices = owSearchCmd(&ow);        // получить ROMid всех устройст на шине или вернуть код ошибки
		if (devices <= 0) {
			printf("\n\rError has happened!");
			while (1){
				int pDelay = 1000000;
				for (int i = 0; i < pDelay * 1; i++){}
			}

		}
		printf("\n\rfound %d devices on 1-wire bus", devices);


		for (int i = 0; i < devices; i++) {//выводим в консоль все найденные ROM
			RomCode *r = &ow.ids[i];
			uint8_t crc = owCRC8(r);
			crcOK = (crc == r->crc)?"CRC OK":"CRC ERROR!";
			printf("\n\rdevice %d (SN: %02X/%02X%02X%02X%02X%02X%02X/%02X) ", i, r->family, r->code[5], r->code[4], r->code[3],
					r->code[2], r->code[1], r->code[0], r->crc);
			printf("crcOK");
			if (crc != r->crc) {
				printf("\n\r can't read cause CNC error");
			}
		}
	}
	int pDelay = 1000000;
	for (int i = 0; i < pDelay * 1; i++){} asm("nop");
}

int16_t Get_Temperature(void)
{
	int16_t Temperature = 0;


	for ( int i = 0 ; i < devices; i++) {
		//Temperature t;
		switch ((ow.ids[i]).family) {//че у нас за датчик
		case DS18B20:
			// будет возвращено значение предыдущего измерения!
			t = readTemperature(&ow, &ow.ids[i], 1);
			printf("\n\rDS18B20 N_%d , Temp: %3d.%dC",i,t.inCelsus, t.frac);
			Temperature += t.inCelsus + t.frac/10;
			break;
		case DS18S20:
			t = readTemperature(&ow, &ow.ids[i], 1);
			printf("\n\rDS18S20 , Temp: %3d.%dC", t.inCelsus, t.frac);
			Temperature += t.inCelsus + t.frac/10;
			break;
			//		case 0x00:
			//			break;
		default:
			printf("\n\rUNKNOWN Family:%x (SN: %x%x%x%x%x%x)", (ow.ids[i]).family, (ow.ids[i]).code[0],(ow.ids[i]).code[1],(ow.ids[i]).code[2],
					(ow.ids[i]).code[3], (ow.ids[i]).code[4], (ow.ids[i]).code[5]);
			break;
		}
	}
	static int counter;
	printf("\n\r%d",counter++);
	int pDelay = 16000000;
	for (int i = 0; i < pDelay * 1; i++) {}//asm("nop");
	if(devices != 0){
		Temperature = Temperature/devices;
	}
	return Temperature;
}

//-----------------------------------------------------------------------------
//                               CALLBACK
//-----------------------------------------------------------------------------

//------------------------------------------------------------------------------
// обработка прерываний АЦП
//------------------------------------------------------------------------------

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	(void)hadc;
	//HAL_ResumeTick();
	double Charge, Rack;

//	HAL_ADC_Stop_DMA(&hadc1);

	if (PWR_CTRL.First_Time == true)
	{
		double temp = (CURRENT_RDIV_VALUE * BAT_ADC_VREF * adc.Charge_Current)/4096.0;
		memcpy(&PWR_CTRL.Rack_VREF, &temp, sizeof(double));
		temp = (CURRENT_RDIV_VALUE * BAT_ADC_VREF * adc.Rack_Current)/4096.0;
		memcpy(&PWR_CTRL.Charge_VREF, &temp, sizeof(double));
//		PWR_CTRL.Rack_VREF    = (CURRENT_RDIV_VALUE * BAT_ADC_VREF * adc.Charge_Current)/4096.0;
//		PWR_CTRL.Charge_VREF  = (CURRENT_RDIV_VALUE * BAT_ADC_VREF * adc.Rack_Current)/4096.0;

		PWR_CTRL.First_Time = false;
	}

	PWR_CTRL.Prev_Voltage_Pb  = PWR_CTRL.Voltage_Pb;
	PWR_CTRL.Voltage_Li_Ion   = ((LI_RDIV_VALUE * BAT_ADC_VREF) * adc.Li_Ion_Voltage/4096.0);
	PWR_CTRL.Voltage_Pb       = ((PB_RDIV_VALUE * BAT_ADC_VREF * adc.Pb_Voltage)/4096.0);
	Rack                      = (CURRENT_RDIV_VALUE * BAT_ADC_VREF * adc.Charge_Current)/4096.0;
	PWR_CTRL.Rack_Current     = (fabs((Rack - PWR_CTRL.Rack_VREF) * 50));
	Charge                    = (CURRENT_RDIV_VALUE * BAT_ADC_VREF * adc.Rack_Current)/4096.0;
	PWR_CTRL.Charge_Current   = (fabs((Charge - PWR_CTRL.Charge_VREF) * 50));

	PWR_CTRL.ADC_busy = false;

}

//------------------------------------------------------------------------------
// обработка прерывания от Кнопки
//------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
//		uint8_t TM = 0;

	if (GPIO_PIN == WAKE_UP_Pin)
	{
		BTN_Event_Fl = true;

//				BTN_Event();
//		HAL_ResumeTick();
//				while ((HAL_GPIO_ReadPin(WAKE_UP_GPIO_Port, WAKE_UP_Pin) == GPIO_PIN_RESET) &&
//						(TM < 200))
//				{
//					TM++;
//					if (TM >= 150) BTN_Event();
//				}
	}
}

//------------------------------------------------------------------------------
// обработка прерывания от Таймера
//------------------------------------------------------------------------------

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
	{

	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
