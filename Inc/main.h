/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdint.h"
#include "stdbool.h"
#include <stdio.h>
#include "LCD_H.h"
#include "ssd1306.h"
#include "fonts.h"

#include "OneWire.h"


#include "tim.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern int TEMPVALUE;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void get_ROMid (void);
void Display_Info(char* str, uint8_t x, uint8_t y, FontDef_t font);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VOLTAGE_LI_ION_Pin GPIO_PIN_0
#define VOLTAGE_LI_ION_GPIO_Port GPIOA
#define VOLTAGE_PB_Pin GPIO_PIN_1
#define VOLTAGE_PB_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_2
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_3
#define DEBUG_RX_GPIO_Port GPIOA
#define RACK_CURRENT_Pin GPIO_PIN_4
#define RACK_CURRENT_GPIO_Port GPIOA
#define CHARGE_CURRENT_Pin GPIO_PIN_5
#define CHARGE_CURRENT_GPIO_Port GPIOA
#define IGNITION_Pin GPIO_PIN_6
#define IGNITION_GPIO_Port GPIOA
#define EN_5V_Pin GPIO_PIN_0
#define EN_5V_GPIO_Port GPIOB
#define EN_12V_Pin GPIO_PIN_1
#define EN_12V_GPIO_Port GPIOB
#define EN_FAN_Pin GPIO_PIN_2
#define EN_FAN_GPIO_Port GPIOB
#define TX_1_WIRE_Pin GPIO_PIN_10
#define TX_1_WIRE_GPIO_Port GPIOB
#define RX_1_WIRE_Pin GPIO_PIN_11
#define RX_1_WIRE_GPIO_Port GPIOB
#define WAKE_UP_Pin GPIO_PIN_12
#define WAKE_UP_GPIO_Port GPIOA
#define WAKE_UP_EXTI_IRQn EXTI15_10_IRQn
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOA
#define PWR_TO_RACK_ON_OFF_Pin GPIO_PIN_3
#define PWR_TO_RACK_ON_OFF_GPIO_Port GPIOB
#define LI_ION_ON_OFF_Pin GPIO_PIN_4
#define LI_ION_ON_OFF_GPIO_Port GPIOB
#define ON_LED_DISP_Pin GPIO_PIN_5
#define ON_LED_DISP_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_8
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_9
#define I2C_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

//#define VOLTAGE_LI_ION_Pin GPIO_PIN_0
//#define VOLTAGE_LI_ION_GPIO_Port GPIOA
//#define VOLTAGE_PB_Pin GPIO_PIN_1
//#define VOLTAGE_PB_GPIO_Port GPIOA
//#define DEBUG_TX_Pin GPIO_PIN_2
//#define DEBUG_TX_GPIO_Port GPIOA
//#define DEBUG_RX_Pin GPIO_PIN_3
//#define DEBUG_RX_GPIO_Port GPIOA
//#define RACK_CURRENT_Pin GPIO_PIN_4
//#define RACK_CURRENT_GPIO_Port GPIOA
//#define CHARGE_CURRENT_Pin GPIO_PIN_5
//#define CHARGE_CURRENT_GPIO_Port GPIOA
//#define IGNITION_Pin GPIO_PIN_6
//#define IGNITION_GPIO_Port GPIOA
//#define EN_5V_Pin GPIO_PIN_0
//#define EN_5V_GPIO_Port GPIOB
//#define EN_12V_Pin GPIO_PIN_1
//#define EN_12V_GPIO_Port GPIOB
//#define EN_FAN_Pin GPIO_PIN_2
//#define EN_FAN_GPIO_Port GPIOB
//#define TX_1_WIRE_Pin GPIO_PIN_10
//#define TX_1_WIRE_GPIO_Port GPIOB
//#define RX_1_WIRE_Pin GPIO_PIN_11
//#define RX_1_WIRE_GPIO_Port GPIOB
//#define WAKE_UP_Pin GPIO_PIN_12
//#define WAKE_UP_GPIO_Port GPIOA
//
//#define PWR_TO_RACK_ON_OFF_Pin GPIO_PIN_3
//#define PWR_TO_RACK_ON_OFF_GPIO_Port GPIOB
//#define LI_ION_ON_OFF_Pin GPIO_PIN_4
//#define LI_ION_ON_OFF_GPIO_Port GPIOB
//#define ON_LED_DISP_Pin GPIO_PIN_5
//#define ON_LED_DISP_GPIO_Port GPIOB
//#define I2C_SCL_Pin GPIO_PIN_8
//#define I2C_SCL_GPIO_Port GPIOB
//#define I2C_SDA_Pin GPIO_PIN_9
//#define I2C_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


//------------------------------------------
// ADC
//------------------------------------------

#define VOLTAGE_LI_ION_Pin        GPIO_PIN_0
#define VOLTAGE_LI_ION_GPIO_Port  GPIOA
#define VOLTAGE_PB_Pin            GPIO_PIN_1
#define VOLTAGE_PB_GPIO_Port      GPIOA

#define RACK_CURRENT_Pin          GPIO_PIN_4
#define RACK_CURRENT_GPIO_Port    GPIOA
#define CHARGE_CURRENT_Pin        GPIO_PIN_5
#define CHARGE_CURRENT_GPIO_Port  GPIOA
//------------------------------------------


//------------------------------------------
// POWER CTRL
//------------------------------------------

#define IGNITION_Pin              GPIO_PIN_6
#define IGNITION_GPIO_Port        GPIOA
//#define EN_5V_Pin                 GPIO_PIN_0
//#define EN_5V_GPIO_Port           GPIOB
#define EN_12V_Pin                GPIO_PIN_1
#define EN_12V_GPIO_Port          GPIOB
#define EN_FAN_Pin                GPIO_PIN_2
#define EN_FAN_GPIO_Port          GPIOB

#define LI_ION_ON_OFF_Pin         GPIO_PIN_4
#define LI_ION_ON_OFF_GPIO_Port   GPIOB
#define RACK_ON_OFF_Pin           GPIO_PIN_3
#define RACK_ON_OFF_GPIO_Port     GPIOB

//#define Beep_Off()        HAL_GPIO_WritePin(BUZZER_GPIO_Port,         BUZZER_Pin,         GPIO_PIN_RESET)
//#define Beep_On()         HAL_GPIO_WritePin(BUZZER_GPIO_Port,         BUZZER_Pin,         GPIO_PIN_SET)

//------------------------------------------

//------------------------------------------
// UART and 1-WIRE
//------------------------------------------

#define DEBUG_TX_Pin              GPIO_PIN_2
#define DEBUG_TX_GPIO_Port        GPIOA
#define DEBUG_RX_Pin              GPIO_PIN_3
#define DEBUG_RX_GPIO_Port        GPIOA
#define TX_1_WIRE_Pin             GPIO_PIN_10
#define TX_1_WIRE_GPIO_Port       GPIOB

#define OW_USART                USART3
#define MAXDEVICES_ON_THE_BUS   5

#define ONEWIRE_OUT   TX_1_WIRE_Pin
#define ONEWIRE_IN    RX_1_WIRE_Pin
#define ONEWIRE_PORT 	GPIOB
//------------------------------------------

//------------------------------------------
// LCD_DISPLAY
//------------------------------------------

#define DISPLAY_DATA_7_Pin        GPIO_PIN_14
#define DISPLAY_DATA_7_GPIO_Port  GPIOB
#define DISPLAY_DATA_6_Pin        GPIO_PIN_15
#define DISPLAY_DATA_6_GPIO_Port  GPIOB
#define DISPLAY_DATA_5_Pin        GPIO_PIN_8
#define DISPLAY_DATA_5_GPIO_Port  GPIOA
#define DISPLAY_DATA_4_Pin        GPIO_PIN_9
#define DISPLAY_DATA_4_GPIO_Port  GPIOA
#define DISPLAY_RESET_Pin         GPIO_PIN_10
#define DISPLAY_RESET_GPIO_Port   GPIOA
#define DISPLAY_ENABLE_Pin        GPIO_PIN_11
#define DISPLAY_ENABLE_GPIO_Port  GPIOA
//------------------------------------------

//------------------------------------------
// WAKE-UP and SLEEP Mode
//------------------------------------------

#define WAKE_UP_Pin               GPIO_PIN_12
#define WAKE_UP_GPIO_Port         GPIOA

//------------------------------------------
// I2C
//------------------------------------------

#define I2C_SCL_Pin               GPIO_PIN_8
#define I2C_SCL_GPIO_Port         GPIOB
#define I2C_SDA_Pin               GPIO_PIN_9
#define I2C_SDA_GPIO_Port         GPIOB
//------------------------------------------

 //------------------------------------------
// POWER
//------------------------------------------

#define R1_DIV_LI 1000 		//kOhm
#define R2_DIV_LI 120  		//kOhm
#define R1_DIV_PB 1000 		//kOhm
#define R2_DIV_PB 287  		//kOhm
#define R1_DIV_CURRENT 10 	//kOhm
#define R2_DIV_CURRENT 3.6 	//kOhm

//#define LI_RDIV_VALUE       (R1_DIV_LI + R2_DIV_LI)/R2_DIV_LI
#define LI_RDIV_VALUE       11.18305
//#define PB_RDIV_VALUE       (R1_DIV_PB + R2_DIV_PB)/R2_DIV_PB
#define PB_RDIV_VALUE       5.044
#define CURRENT_RDIV_VALUE  (R1_DIV_CURRENT + R2_DIV_CURRENT)/R2_DIV_CURRENT
#define BAT_ADC_VREF        3.35 //V

#define CURRENT_CHARGE_VREF 6.24 //V
#define CURRENT_RACK_VREF   6.4 //V

#define GENERATOR_VOLTAGE   13.0 //V
#define LI_ION_MAX_VOLTAGE  30.0 //V
#define LI_ION_MIN_VOLTAGE  21.0 //V

#define MAX_CHARGE_CURRENT  120.0 //A
#define MAX_RACK_CURRENT    40.0 //A

#define MAX_CHARGE_TEMP     40.0 //C
#define MIN_CHARGE_TEMP     5.0  //C
#define MAX_WORK_TEMP       45.0 //C
#define MIN_WORK_TEMP       5.0  //C

typedef struct
{
  double Voltage_Li_Ion;	//Напряжение на литие
  double Voltage_Pb;		//Напряжение на свинце
  double Prev_Voltage_Pb;
  double Charge_Current;	//Ток заряда
  double Rack_Current;		//Весь ток через стойку

  const double Charge_VREF;	//Напряжение "покоя" на датчике тока заряда
  const double Rack_VREF;	//Напряжение "покоя" на датчике тока стойки

  bool Charger_On;			//Заряд включен
  bool Ingition_On;
  bool Li_Ion_On;			//Литий влкючен
  bool Rack_On;				//Стойка включена
  bool Five_V_On;			//Напряжение на дисплей включено
  bool Twelve_V_On;			//Напряжение на датчики тока, ацп
  bool Fan_On;				//Включение вентилятора

  bool ADC_busy;
  bool First_Time;

  bool Beep_On;
} Pwr_ctrl_t;

Pwr_ctrl_t PWR_CTRL;


//------------------------------------------

#define OLED_DISPLAY


//Power Control

static inline void Disable_Rack(void){
	PWR_CTRL.Rack_On = false;
	HAL_GPIO_WritePin(RACK_ON_OFF_GPIO_Port,    RACK_ON_OFF_Pin,    GPIO_PIN_RESET);
}
static inline void Enable_Rack(void){
	PWR_CTRL.Rack_On = true;
	HAL_GPIO_WritePin(RACK_ON_OFF_GPIO_Port,    RACK_ON_OFF_Pin,    GPIO_PIN_SET);
}

static inline void Ignition_On(void){
	PWR_CTRL.Ingition_On = true;
	HAL_GPIO_WritePin(IGNITION_GPIO_Port,       IGNITION_Pin,       GPIO_PIN_SET);
}
static inline void Ignition_Off(void){
	PWR_CTRL.Ingition_On = false;
	HAL_GPIO_WritePin(IGNITION_GPIO_Port,       IGNITION_Pin,       GPIO_PIN_RESET);
}

static inline void Enable_5V(void){
	PWR_CTRL.Five_V_On = true;
	HAL_GPIO_WritePin(EN_5V_GPIO_Port,          EN_5V_Pin,          GPIO_PIN_SET);
}
static inline void Disable_5V(void){
	PWR_CTRL.Five_V_On = false;
	HAL_GPIO_WritePin(EN_5V_GPIO_Port,          EN_5V_Pin,          GPIO_PIN_RESET);
}

static inline void Enable_12V(void){
	PWR_CTRL.Twelve_V_On = true;
	HAL_GPIO_WritePin(EN_12V_GPIO_Port,         EN_12V_Pin,         GPIO_PIN_SET);
}
static inline void Disable_12V(void){
	PWR_CTRL.Twelve_V_On = false;
	HAL_GPIO_WritePin(EN_12V_GPIO_Port,         EN_12V_Pin,         GPIO_PIN_RESET);
}

static inline void Enable_Fan(void){
	PWR_CTRL.Fan_On = true;
	HAL_GPIO_WritePin(EN_FAN_GPIO_Port,         EN_FAN_Pin,         GPIO_PIN_SET);
}
static inline void Disable_Fan(void){
	PWR_CTRL.Fan_On = false;
	HAL_GPIO_WritePin(EN_FAN_GPIO_Port,         EN_FAN_Pin,         GPIO_PIN_RESET);
}

static inline void Enable_Li_Ion(void){
	PWR_CTRL.Li_Ion_On = true;
	HAL_GPIO_WritePin(LI_ION_ON_OFF_GPIO_Port,  LI_ION_ON_OFF_Pin,  GPIO_PIN_SET);
}
static inline void Disable_Li_Ion(void){
	PWR_CTRL.Li_Ion_On = false;
	HAL_GPIO_WritePin(LI_ION_ON_OFF_GPIO_Port,  LI_ION_ON_OFF_Pin,  GPIO_PIN_RESET);
}

//Timer

static inline void Start_timer(TIM_HandleTypeDef *htim)
{
HAL_TIM_Base_Start(htim);
}
static inline void Stop_timer(TIM_HandleTypeDef *htim)
{
HAL_TIM_Base_Stop(htim);
htim->Instance->CNT = 0;
}
static inline double CNTtoSeconds(TIM_HandleTypeDef *htim)
{
return htim->Instance-> CNT * 65535 / 32000000.0;
}

//is X ok?
extern TIM_HandleTypeDef htim6;
static inline bool is_Rack_OK(void){
	if(PWR_CTRL.Rack_Current > MAX_RACK_CURRENT)
	{
		Start_timer(&htim6);
		if(CNTtoSeconds(&htim6) > 10)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		Stop_timer(&htim6);
		return false;
	}
}

static inline bool is_Charge_OK(void){
	if ((PWR_CTRL.Charge_Current < MAX_CHARGE_CURRENT) &&
		(PWR_CTRL.Voltage_Pb > GENERATOR_VOLTAGE) 	   &&
		((TEMPVALUE >= 5) || (TEMPVALUE < 40)))
	{
		return true;
	}
	else
	{
		return false;
	}
}

static inline bool is_Charge_Alarm(void){
	if(PWR_CTRL.Charge_Current > MAX_CHARGE_CURRENT)
	{
		return true;
	}
	else
	{
		return false;
	}
}

static inline bool is_Li_Ion_OK(void){
	if (((PWR_CTRL.Voltage_Li_Ion) > LI_ION_MIN_VOLTAGE) && //емкость
		((TEMPVALUE >= 5) && (TEMPVALUE < 40))           && //температура
		(PWR_CTRL.Voltage_Pb < GENERATOR_VOLTAGE))          //отсутствие генератора
	{
		return true;
	}
	else
	{
		return false;
	}
}


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
