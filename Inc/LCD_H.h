#ifndef LCD_H_
#define LCD_H_

#include "main.h"

#define d4_set()    HAL_GPIO_WritePin(DISPLAY_DATA_4_GPIO_Port, DISPLAY_DATA_4_Pin, GPIO_PIN_SET)
#define d5_set()    HAL_GPIO_WritePin(DISPLAY_DATA_5_GPIO_Port, DISPLAY_DATA_5_Pin, GPIO_PIN_SET)
#define d6_set()    HAL_GPIO_WritePin(DISPLAY_DATA_6_GPIO_Port, DISPLAY_DATA_6_Pin, GPIO_PIN_SET)
#define d7_set()    HAL_GPIO_WritePin(DISPLAY_DATA_7_GPIO_Port, DISPLAY_DATA_7_Pin, GPIO_PIN_SET)
#define d4_reset()  HAL_GPIO_WritePin(DISPLAY_DATA_4_GPIO_Port, DISPLAY_DATA_4_Pin, GPIO_PIN_RESET)
#define d5_reset()  HAL_GPIO_WritePin(DISPLAY_DATA_5_GPIO_Port, DISPLAY_DATA_5_Pin, GPIO_PIN_RESET)
#define d6_reset()  HAL_GPIO_WritePin(DISPLAY_DATA_6_GPIO_Port, DISPLAY_DATA_6_Pin, GPIO_PIN_RESET)
#define d7_reset()  HAL_GPIO_WritePin(DISPLAY_DATA_7_GPIO_Port, DISPLAY_DATA_7_Pin, GPIO_PIN_RESET)

#define e1    HAL_GPIO_WritePin(DISPLAY_ENABLE_GPIO_Port, DISPLAY_ENABLE_Pin, GPIO_PIN_SET)    // установка линии E в 1
#define e0    HAL_GPIO_WritePin(DISPLAY_ENABLE_GPIO_Port, DISPLAY_ENABLE_Pin, GPIO_PIN_RESET)  // установка линии E в 0
#define rs1   HAL_GPIO_WritePin(DISPLAY_RESET_GPIO_Port, DISPLAY_RESET_Pin, GPIO_PIN_SET)    // установка линии RS в 1 (данные)
#define rs0   HAL_GPIO_WritePin(DISPLAY_RESET_GPIO_Port, DISPLAY_RESET_Pin, GPIO_PIN_RESET)  // установка линии RS в 0 (команда)

void LCD_Init(void);
void LCD_Clear(void);
void LCD_SendChar(char ch);
void LCD_String(char* st);
void LCD_SetPos(uint8_t x, uint8_t y);

#endif /* LCD_H_ */