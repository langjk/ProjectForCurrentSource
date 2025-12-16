/* oled_hal.h */
#ifndef __OLED_HAL_H
#define __OLED_HAL_H

#include "stm32f1xx_hal.h"  // 根据所用 MCU 修改
#include <stdint.h>

#include "OLED_Data.h"  // 加入字模数据头文件

#define OLED_6X8   6
#define OLED_8X16  8

#define OLED_UNFILLED 0
#define OLED_FILLED   1

extern I2C_HandleTypeDef hi2c1;  // HAL I2C句柄（确保定义于 main.c 或其他文件中）

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Update(void);
void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);

void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize);
void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize);
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize);

#endif /* __OLED_HAL_H */
