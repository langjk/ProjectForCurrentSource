/* oled_hal.c */
#include "oled.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#define OLED_I2C_ADDR 0x78

uint8_t OLED_DisplayBuf[8][128];

void OLED_WriteCommand(uint8_t Command)
{
    uint8_t data[2] = {0x00, Command};
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

void OLED_WriteData(uint8_t *Data, uint8_t Count)
{
    uint8_t buf[129];
    if (Count > 128) {
        Count = 128;
    }
    buf[0] = 0x40;
    memcpy(&buf[1], Data, Count);
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, buf, Count + 1, HAL_MAX_DELAY);
}

void OLED_SetCursor(uint8_t Page, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Page);
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (X & 0x0F));
}

void OLED_Update(void)
{
    for (uint8_t j = 0; j < 8; j++) {
        OLED_SetCursor(j, 0);
        OLED_WriteData(OLED_DisplayBuf[j], 128);
    }
}

void OLED_Clear(void)
{
    for (uint8_t j = 0; j < 8; j++) {
        memset(OLED_DisplayBuf[j], 0x00, 128);
    }
}

void OLED_Init(void)
{
    HAL_Delay(100);
    OLED_WriteCommand(0xAE);
    OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8);
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81);
    OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9);
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB);
    OLED_WriteCommand(0x30);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);
    OLED_Clear();
    OLED_Update();
}

static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
    const uint8_t *font = NULL;
    uint8_t width = 0, height = 0;
    if (FontSize == OLED_6X8) {
        font = OLED_F6x8[Char - ' '];
        width = 6; height = 8;
    } else if (FontSize == OLED_8X16) {
        font = OLED_F8x16[Char - ' '];
        width = 8; height = 16;
    }
    if (!font) return;

    for (uint8_t i = 0; i < width; i++) {
        for (uint8_t j = 0; j < height; j++) {
            uint8_t page = (Y + j) / 8;
            uint8_t bit_pos = (Y + j) % 8;
            uint8_t pixel = (font[i + (j / 8) * width] >> (j % 8)) & 0x01;

            if (pixel) {
                OLED_DisplayBuf[page][X + i] |= (1 << bit_pos);
            } else {
                OLED_DisplayBuf[page][X + i] &= ~(1 << bit_pos);
            }
        }
    }
}

void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{
    for (uint8_t i = 0; String[i] != '\0'; i++) {
        OLED_ShowChar(X + i * FontSize, Y, String[i], FontSize);
    }
}

void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    for (uint8_t i = 0; i < Length; i++)
    {
        OLED_ShowChar(X + i * FontSize, Y, Number / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
    }
}

void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint32_t Number1;
    if (Number >= 0) {
        OLED_ShowChar(X, Y, '+', FontSize);
        Number1 = Number;
    } else {
        OLED_ShowChar(X, Y, '-', FontSize);
        Number1 = -Number;
    }
    OLED_ShowNum(X + FontSize, Y, Number1, Length, FontSize);
}

void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t SingleNumber;
    for (uint8_t i = 0; i < Length; i++) {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        OLED_ShowChar(X + i * FontSize, Y, SingleNumber < 10 ? (SingleNumber + '0') : (SingleNumber - 10 + 'A'), FontSize);
    }
}

void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    for (uint8_t i = 0; i < Length; i++) {
        OLED_ShowChar(X + i * FontSize, Y, Number / OLED_Pow(2, Length - i - 1) % 2 + '0', FontSize);
    }
}

void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize)
{
    uint32_t PowNum, IntNum, FraNum;

    if (Number >= 0) {
        OLED_ShowChar(X, Y, '+', FontSize);
    } else {
        OLED_ShowChar(X, Y, '-', FontSize);
        Number = -Number;
    }

    IntNum = Number;
    Number -= IntNum;
    PowNum = OLED_Pow(10, FraLength);
    FraNum = round(Number * PowNum);
    IntNum += FraNum / PowNum;

    OLED_ShowNum(X + FontSize, Y, IntNum, IntLength, FontSize);
    OLED_ShowChar(X + (IntLength + 1) * FontSize, Y, '.', FontSize);
    OLED_ShowNum(X + (IntLength + 2) * FontSize, Y, FraNum, FraLength, FontSize);
}
void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    for (int16_t j = Y; j < Y + Height; j++) {
        for (int16_t i = X; i < X + Width; i++) {
            if (i >= 0 && i < 128 && j >= 0 && j < 64) {
                OLED_DisplayBuf[j / 8][i] ^= 0x01 << (j % 8);
            }
        }
    }
}
