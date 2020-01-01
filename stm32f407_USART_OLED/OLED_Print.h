#ifndef OLED_PRINT_H_
#define OLED_PRINT_H_

#include "stm32f4xx.h"
#include "./ssd1306/ssd1306.h"

#define OLED_PRINT_CHINESE_HIGHT				24
#define OLED_PRINT_CHINESE_WIDTH				24
#define OLED_PRINT_ASCII_HIGHT					24
#define OLED_PRINT_ASCII_WIDTH					12

#define OLED_Print_Function(x, y, pixel)		ssd1306_DrawPixel(x, y, pixel)

static const uint8_t DigitPrintData[10][OLED_PRINT_ASCII_HIGHT/8][OLED_PRINT_ASCII_WIDTH] = {
{
{0x00,0xC0,0xF0,0x38,0x0C,0x04,0x0C,0x38,0xF0,0xC0,0x00,0x00},
{0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00},
{0x00,0x03,0x0F,0x1C,0x30,0x20,0x30,0x1C,0x0F,0x03,0x00,0x00}},	// "0"
{
{0x00,0x10,0x10,0x18,0x18,0xFC,0xFC,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x20,0x20,0x20,0x3F,0x3F,0x20,0x20,0x20,0x00,0x00}},	// "1"
{
{0x60,0xF0,0xF8,0x0C,0x04,0x04,0x04,0x0C,0x18,0xF0,0xE0,0x00},
{0x00,0x00,0x00,0x80,0xC0,0x60,0x30,0x18,0x0E,0x07,0x01,0x00},
{0x38,0x3E,0x3F,0x33,0x30,0x30,0x30,0x30,0x30,0x30,0x3C,0x00}},	// "2"
{
{0x00,0x30,0x78,0x78,0x0C,0x04,0x04,0x0C,0x18,0xF8,0xF0,0x00},
{0x00,0x00,0x00,0x00,0x08,0x08,0x1C,0x16,0x33,0xE1,0xC0,0x00},
{0x0C,0x1E,0x3E,0x30,0x20,0x20,0x20,0x30,0x18,0x1F,0x0F,0x00}},	// "3"
{
{0x00,0x00,0x00,0x00,0x80,0xE0,0x38,0xFC,0xFC,0x00,0x00,0x00},
{0x80,0xE0,0x38,0x0E,0x03,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00},
{0x01,0x01,0x01,0x01,0x01,0x21,0x21,0x3F,0x3F,0x21,0x21,0x01}},	// "4"
{
{0x00,0xFC,0xFC,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x00,0x00},
{0x00,0x1F,0x0F,0x08,0x04,0x02,0x02,0x06,0x1C,0xF8,0xE0,0x00},
{0x0C,0x1E,0x3E,0x30,0x20,0x20,0x20,0x30,0x1C,0x0F,0x03,0x00}},	// "5"
{
{0xC0,0xF0,0x38,0x0C,0x04,0x04,0x0C,0x7C,0x78,0x30,0x00,0x00},
{0xFF,0xFF,0x08,0x04,0x02,0x02,0x02,0x06,0x1C,0xF8,0xE0,0x00},
{0x03,0x0F,0x1C,0x30,0x20,0x20,0x20,0x30,0x1C,0x0F,0x03,0x00}},	// "6"
{
{0x78,0x1C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0xCC,0x7C,0x1C,0x00},
{0x00,0x00,0x00,0x00,0x80,0xF0,0x7C,0x0F,0x01,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x3F,0x3F,0x00,0x00,0x00,0x00,0x00,0x00}},	// "7"
{
{0x00,0xF0,0xF8,0x0C,0x04,0x04,0x04,0x0C,0xF8,0xF0,0x00,0x00},
{0xC0,0xE0,0x31,0x1B,0x0E,0x0C,0x0E,0x1B,0x31,0xE0,0xC0,0x00},
{0x07,0x0F,0x18,0x30,0x20,0x20,0x20,0x30,0x18,0x0F,0x07,0x00}},	// "8"
{
{0xC0,0xF0,0x38,0x0C,0x04,0x04,0x04,0x0C,0x38,0xF0,0xC0,0x00},
{0x07,0x1F,0x38,0x60,0x40,0x40,0x40,0x20,0x10,0xFF,0xFF,0x00},
{0x00,0x0C,0x1E,0x3E,0x30,0x20,0x20,0x30,0x1C,0x0F,0x03,0x00}}	// "9"
};

static const uint8_t UpperPrintData[26][OLED_PRINT_ASCII_HIGHT/8][OLED_PRINT_ASCII_WIDTH] = {
{
{0x00,0x00,0x00,0x00,0xF0,0x0C,0x3C,0xF0,0x00,0x00,0x00,0x00},
{0x00,0x00,0xF0,0xFF,0x43,0x40,0x40,0x43,0xFF,0xF0,0x00,0x00},
{0x20,0x3E,0x3F,0x20,0x00,0x00,0x00,0x00,0x20,0x3F,0x3E,0x20}},	// "A"
{
{0x04,0xFC,0xFC,0x04,0x04,0x04,0x0C,0x18,0xF0,0xE0,0x00,0x00},
{0x00,0xFF,0xFF,0x08,0x08,0x08,0x1C,0x16,0x73,0xE1,0x80,0x00},
{0x20,0x3F,0x3F,0x20,0x20,0x20,0x20,0x30,0x1C,0x0F,0x03,0x00}},	// "B"
{
{0xC0,0xF0,0x38,0x0C,0x04,0x04,0x04,0x0C,0x18,0x38,0x7C,0x00},
{0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x03,0x0F,0x1C,0x30,0x20,0x20,0x20,0x30,0x18,0x0E,0x06,0x00}},	// "C"
{
{0x04,0xFC,0xFC,0x04,0x04,0x04,0x0C,0x18,0x70,0xE0,0x80,0x00},
{0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00},
{0x20,0x3F,0x3F,0x20,0x20,0x20,0x30,0x18,0x0E,0x07,0x01,0x00}},	// "D"
{
{0x04,0xFC,0xFC,0x04,0x04,0x04,0x04,0x04,0x0C,0x1C,0x7C,0x00},
{0x00,0xFF,0xFF,0x08,0x08,0x08,0x1C,0x7F,0x00,0x00,0x00,0x00},
{0x20,0x3F,0x3F,0x20,0x20,0x20,0x20,0x20,0x30,0x38,0x3E,0x00}},	// "E"
{
{0x04,0xFC,0xFC,0x04,0x04,0x04,0x04,0x04,0x0C,0x1C,0x7C,0x00},
{0x00,0xFF,0xFF,0x08,0x08,0x08,0x1C,0x7F,0x00,0x00,0x00,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},	// "F"
{
{0xC0,0xF0,0x38,0x0C,0x04,0x04,0x0C,0x08,0x18,0x7C,0x00,0x00},
{0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x08,0xF8,0xF8,0x08,0x00},
{0x03,0x0F,0x1C,0x30,0x20,0x30,0x18,0x0C,0x3F,0x3F,0x00,0x00}},	// "G"
{
{0x04,0xFC,0xFC,0x04,0x00,0x00,0x00,0x04,0xFC,0xFC,0x04,0x00},
{0x00,0xFF,0xFF,0x08,0x08,0x08,0x08,0x08,0xFF,0xFF,0x00,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x00,0x00,0x20,0x3F,0x3F,0x20,0x00}},	// "H"
{
{0x00,0x00,0x00,0x04,0x04,0x04,0xFC,0xFC,0x04,0x04,0x04,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x20,0x20,0x20,0x3F,0x3F,0x20,0x20,0x20,0x00}},	// "I"
{
{0x00,0x00,0x00,0x00,0x00,0x04,0x04,0xFC,0xFC,0x04,0x04,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00},
{0x0E,0x1F,0x37,0x22,0x20,0x30,0x30,0x1F,0x0F,0x00,0x00,0x00}},	// "J"
{
{0x04,0xFC,0xFC,0x04,0x00,0x80,0xE4,0x7C,0x1C,0x04,0x04,0x00},
{0x00,0xFF,0xFF,0x78,0x1E,0x7F,0xE1,0x80,0x00,0x00,0x00,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x00,0x01,0x27,0x3E,0x38,0x20,0x20}},	// "K"
{
{0x04,0xFC,0xFC,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x20,0x3F,0x3F,0x20,0x20,0x20,0x20,0x20,0x30,0x38,0x3E,0x00}},	// "L"
{
{0x04,0xFC,0xFC,0xC0,0x00,0x00,0x00,0x00,0xC0,0xFC,0xFC,0x04},
{0x00,0xFF,0xFF,0x07,0x3C,0xE0,0xE0,0x3C,0x07,0xFF,0xFF,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x01,0x01,0x00,0x20,0x3F,0x3F,0x20}},	// "M"
{
{0x04,0xFC,0xFC,0xE0,0x00,0x00,0x00,0x04,0xFC,0xFC,0x04,0x00},
{0x00,0xFF,0xFF,0x01,0x07,0x1C,0x70,0xC0,0xFF,0xFF,0x00,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x00,0x00,0x03,0x0F,0x3F,0x00,0x00}},	// "N"
{
{0xE0,0xF0,0x18,0x0C,0x04,0x04,0x04,0x0C,0x18,0xF0,0xE0,0x00},
{0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00},
{0x07,0x0F,0x18,0x30,0x20,0x20,0x20,0x30,0x18,0x0F,0x07,0x00}},	// "O"
{
{0x04,0xFC,0xFC,0x04,0x04,0x04,0x0C,0x08,0x38,0xF0,0xC0,0x00},
{0x00,0xFF,0xFF,0x10,0x10,0x10,0x18,0x08,0x0E,0x07,0x01,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},	// "P"
{
{0xE0,0xF0,0x18,0x0C,0x04,0x04,0x04,0x0C,0x18,0xF0,0xE0,0x00},
{0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00},
{0x07,0x0F,0x18,0x30,0x22,0x26,0x2C,0x18,0x30,0x7F,0x4F,0x40}},	// "Q"
{
{0x04,0xFC,0xFC,0x04,0x04,0x04,0x0C,0x18,0xF8,0xE0,0x00,0x00},
{0x00,0xFF,0xFF,0x08,0x38,0xF8,0xCC,0x06,0x07,0x01,0x00,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x00,0x03,0x2F,0x3C,0x30,0x20,0x00}},	// "R"
{
{0xF0,0xF8,0x88,0x0C,0x04,0x04,0x0C,0x18,0x38,0x7C,0x00,0x00},
{0x00,0x01,0x03,0x07,0x0E,0x1C,0x38,0x70,0xE0,0xC0,0x80,0x00},
{0x3F,0x1E,0x0C,0x18,0x30,0x20,0x20,0x30,0x18,0x0F,0x07,0x00}},	// "S"
{
{0x7C,0x1C,0x0C,0x04,0x04,0xFC,0xFC,0x04,0x04,0x0C,0x1C,0x7C},
{0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x20,0x20,0x3F,0x3F,0x20,0x20,0x00,0x00,0x00}},	// "T"
{
{0x04,0xFC,0xFC,0x04,0x00,0x00,0x00,0x04,0xFC,0xFC,0x04,0x00},
{0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00},
{0x00,0x0F,0x1F,0x30,0x20,0x20,0x20,0x30,0x1F,0x0F,0x00,0x00}},	// "U"
{
{0x04,0xFC,0xFC,0x04,0x00,0x00,0x00,0x04,0xFC,0xFC,0x04,0x00},
{0x00,0x00,0x3F,0xFF,0xC0,0x00,0xC0,0xFF,0x3F,0x00,0x00,0x00},
{0x00,0x00,0x00,0x03,0x1F,0x3C,0x1F,0x03,0x00,0x00,0x00,0x00}},	// "V"
{
{0x04,0xFC,0xFC,0x04,0x00,0x00,0x00,0x00,0x04,0xFC,0xFC,0x04},
{0x00,0x03,0xFF,0x80,0x00,0xFC,0xFC,0x00,0x80,0xFF,0x03,0x00},
{0x00,0x00,0x03,0x3F,0x3C,0x03,0x03,0x3C,0x3F,0x03,0x00,0x00}},	// "W"
{
{0x04,0x1C,0x7C,0xE4,0x80,0x00,0x00,0x80,0xE4,0x7C,0x1C,0x04},
{0x00,0x00,0x00,0x81,0xE7,0x7E,0x7E,0xE7,0x81,0x00,0x00,0x00},
{0x20,0x38,0x3E,0x27,0x01,0x00,0x00,0x01,0x27,0x3E,0x38,0x20}},	// "X"
{
{0x04,0x1C,0xFC,0xE4,0x00,0x00,0x00,0x00,0xE4,0xFC,0x1C,0x04},
{0x00,0x00,0x00,0x07,0x1F,0xF8,0xF8,0x1F,0x07,0x00,0x00,0x00},
{0x00,0x00,0x00,0x20,0x20,0x3F,0x3F,0x20,0x20,0x00,0x00,0x00}},	// "Y"
{
{0x7C,0x1C,0x0C,0x04,0x04,0x04,0xC4,0xF4,0x3C,0x1C,0x00,0x00},
{0x00,0x00,0xC0,0xF0,0x3C,0x0F,0x03,0x00,0x00,0x00,0x00,0x00},
{0x3C,0x3F,0x23,0x20,0x20,0x20,0x20,0x20,0x30,0x38,0x3E,0x00}} 	// "Z"
};

static const uint8_t LowerPrintData[26][OLED_PRINT_ASCII_HIGHT/8][OLED_PRINT_ASCII_WIDTH] = {
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x08,0x1C,0x9E,0x8A,0xC2,0x42,0x42,0x46,0xFC,0xF8,0x00,0x00},
{0x1E,0x1F,0x31,0x20,0x20,0x20,0x30,0x10,0x1F,0x3F,0x20,0x00}},	// "a"
{
{0x04,0xFC,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0xFF,0xFF,0x0C,0x06,0x02,0x02,0x06,0x1C,0xF8,0xE0,0x00},
{0x20,0x3F,0x1F,0x18,0x30,0x20,0x20,0x30,0x1C,0x0F,0x03,0x00}},	// "b"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0xE0,0xF8,0x1C,0x06,0x02,0x02,0x02,0x16,0x3C,0x3C,0x18,0x00},
{0x03,0x0F,0x1C,0x30,0x20,0x20,0x20,0x20,0x30,0x18,0x08,0x00}},	// "c"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0xFC,0xFC,0x00},
{0xE0,0xF8,0x1C,0x06,0x02,0x02,0x02,0x06,0x0C,0xFF,0xFF,0x00},
{0x03,0x0F,0x1C,0x30,0x20,0x20,0x20,0x30,0x18,0x1F,0x3F,0x00}},	// "d"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0xE0,0xF8,0x9C,0x86,0x82,0x82,0x82,0x86,0x9C,0xF8,0xE0,0x00},
{0x03,0x0F,0x1C,0x30,0x20,0x20,0x20,0x20,0x30,0x18,0x08,0x00}},	// "e"
{
{0x00,0x00,0x00,0xF0,0xF8,0x0C,0x04,0x04,0x1C,0x3C,0x18,0x00},
{0x02,0x02,0x02,0xFF,0xFF,0x02,0x02,0x02,0x00,0x00,0x00,0x00},
{0x00,0x20,0x20,0x3F,0x3F,0x20,0x20,0x00,0x00,0x00,0x00,0x00}},	// "f"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x38,0x7C,0xC6,0x82,0x82,0x82,0xC6,0x7C,0x3C,0x06,0x06,0x00},
{0x3C,0x7F,0xC3,0x82,0x86,0x84,0x84,0x8C,0xCC,0x78,0x30,0x00}},	// "g"
{
{0x04,0xFC,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0xFF,0xFF,0x18,0x0C,0x06,0x02,0x06,0xFC,0xF8,0x00,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x00,0x00,0x20,0x3F,0x3F,0x20,0x00}},	// "h"
{
{0x00,0x00,0x00,0x00,0x18,0x3C,0x3C,0x18,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x02,0x02,0xFE,0xFE,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x20,0x20,0x3F,0x3F,0x20,0x20,0x00,0x00,0x00}},	// "i"
{
{0x00,0x00,0x00,0x00,0x00,0x18,0x3C,0x3C,0x18,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x02,0x02,0xFE,0xFE,0x00,0x00,0x00},
{0x30,0x78,0xF8,0x90,0x80,0x80,0xC0,0x7F,0x3F,0x00,0x00,0x00}},	// "j"
{
{0x04,0xFC,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0xFF,0xFF,0xC0,0x60,0xF0,0xDA,0x0E,0x06,0x02,0x02,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x00,0x03,0x2F,0x3C,0x30,0x20,0x00}},	// "k"
{
{0x00,0x00,0x00,0x04,0x04,0xFC,0xFC,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x20,0x20,0x3F,0x3F,0x20,0x20,0x00,0x00,0x00}},	// "l"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x02,0xFE,0xFE,0x04,0x06,0xFE,0xFC,0x06,0x06,0xFE,0xFC,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x3F,0x3F,0x20,0x00,0x3F,0x3F,0x20}},	// "m"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x02,0xFE,0xFE,0x18,0x0C,0x06,0x02,0x06,0xFC,0xF8,0x00,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x00,0x00,0x20,0x3F,0x3F,0x20,0x00}},	// "n"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0xE0,0xF8,0x1C,0x06,0x02,0x02,0x02,0x06,0x1C,0xF8,0xE0,0x00},
{0x03,0x0F,0x1C,0x30,0x20,0x20,0x20,0x30,0x1C,0x0F,0x03,0x00}},	// "o"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x02,0xFE,0xFE,0x0C,0x06,0x02,0x02,0x06,0x1C,0xF8,0xE0,0x00},
{0x80,0xFF,0xFF,0x86,0x0C,0x08,0x08,0x0C,0x07,0x03,0x00,0x00}},	// "p"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0xE0,0xF8,0x1C,0x06,0x02,0x02,0x06,0x0C,0xFC,0xFE,0x00,0x00},
{0x00,0x03,0x07,0x0C,0x08,0x08,0x0C,0x86,0xFF,0xFF,0x80,0x00}},	// "q"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x02,0xFE,0xFE,0x18,0x0C,0x06,0x02,0x02,0x1E,0x1C,0x08,0x00},
{0x20,0x3F,0x3F,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},	// "r"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x38,0x7C,0x66,0xC2,0xC2,0x82,0x82,0x86,0x0C,0x1C,0x00,0x00},
{0x1C,0x18,0x30,0x20,0x20,0x20,0x21,0x21,0x33,0x1F,0x0E,0x00}},	// "s"
{
{0x00,0x00,0x00,0xE0,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x02,0x02,0x02,0xFF,0xFF,0x02,0x02,0x02,0x02,0x02,0x00,0x00},
{0x00,0x00,0x00,0x0F,0x1F,0x30,0x20,0x20,0x30,0x1C,0x0C,0x00}},	// "t"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x02,0xFE,0xFE,0x00,0x00,0x00,0x00,0x02,0xFE,0xFE,0x00,0x00},
{0x00,0x0F,0x1F,0x30,0x20,0x30,0x18,0x0C,0x3F,0x3F,0x20,0x00}},	// "u"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x02,0x0E,0x7E,0xF2,0x80,0x00,0x80,0xF2,0x7E,0x0E,0x02,0x00},
{0x00,0x00,0x00,0x03,0x1F,0x3C,0x1F,0x03,0x00,0x00,0x00,0x00}},	// "v"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x02,0x3E,0xFE,0xC2,0x00,0xF0,0xF0,0x00,0xC2,0xFE,0x3E,0x02},
{0x00,0x00,0x03,0x3F,0x3E,0x03,0x03,0x3E,0x3F,0x03,0x00,0x00}},	// "w"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x02,0x0E,0x1E,0x32,0xE0,0xC0,0xE0,0x32,0x1E,0x0E,0x02,0x00},
{0x20,0x38,0x3C,0x26,0x03,0x01,0x03,0x26,0x3C,0x38,0x20,0x00}},	// "x"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x02,0x06,0x1E,0x7A,0xE0,0x80,0x80,0xF2,0x7E,0x0E,0x02,0x00},
{0x60,0xE0,0x80,0x80,0xE1,0x7F,0x1F,0x03,0x00,0x00,0x00,0x00}},	// "y"
{
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x0E,0x02,0x02,0x82,0xC2,0x62,0x32,0x1A,0x0E,0x06,0x00},
{0x38,0x3C,0x26,0x23,0x21,0x20,0x20,0x20,0x20,0x30,0x3C,0x00}},	// "z"
};

static const uint8_t Text_shang[3][24] = {
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x02,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00
},
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x02,0x02,0x02,0x02,0x02,0x03,0x03,0x03,0x02,0x00,0x00,0x00
},
{
	0x00,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x7F,0x7F,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x60,0x70,0x60,0x40,0x00
}
};

static const uint8_t Text_ban[3][24] = {
{
	0x00,0x08,0x08,0x08,0xF8,0xF8,0x08,0x0C,0x0C,0x88,0x00,0xFF,0xFE,0x02,0x08,0x08,0x08,0xF8,0xF8,0x08,0x0C,0x0C,0x08,0x00
},
{
	0x00,0x02,0x02,0x02,0xFF,0xFF,0x02,0x13,0x9A,0x8F,0x00,0xFF,0xFF,0x00,0x08,0x08,0x08,0xFF,0xFF,0x08,0x0C,0x08,0x00,0x00
},
{
	0x00,0x04,0x0C,0x06,0x03,0x43,0x21,0x21,0x10,0x18,0x0E,0x07,0x03,0x10,0x10,0x10,0x10,0x1F,0x1F,0x10,0x18,0x18,0x10,0x00
}
};

static const uint8_t Text_sia[3][24] = {
{
	0x00,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0xF8,0xF8,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x0C,0x0E,0x0C,0x08,0x00
},
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x02,0x02,0x06,0x0C,0x1C,0x38,0x00,0x00,0x00,0x00,0x00
},
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
}
};

static const uint8_t Text_ke[3][24] = {
{
	0x00,0x08,0x88,0x88,0x88,0x89,0x8E,0x88,0xC8,0x88,0x0C,0x08,0x00,0x00,0xFE,0x44,0x44,0x44,0xFC,0x44,0x44,0xFE,0x04,0x00
},
{
	0x00,0x00,0x08,0x88,0x88,0x88,0x88,0x88,0xCC,0x88,0x00,0x00,0x00,0x20,0x2F,0x24,0xA4,0x64,0xFF,0xE4,0x24,0x2F,0x30,0x20
},
{
	0x00,0x00,0x00,0x7F,0x10,0x10,0x10,0x10,0xFF,0x00,0x00,0x00,0x00,0x10,0x08,0x06,0x01,0x00,0x7F,0x00,0x06,0x1C,0x18,0x10
}
};

static const uint8_t Text_jhan[3][24] = {
{
	0x00,0x80,0x80,0x80,0x82,0x8C,0xB8,0x90,0x80,0xC0,0xC0,0x80,0x00,0x00,0x00,0xFF,0xFE,0x82,0x80,0x80,0xC0,0xC0,0x80,0x00
},
{
	0x00,0x00,0x04,0xF8,0xF0,0x00,0x00,0xF0,0x3E,0x0C,0x04,0xF0,0xE0,0x20,0x20,0x3F,0x3F,0x20,0x20,0xF0,0xF0,0x20,0x00,0x00
},
{
	0x00,0x08,0x18,0x18,0x09,0x08,0x07,0x04,0x04,0x02,0x02,0xFF,0x7F,0x10,0x10,0x10,0x10,0x10,0x10,0xFF,0x7F,0x00,0x00,0x00
}
};

static const uint8_t Text_hao[3][24] = {
{
	0x00,0x00,0x00,0xFC,0x04,0x04,0x04,0x04,0x04,0xFE,0x00,0x00,0x00,0x00,0xC0,0x40,0x40,0xFF,0x4A,0x48,0x4C,0x48,0xE0,0x00
},
{
	0x00,0x08,0x08,0xCB,0xB9,0x89,0x89,0x89,0x89,0xCB,0x0C,0x08,0x00,0x00,0xFF,0x04,0xC4,0x8F,0x92,0xD3,0x90,0x11,0x0E,0x00
},
{
	0x00,0x00,0x00,0x01,0x00,0x20,0x20,0xE0,0x3E,0x01,0x00,0x00,0x00,0x70,0x0F,0x40,0x3F,0x00,0x00,0x3F,0x60,0x60,0x60,0x30
}
};

static const uint8_t Symbol_0x3A[OLED_PRINT_ASCII_HIGHT/8][OLED_PRINT_ASCII_WIDTH] = {
{0x00,0x00,0x00,0x00,0x80,0xC0,0xC0,0x80,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x81,0xC3,0xC3,0x81,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x01,0x03,0x03,0x01,0x00,0x00,0x00,0x00}};

uint8_t *Get_ASCII_Print_Pixel(char Data);
void Print_ASCII(uint8_t x, uint8_t y, char data);
void Print_shang(uint8_t x, uint8_t y);
void Print_ban(uint8_t x, uint8_t y);
void Print_sia(uint8_t x, uint8_t y);
void Print_ke(uint8_t x, uint8_t y);
void Print_jhan(uint8_t x, uint8_t y);
void Print_hao(uint8_t x, uint8_t y);

#endif