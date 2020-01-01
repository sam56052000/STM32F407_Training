#include "OLED_Print.h"

uint8_t *Get_ASCII_Print_Pixel(char Data)
{
	if( (Data >= '0') && (Data <= '9') )
	{
		return (uint8_t *)&DigitPrintData[Data - '0'];
	}
	else if( (Data >= 'A') && (Data <= 'Z') )
	{
		return (uint8_t *)&UpperPrintData[Data - 'A'];
	}
	else if( (Data >= 'a') && (Data <= 'z') )
	{
		return (uint8_t *)&LowerPrintData[Data - 'a'];
	}
	else
	{
		switch(Data)
		{
			case ':':
			{
				return (uint8_t *)Symbol_0x3A;
			}
			break;

			default:
			{
				return (uint8_t *)0x0;
			}
			break;
		}
	}
}

void Print_ASCII(uint8_t x, uint8_t y, char data)
{
	uint8_t *pixel_array = Get_ASCII_Print_Pixel(data);

	uint8_t i, j, k;

	for(i = 0; i < 24; i++)
	{
		for(j = 0; j < 12; j++)
		{
			uint8_t pixel = ((pixel_array[((i/8) * OLED_PRINT_ASCII_WIDTH) + j]) >> (i%8)) & 0x1;

			for(k = 0; k < 2; k++)
			{
				OLED_Print_Function(x + j, y + i*2 + k, pixel);
			}
		}
	}
}

void Print_shang(uint8_t x, uint8_t y)
{
	uint8_t i, j, k;

	for(i = 0; i < 24; i++)
	{
		for(j = 0; j < 24; j++)
		{
			uint8_t pixel = ((Text_shang[i/8][j])>>(i%8)) & 0x1;

			for(k = 0; k < 2; k++)
			{
				OLED_Print_Function(x + j, y + i*2 + k, pixel);
			}
		}
	}	
}

void Print_ban(uint8_t x, uint8_t y)
{
	uint8_t i, j, k;

	for(i = 0; i < 24; i++)
	{
		for(j = 0; j < 24; j++)
		{
			uint8_t pixel = ((Text_ban[i/8][j])>>(i%8)) & 0x1;

			for(k = 0; k < 2; k++)
			{
				OLED_Print_Function(x + j, y + i*2 + k, pixel);
			}
		}
	}	
}

void Print_sia(uint8_t x, uint8_t y)
{
	uint8_t i, j, k;

	for(i = 0; i < 24; i++)
	{
		for(j = 0; j < 24; j++)
		{
			uint8_t pixel = ((Text_sia[i/8][j])>>(i%8)) & 0x1;

			for(k = 0; k < 2; k++)
			{
				OLED_Print_Function(x + j, y + i*2 + k, pixel);
			}
		}
	}	
}

void Print_ke(uint8_t x, uint8_t y)
{
	uint8_t i, j, k;

	for(i = 0; i < 24; i++)
	{
		for(j = 0; j < 24; j++)
		{
			uint8_t pixel = ((Text_ke[i/8][j])>>(i%8)) & 0x1;

			for(k = 0; k < 2; k++)
			{
				OLED_Print_Function(x + j, y + i*2 + k, pixel);
			}
		}
	}	
}

void Print_jhan(uint8_t x, uint8_t y)
{
	uint8_t i, j, k;

	for(i = 0; i < 24; i++)
	{
		for(j = 0; j < 24; j++)
		{
			uint8_t pixel = ((Text_jhan[i/8][j])>>(i%8)) & 0x1;

			for(k = 0; k < 2; k++)
			{
				OLED_Print_Function(x + j, y + i*2 + k, pixel);
			}
		}
	}	
}

void Print_hao(uint8_t x, uint8_t y)
{
	uint8_t i, j, k;

	for(i = 0; i < 24; i++)
	{
		for(j = 0; j < 24; j++)
		{
			uint8_t pixel = ((Text_hao[i/8][j])>>(i%8)) & 0x1;

			for(k = 0; k < 2; k++)
			{
				OLED_Print_Function(x + j, y + i*2 + k, pixel);
			}
		}
	}	
}