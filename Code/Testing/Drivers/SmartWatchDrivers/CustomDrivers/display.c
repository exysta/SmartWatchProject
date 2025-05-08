/*
 * display.c
 *
 *  Created on: May 5, 2025
 *      Author: exysta
 */

#include "display.h"
#include "fonts.h"

#include "common_defs.h"

#include "fonts.h"  // Assurez-vous que vos polices sont incluses
#include <stdio.h>  // Pour sprintf
#include <string.h> // Pour strlen

const uint16_t heart_icon_width = 40;
const uint16_t heart_icon_height = 40;

extern SmartWatchData_t SmartWatchData_handle;

void Display_Init()
{
	ST7789_Init();

}

//this shit is need to have the output image with the right color
void split_color_array(const uint16_t *input, uint8_t *output, size_t length)
{
	for (size_t i = 0; i < length; ++i)
	{
		output[2 * i] = COLOR_HIGH_BYTE(input[i]);  // High byte first
		output[2 * i + 1] = COLOR_LOW_BYTE(input[i]);   // Then low byte
	}
}

void Display_Image(uint16_t x_center, uint16_t y_center,uint16_t x_width, uint16_t y_width,const uint16_t *img,size_t pixel_count)
{
	// We use the fixed size from the image data now
	uint16_t img_x = x_center - (x_width / 2);
	uint16_t img_y = y_center - (y_width / 2);
	uint8_t result[pixel_count * 2];

	//lenght of the array
	split_color_array(img, result,
			pixel_count/ sizeof(uint16_t));
	ST7789_DrawImage(img_x, img_y, x_width, y_width, result);
}

void Display_DrawHeart(uint16_t x_center, uint16_t y_center)
{

	Display_Image(x_center,y_center,heart_icon_width,heart_icon_width,heart_icon_data,sizeof(heart_icon_data) );
}


void Display_HeartRate(uint16_t x_center, uint16_t y_center)
{
	//uint8_t hr = SmartWatchData_handle.heart_rate;

	uint8_t hr = 72;
	char buf[16];
	snprintf(buf, sizeof(buf), "%u", (unsigned) hr);

	//for coorect rendering of both text and heart
	int text_x_offset = 30;
	int text_y_offset = -5;

	ST7789_WriteString(x_center + text_x_offset, y_center + text_y_offset, buf, Font_11x18, RED, BLACK);
	Display_DrawHeart(x_center, y_center);
}

void Display_EnvironnementData(uint16_t x, uint16_t y)
{
	//uint8_t hr = SmartWatchData_handle.heart_rate;
	uint16_t temp = SmartWatchData_handle.temperature;
	uint16_t pressure = SmartWatchData_handle.pressure;
	uint16_t humidity = SmartWatchData_handle.humidity;

	temp = 0;
	pressure = 0;
	humidity = 0;

	char buf[32];
	snprintf(buf, sizeof(buf), "T :%d, P : %d, H : %d ", temp, pressure,
			humidity);

	ST7789_WriteString(x, y, buf, Font_11x18, GREEN, BLACK);
}
