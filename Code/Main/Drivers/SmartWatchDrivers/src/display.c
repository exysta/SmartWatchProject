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
const uint16_t thermometer_icon_width = 40;
const uint16_t thermometer_icon_height = 40;
const uint16_t weather_animation_width = 48;
const uint16_t weather_animation_height = 48;
const uint16_t heart_animation_width = 48;
const uint16_t heart_animation_height = 48;

//top middle of screen
const static uint16_t gif_center_x = 160;
const static uint16_t gif_center_y = 30;

extern SmartWatchData_t SmartWatchData_handle;
static UI_Screen_State_t previous_screenState;
static UI_Screen_State_t current_screenState;

// --- Static variables for animation state ---
static uint8_t  s_animation_current_frame = 0;
static uint32_t s_animation_last_update_tick = 0;


#define ANIMATION_FRAME_DELAY_MS 80 // Adjust for desired speed (milliseconds)

void Display_Init(UI_Screen_State_t screenState)
{
	ST7789_Init();
	HAL_GPIO_WritePin(ST7789_BLK_GPIO_Port  , ST7789_BLK_Pin, GPIO_PIN_SET);
    // 2) Disable SPI2 before touching CR2
    __HAL_SPI_DISABLE(&ST7789_SPI_PORT);

    // 3) Turn on hardware‐NSS pulse management (NSSP) in CR2
    //    and be sure SSOE (NSS output enable) is set too:
    MODIFY_REG(
      ST7789_SPI_PORT.Instance->CR2,
      SPI_CR2_NSSP  | SPI_CR2_SSOE,    // mask
      SPI_CR2_NSSP  | SPI_CR2_SSOE     // value
    );

    // 4) Re-enable SPI2 so your change takes effect
    __HAL_SPI_ENABLE(&ST7789_SPI_PORT);
	previous_screenState = screenState;
	current_screenState = screenState;
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

void Display_DrawThermo(uint16_t x_center, uint16_t y_center)
{
	Display_Image(x_center,y_center,thermometer_icon_width,thermometer_icon_height,thermometer_icon_data,sizeof(thermometer_icon_data) );
}

void Display_HeartRate(uint16_t x_center, uint16_t y_center,const SmartWatchData_t* pData)
{
	uint8_t hr = pData->heart_rate;
	uint16_t spo2 = pData->spo2;

	char hr_buf[32];
	char spo2_buf[32];

	snprintf(hr_buf, sizeof(hr_buf), " HeartRate : %u", (unsigned) hr);
	snprintf(spo2_buf, sizeof(spo2_buf), " SPO2 : %u", (unsigned) spo2);

	//space between lines of text
	int text_y_offset = 30;

	ST7789_WriteString(x_center, y_center , hr_buf, Font_11x18, RED, BLACK);
	ST7789_WriteString(x_center, y_center + text_y_offset, spo2_buf, Font_11x18, RED, BLACK);


	Display_RenderAnimation(gif_center_x,gif_center_y,heart_animation_width,heart_animation_height,heart_gif_array,heart_gif_array_LEN,heart_gif_frame_pixel_count);
}

void Display_Clock(uint16_t x_center, uint16_t y_center,const SmartWatchData_t* pData)
{
	uint16_t year = pData->gps_data.year;
	uint8_t month = pData->gps_data.month;
	uint8_t day = pData->gps_data.day;
	uint8_t hour = pData->gps_data.hour;
	uint8_t min = pData->gps_data.min;
	uint8_t sec = pData->gps_data.sec;

	hour = (hour +3 )%24; // for UTC
	char date_buf[64];
	char clock_buf[32];

	snprintf(date_buf, sizeof(date_buf), "  %d. %u. %u", (unsigned) year, (unsigned) month, (unsigned) day);
	snprintf(clock_buf, sizeof(clock_buf), " %u h %u min %u sec",  (unsigned) hour, (unsigned) min, (unsigned) sec);

	//space between lines of text
	int text_y_offset = 30;

	ST7789_WriteString(x_center, y_center , date_buf, Font_11x18, GREEN, BLACK);
	ST7789_WriteString(x_center, y_center + text_y_offset, clock_buf, Font_11x18, GREEN, BLACK);

}



void Display_Position(uint16_t x_center, uint16_t y_center,const SmartWatchData_t* pData)
{
	int32_t lat = pData->gps_data.fLat;
	int32_t lon = pData->gps_data.fLon;
	int32_t altitude = pData->gps_data.hMSL / 1000.0f;;


	char pos_buf[80];
	char height_buf[64];

	snprintf(pos_buf, sizeof(pos_buf), "  latitude %ld longitude : %ld", lat, lon);
	snprintf(height_buf, sizeof(height_buf), "  height %ld ", altitude);

	//space between lines of text
	int text_y_offset = 30;

	ST7789_WriteString(x_center, y_center , pos_buf, Font_11x18, GREEN, BLACK);
	ST7789_WriteString(x_center, y_center + text_y_offset, height_buf, Font_11x18, GREEN, BLACK);

}

void Display_Motion(uint16_t x_center, uint16_t y_center,const SmartWatchData_t* pData)
{
	float accel_x = pData->accel_g[0][0];
	float accel_y = pData->accel_g[0][1];
	float accel_z = pData->accel_g[0][2];


	float gyro_x = pData->gyro_dps[0][0];
	float gyro_y = pData->gyro_dps[0][1];
	float gyro_z = pData->gyro_dps[0][2];

	char accel_buf[64];
	char gyro_buf[64];

	snprintf(accel_buf, sizeof(accel_buf), "  accel x:%.4f y :%.4f z: %.4f",  accel_x, accel_y, accel_z);
	snprintf(gyro_buf, sizeof(gyro_buf), "  gyro x:%.4f y :%.4f z: %.4f",  gyro_x, gyro_y, gyro_z);

	//space between lines of text
	int text_y_offset = 30;

	ST7789_WriteString(x_center, y_center , accel_buf, Font_11x18, GREEN, BLACK);
	ST7789_WriteString(x_center, y_center + text_y_offset, gyro_buf, Font_11x18, GREEN, BLACK);

}

void Display_EnvironnementData(uint16_t x, uint16_t y,const SmartWatchData_t* pData)
{


	//uint8_t hr = SmartWatchData_handle.heart_rate;
	uint16_t temp = pData->temperature;
	uint16_t pressure = pData->pressure;
	uint16_t humidity = pData->humidity;


	temp = 0;
	//pressure = 0;
	humidity = 0;

	char buf_temp[32];
	char buf_pressure[32];
	char buf_humidity[32];

	snprintf(buf_temp, sizeof(buf_temp), " Temperature : %d", temp);
	snprintf(buf_pressure, sizeof(buf_pressure), " Pressure : %d", pressure);
	snprintf(buf_humidity, sizeof(buf_humidity), " Humidity : %d", humidity);

	//space between lines of text
	int text_y_offset = 30;

	// no border checking so be careful when calling this
	ST7789_WriteString(x , y , buf_temp, Font_11x18, GREEN, BLACK);
	ST7789_WriteString(x , y + text_y_offset , buf_pressure, Font_11x18, GREEN, BLACK);
	ST7789_WriteString(x , y + 2 * text_y_offset , buf_humidity, Font_11x18, GREEN, BLACK);

//	Display_DrawThermo(x,y);
	Display_RenderAnimation(gif_center_x,gif_center_y,weather_animation_width,weather_animation_height,weather_gif_array,weather_gif_array_LEN,weather_gif_frame_pixel_count);

}

// --- Animation Function ---
// This function should be called repeatedly (e.g., from the main loop or Display_Update)
// when the animation needs to be displayed.
void Display_RenderAnimation(uint16_t x_center, uint16_t y_center,uint16_t x_width, uint16_t y_width,const uint16_t **frame_array,uint8_t animation_frame_array_len,size_t pixel_count)
{
    uint32_t current_tick = HAL_GetTick();

    // Check if it's time to update the frame
    if (current_tick - s_animation_last_update_tick >= ANIMATION_FRAME_DELAY_MS) {
        s_animation_last_update_tick = current_tick;

        // Get the pointer to the current frame data
        const uint16_t* frame_data = frame_array[s_animation_current_frame];
    	Display_Image(x_center,y_center,x_width,y_width,frame_data,pixel_count );

        // Advance to the next frame
        s_animation_current_frame++;
        if (s_animation_current_frame >= animation_frame_array_len) {
            s_animation_current_frame = 0; // Loop back to the beginning
        }
    }
    // If not enough time has passed, do nothing, the previous frame remains displayed.
}


void Display_RenderClock(const SmartWatchData_t* pData)
{
	Display_Clock(40,60,pData);

}

void Display_RenderEnvironmental(const SmartWatchData_t* pData)
{
	Display_EnvironnementData(40,60,pData);
}

void Display_RenderMotion(const SmartWatchData_t* pData)
{
	Display_Motion(40,60,pData);
}

void Display_RenderHeartRate(const SmartWatchData_t* pData)
{
	Display_HeartRate(40, 60, pData);

}

void Display_RenderGPS(const SmartWatchData_t* pData)
{
	Display_Position(40,50,pData);
}


// --- Update the main Display_Update function ---
void Display_Update(UI_Screen_State_t screen, const SmartWatchData_t* pData) {

	previous_screenState = current_screenState;
	current_screenState = screen;
    // OPTIONAL: Clear screen only when changing screen state, not every frame of animation.
    if (current_screenState != previous_screenState) {
        ST7789_Fill_Color(BLACK); // Clear only when screen changes
        s_animation_current_frame = 0; // Reset animation frame when switching to it
        s_animation_last_update_tick = HAL_GetTick(); // Reset timer to draw first frame immediately
    }

    switch (screen) {
        case SCREEN_CLOCK:
            Display_RenderClock(pData);
            break;
        case SCREEN_ENVIRONMENTAL:
            Display_RenderEnvironmental(pData);
            break;
        case SCREEN_MOTION:
            Display_RenderMotion(pData);
            break;
        case SCREEN_HEART_RATE: // Example: Show static heart + text here
             Display_RenderHeartRate(pData); // Assuming this renders text + static heart
             break;
        case SCREEN_GPS_STATUS:
            Display_RenderGPS(pData);
            break;

        // ... other cases ...
        default:
            ST7789_WriteString(30, 20, "Unknown Screen", Font_11x18, RED, BLACK);
            break;
    }
}
