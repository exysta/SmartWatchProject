/*
 * display.h
 *
 *  Created on: May 5, 2025
 *      Author: exysta
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "st7789.h" // Inclure le pilote de l'écran
#include "main.h"   // Pour UI_Screen_State_t et SmartWatchData_t
#include "common_defs.h"

#define COLOR_HIGH_BYTE(color) ((uint8_t)((color) >> 8))
#define COLOR_LOW_BYTE(color)  ((uint8_t)((color) & 0xFF))


// (Gardez vos définitions de UI_Screen_State_t et SmartWatchData_t de la réponse précédente)

// Fonctions publiques du module display
void Display_Init();
void Display_ShowMessage(const char* message, uint16_t color);
void Display_Image(uint16_t x_center, uint16_t y_center,uint16_t x_width, uint16_t y_width,const uint16_t *img,size_t size);

// Nouvelle fonction pour dessiner un cœur

void Display_DrawHeart(uint16_t x_center, uint16_t y_center);
void Display_DrawThermo(uint16_t x_center, uint16_t y_center);

void Display_HeartRate(uint16_t x_center, uint16_t y_center,const SmartWatchData_t* pData);
void Display_EnvironnementData(uint16_t x, uint16_t y,const SmartWatchData_t* pData);

void Display_RenderAnimation(uint16_t x_center, uint16_t y_center,uint16_t x_width, uint16_t y_width,const uint16_t **frame_array,uint8_t animation_frame_array_len,size_t pixel_count);

void split_color_array(const uint16_t *input, uint8_t *output, size_t length);

#endif /* INC_DISPLAY_H_ */
