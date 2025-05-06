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

#define COLOR_HIGH_BYTE(color) ((uint8_t)((color) >> 8))
#define COLOR_LOW_BYTE(color)  ((uint8_t)((color) & 0xFF))


// (Gardez vos définitions de UI_Screen_State_t et SmartWatchData_t de la réponse précédente)

// Fonctions publiques du module display
int Display_Init(SPI_HandleTypeDef* hspi_display);
void Display_ShowMessage(const char* message, uint16_t color);

// Nouvelle fonction pour dessiner un cœur
void Display_DrawHeart(uint16_t x_center, uint16_t y_center);


#endif /* INC_DISPLAY_H_ */
