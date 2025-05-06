/*
 * ble_comms.c
 *
 *  Created on: May 5, 2025
 *      Author: exysta
 */

#include "ble_comms.h"
#include "common_defs.h"
#include "usart.h"
#include <stdio.h>          // For printf() (if using debugging via UART)
#include <string.h>

uint8_t rxBuffer[RX_BUFFER_SIZE];

// Function to send AT command and receive response
void sendATCommand(UART_HandleTypeDef *huart, char *command)
{
	uint8_t txBuffer[100];
	uint8_t rxBuffer[100];

	// Préparer la commande
	sprintf((char*) txBuffer, "%s\r\n", command);

	// Vider le buffer de réception
	HAL_UART_AbortReceive(huart);
	memset(rxBuffer, 0, sizeof(rxBuffer));

	// Envoyer la commande
	HAL_UART_Transmit(huart, txBuffer, strlen((char*) txBuffer), 300);

	// Réception avec timeout étendu
	HAL_UART_Receive(huart, rxBuffer, sizeof(rxBuffer), 2000);

	printf("Received: %s\r\n", rxBuffer);
	HAL_Delay(1500);
}

void configureHM10()
{
	sendATCommand(&BLE_UART, "AT");          // Basic test - should return "OK"
//	  // First connect at current baud rate (likely 9600)
//	  sendATCommand(&huart4, "AT+VERSION");  // Get firmware version
//	  sendATCommand(&huart4, "AT+LADDR");    // Get Bluetooth MAC address
//	  sendATCommand(&huart4, "AT+NAMESmartProject");     // Check current device name
//	  sendATCommand(&huart4, "AT+ROLE0");    // Set to slave mode
//	  sendATCommand(&huart4, "AT+ADVI0");    // Set to 100ms for quick discovery

}

// Start DMA reception with idle line detection
void startUartReception(UART_HandleTypeDef *huart)
{
	HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, RX_BUFFER_SIZE);
	// Optionally disable half-transfer interrupt to reduce overhead
	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
