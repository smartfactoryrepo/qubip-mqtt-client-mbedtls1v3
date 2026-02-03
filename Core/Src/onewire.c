/*
 * onewire.c
 *
 *  Created on: Jan 24, 2025
 *      Author: federico
 */

#include "onewire.h"



OneWire_Status_t OneWire_Init(OneWire_Handle_t *handle, UART_HandleTypeDef* uart)
{
	if(uart == NULL)
	{
		return ONEWIRE_ERROR;
	}

	handle->uart_port = uart;
	return ONEWIRE_OK;
}

OneWire_Status_t OneWire_Reset(OneWire_Handle_t *handle)
{
	uint8_t ResetByte = 0xF0;
	uint8_t PresenceByte = 0;

	LL_USART_SetBaudRate(handle->uart_port->Instance, HAL_RCC_GetPCLK1Freq(),  handle->uart_port->Init.OverSampling, 9600);
	// Send reset pulse (0xF0)
	HAL_UART_Transmit(handle->uart_port, &ResetByte, 1, 1);
	// Wait for the presence pulse
	HAL_UART_Receive(handle->uart_port, &PresenceByte, 1, 1);
	LL_USART_SetBaudRate(handle->uart_port->Instance, HAL_RCC_GetPCLK1Freq(), handle->uart_port->Init.OverSampling, 115200);
	// Check presence pulse
	if (PresenceByte != ResetByte)
	{
		// Presence pulse detected
		return ONEWIRE_OK;
	}
	else
	{
		// No presence pulse detected
		return ONEWIRE_NO_DEVICE;
	}
    return ONEWIRE_OK;
}

OneWire_Status_t OneWire_ReadBit(OneWire_Handle_t *handle, uint8_t *data)
{
    uint8_t ReadBitCMD = 0xFF;
    uint8_t RxBit;

    // Send Read Bit CMD
    HAL_UART_Transmit(handle->uart_port, &ReadBitCMD, 1, 1);
    // Receive The Bit
    HAL_UART_Receive(handle->uart_port, &RxBit, 1, 1);

    return (RxBit & 0x01) ? ONEWIRE_OK : ONEWIRE_ERROR;
}

OneWire_Status_t OneWire_ReadByte(OneWire_Handle_t *handle, uint8_t *data)
{
	uint8_t RxByte = 0;
	uint8_t dummy = 0;

	for (uint8_t i = 0; i < 8; ++i)
	{
		RxByte >>= 1;
		if (OneWire_ReadBit(handle, &dummy) == ONEWIRE_OK)
		{
			RxByte |= 0x80;
		}
	}

	*data = RxByte;
    return ONEWIRE_OK;
}

OneWire_Status_t OneWire_WriteByte(OneWire_Handle_t *handle, uint8_t data)
{
	uint8_t TxBuffer[8];

    for (int i = 0; i < 8; ++i)
    {
    	if ((data & (1 << i)) != 0)
    	{
    		TxBuffer[i] = 0xFF;
    	}
    	else
    	{
    		TxBuffer[i] = 0;
    	}
    }
    HAL_UART_Transmit(handle->uart_port, TxBuffer, 8, 10);
    return ONEWIRE_OK;
}


