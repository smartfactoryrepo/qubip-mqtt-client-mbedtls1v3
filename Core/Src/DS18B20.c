/*
 * DS18B20.c
 *
 *  Created on: Jan 27, 2025
 *      Author: federico
 */

#include "DS18B20.h"


void DS18B20_Init(OneWire_Handle_t *handle, UART_HandleTypeDef* uart)
{
	OneWire_Init(handle, uart);
}

void DS18B20_SampleTemp(OneWire_Handle_t *handle)
{
	OneWire_Reset(handle);
	OneWire_WriteByte(handle, 0xCC);	// Skip ROM   (ROM-CMD)
	OneWire_WriteByte(handle, 0x44);	// Convert T  (F-CMD)
}

float DS18B20_ReadTemp(OneWire_Handle_t *handle)
{
	uint8_t temp_lsb = 0;
	uint8_t temp_msb = 0;
	uint16_t temp = 0;
	float temperature = 0.0;

	OneWire_Reset(handle);
	OneWire_WriteByte(handle, 0xCC);	// Skip ROM   (ROM-CMD)
	OneWire_WriteByte(handle, 0xBE);  	// Read Scratchpad  (F-CMD)

	OneWire_ReadByte(handle, &temp_lsb);
	OneWire_ReadByte(handle, &temp_msb);
	temp = ((temp_msb << 8)) | temp_lsb;
	temperature = (float) temp / 16.0;

	return temperature;
}

