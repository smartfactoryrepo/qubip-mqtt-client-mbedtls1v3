/*
 * DS18B20.h
 *
 *  Created on: Jan 27, 2025
 *      Author: federico
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include "onewire.h"

/**
 * @brief Inizializza il sensore DS18B20.
 *
 * @param handle Puntatore alla struttura di gestione OneWire.
 * @param uart Puntatore alla struttura della UART da utilizzare.
 */
void DS18B20_Init(OneWire_Handle_t *handle, UART_HandleTypeDef* uart);

/**
 * @brief Avvia la conversione della temperatura sul sensore DS18B20.
 *
 * @param handle Puntatore alla struttura di gestione OneWire.
 */
void DS18B20_SampleTemp(OneWire_Handle_t *handle);

/**
 * @brief Legge la temperatura dal sensore DS18B20.
 *
 * @param handle Puntatore alla struttura di gestione OneWire.
 * @return float La temperatura letta in gradi Celsius.
 */
float DS18B20_ReadTemp(OneWire_Handle_t *handle);


#endif /* INC_DS18B20_H_ */
