/*
 * onewire.h
 *
 *  Created on: Jan 24, 2025
 *      Author: federico
 */

#ifndef INC_ONEWIRE_H_
#define INC_ONEWIRE_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_usart.h"
#include <stdint.h>


/* Enumerazione per rappresentare lo stato del bus OneWire */
typedef enum {
    ONEWIRE_OK = 0,        // Operazione completata con successo
    ONEWIRE_ERROR,         // Errore generico
    ONEWIRE_NO_DEVICE,     // Nessun dispositivo rilevato
    ONEWIRE_BUS_BUSY       // Bus occupato
} OneWire_Status_t;

/* Struttura per rappresentare il contesto di una connessione OneWire */
typedef struct {
	UART_HandleTypeDef* uart_port;        // Identificatore della UART utilizzata
    uint8_t device_address[8]; 			  // Indirizzo unico del dispositivo OneWire
} OneWire_Handle_t;



/**
 * @brief Inizializza una connessione OneWire su una specifica UART.
 *
 * @param handle Puntatore alla struttura di gestione OneWire.
 * @param uart Identificatore della UART da utilizzare.
 * @return OneWire_Status_t Stato dell'operazione.
 */
OneWire_Status_t OneWire_Init(OneWire_Handle_t *handle, UART_HandleTypeDef* uart);

/**
 * @brief Effettua un reset sul bus OneWire.
 *
 * @param handle Puntatore alla struttura di gestione OneWire.
 * @return OneWire_Status_t Stato dell'operazione.
 */
OneWire_Status_t OneWire_Reset(OneWire_Handle_t *handle);

/**
 * @brief Invia un byte sul bus OneWire.
 *
 * @param handle Puntatore alla struttura di gestione OneWire.
 * @param data Byte da inviare.
 * @return OneWire_Status_t Stato dell'operazione.
 */
OneWire_Status_t OneWire_WriteByte(OneWire_Handle_t *handle, uint8_t data);

/**
 * @brief Legge un byte dal bus OneWire.
 *
 * @param handle Puntatore alla struttura di gestione OneWire.
 * @param data Puntatore alla variabile in cui salvare il dato letto.
 * @return OneWire_Status_t Stato dell'operazione.
 */
OneWire_Status_t OneWire_ReadByte(OneWire_Handle_t *handle, uint8_t *data);


/**
 * @brief Legge un bit dal bus OneWire.
 *
 * @param handle Puntatore alla struttura di gestione OneWire.
 * @param data Puntatore alla variabile in cui salvare il dato letto.
 * @return OneWire_Status_t Stato dell'operazione.
 */
OneWire_Status_t OneWire_ReadBit(OneWire_Handle_t *handle, uint8_t *data);



#endif /* INC_ONEWIRE_H_ */
