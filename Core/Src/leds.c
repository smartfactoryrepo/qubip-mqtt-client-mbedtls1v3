/*
 * leds.c
 *
 *  Created on: Aug 12, 2024
 *      Author: federico parente
 */
#include "leds.h"
#include "main.h"
#include "cmsis_os.h"

/*
 * Events list:
 *
 * 1) FreeRTOS Stack Overflow
 * 2) FreeRTOS Malloc Failed
 * 3) Hard Fault Handler
 * 4) MBEDTLS TLS Handshake Failed
 * 5) MQTT Client Not Connected (yet)
 * 6) MQTT Client send a message
 *
 * Fatal Error (led red): 1, 2, 3
 * Network Error (led blue): 4
 * Info (led green): 5, 6
 *
 * LED message in morse code:
 *
 * 1) SO -> ... ---
 * 2) MF -> -- ..-.
 * 4) TLS -> - .-.. ...
 *
 * LED blink:
 *
 * 3) 125ms period (led blue)
 * 5) 125ms period (led green)
 * 6) 500ms period at each message sent  (led green)
 *
 */
void dot(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(DOT_DURATION);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	HAL_Delay(SAME_LETTER_DURATION);
}

void dash(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(DASH_DURATION);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	HAL_Delay(SAME_LETTER_DURATION);
}


void leds_blink_on_stackoverflow()
{
	// 1) SO -> ... ---
	dot(LD3_GPIO_Port, LD3_Pin);  dot(LD3_GPIO_Port, LD3_Pin);  dot(LD3_GPIO_Port, LD3_Pin);
	HAL_Delay(DIFFERENT_LETTER_DURATION);
	dash(LD3_GPIO_Port, LD3_Pin); dash(LD3_GPIO_Port, LD3_Pin); dash(LD3_GPIO_Port, LD3_Pin);
	HAL_Delay(DIFFERENT_LETTER_DURATION + END_WORD);
}

void leds_flash_error_on_malloc_failure()
{
	// 2) MF -> -- ..-.
	dash(LD3_GPIO_Port, LD3_Pin); dash(LD3_GPIO_Port, LD3_Pin);
	HAL_Delay(DIFFERENT_LETTER_DURATION);
	dot(LD3_GPIO_Port, LD3_Pin);  dot(LD3_GPIO_Port, LD3_Pin);  dash(LD3_GPIO_Port, LD3_Pin); dot(LD3_GPIO_Port, LD3_Pin);
	HAL_Delay(DIFFERENT_LETTER_DURATION + END_WORD);
}

void leds_signal_hard_fault()
{
	// 3) 125ms period
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    HAL_Delay(62);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    HAL_Delay(62);
}



void leds_indicate_tls_handshake_failure()
{
	// 4) TLS -> - .-.. ...
	dash(LD2_GPIO_Port, LD2_Pin);
	HAL_Delay(DIFFERENT_LETTER_DURATION);
	dot(LD2_GPIO_Port, LD2_Pin);  dash(LD2_GPIO_Port, LD2_Pin); dot(LD2_GPIO_Port, LD2_Pin);  dot(LD2_GPIO_Port, LD2_Pin);
	HAL_Delay(DIFFERENT_LETTER_DURATION);
	dot(LD2_GPIO_Port, LD2_Pin);  dot(LD2_GPIO_Port, LD2_Pin);  dot(LD2_GPIO_Port, LD2_Pin);
	HAL_Delay(DIFFERENT_LETTER_DURATION + END_WORD);
}

void leds_blink_while_mqtt_client_disconnected()
{
	// 5) 125ms period
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	osDelay(62);
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	osDelay(62);
}

void leds_blink_on_mqtt_message_sent()
{
	// 6) 500ms period
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	osDelay(250);
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	osDelay(250);
}

