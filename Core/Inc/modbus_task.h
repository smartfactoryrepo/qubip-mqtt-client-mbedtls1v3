/*
 * modbus_task.h
 *
 *  Created on: Aug 21, 2024
 *      Author: federico
 */

#ifndef INC_MODBUS_TASK_H_
#define INC_MODBUS_TASK_H_
/* Private includes ----------------------------------------------------------*/
#include "FreeRTOS.h"

/* Private define ------------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void ModbusClientTask(void *argument);    		// modbus client task function

#endif /* INC_MODBUS_TASK_H_ */
