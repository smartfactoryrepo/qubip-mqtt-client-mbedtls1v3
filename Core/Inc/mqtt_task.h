/*
 * mqtt_task.h
 *
 *  Created on: Aug 21, 2024
 *      Author: federico
 */

#ifndef INC_MQTT_TASK_H_
#define INC_MQTT_TASK_H_

#include "MQTTClient.h"
#include "cmsis_os.h"


extern MQTTClient mqttClient;
extern osThreadId mqttClientSubTaskHandle;
extern osThreadId mqttClientPubTaskHandle;

/* Private function prototypes -----------------------------------------------*/
void MqttClientSubTask(void const *argument); 	// mqtt client subscribe task function
void MqttClientPubTask(void const *argument); 	// mqtt client publish task function

#endif /* INC_MQTT_TASK_H_ */
