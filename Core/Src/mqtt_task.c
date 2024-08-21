/*
 * mqtt_task.c
 *
 *  Created on: Aug 21, 2024
 *      Author: federico
 */
/* Private includes ----------------------------------------------------------*/
#include "mqtt_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include <string.h>
#include "lwip.h"
#include "lwip/api.h"
#include "MQTTInterface.h"
#include "rng.h"
#include "platform.h"
#include "leds.h"
#include "nanomodbus_interface.h"
#include "iperf_server.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern struct netif gnetif; //extern gnetif

osThreadId mqttClientSubTaskHandle;  //mqtt client task handle
osThreadId mqttClientPubTaskHandle;  //mqtt client task handle

Network mqttNet; 		//mqtt network
MQTTClient mqttClient; 	//mqtt client

static uint8_t sndBuffer[MQTT_BUFSIZE]; //mqtt send buffer
static uint8_t rcvBuffer[MQTT_BUFSIZE]; //mqtt receive buffer
static uint8_t msgBuffer[MQTT_BUFSIZE]; //mqtt message buffer
static uint8_t need_to_reconnect = 0;

/* Private function prototypes -----------------------------------------------*/
int  MqttConnectBroker(void); 					// mqtt broker connect function
void MqttMessageArrived(MessageData *msg); 		// mqtt message callback function


/* Private application code --------------------------------------------------*/
/**
 * @brief  Function implementing the MqttClientSubTask thread.
 * @param  argument: Not used
 * @retval None
 */
void MqttClientSubTask(void const *argument)
{
	for(;;)
	{
		// If disconnected from MQTT broker fast blink the green led
		if(!mqttClient.isconnected)
		{
			leds_blink_while_mqtt_client_disconnected();
			continue;
		}

		// Handle timer and incoming messages
		MQTTYield(&mqttClient, 500); /* Don't wait too long if no traffic is incoming */
		osDelay(100);
	}
}



/**
 * @brief  Function implementing the MqttClientPubTask thread.
 * @param  argument: Not used
 * @retval None
 */
void MqttClientPubTask(void const *argument)
{
	char str[50];
	MQTTMessage message;

	uint32_t ulNotifiedValue = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = 1000;

	// 1) Check the status of the network link:
	// - If the link is inactive, wait until it becomes active.
	// - If the link is active, proceed to the next step.

	// 2) Check for a valid IP address:
	// - If there is no valid IP address, wait until it is assigned.
	// - If the IP address is valid, proceed to the next step.

	// 3) Attempt to establish the connection with the MQTT broker:
	// - If connection fails, frees resources and returns to network link control (step 1).
	// - If the connection is successful, sends MQTT messages.

	// Note: In case of IP address change, it is necessary to reconnect.

	for(;;)
	{
		// Is link up?
		// Write once link is down
		if(!netif_is_link_up(&gnetif))
		{
			LOG_DEBUG("Link is down\n");
		}
		while(!netif_is_link_up(&gnetif))
		{
			osDelay(250);
		}
		LOG_DEBUG("Link is up\n");

		// Have valid IP?
		LOG_DEBUG("Waiting for valid ip address\n");
		while (gnetif.ip_addr.addr == 0 || gnetif.netmask.addr == 0 || gnetif.gw.addr == 0)
		{
			// System has no valid ip address, wait for 1/4 second
			osDelay(250);
		}

		// IP address is valid, log the details
		LOG_DEBUG("DHCP/Static IP O.K.\n");
		LOG_DEBUG("IP %lu.%lu.%lu.%lu\n\r",(gnetif.ip_addr.addr & 0xff), ((gnetif.ip_addr.addr >> 8) & 0xff), ((gnetif.ip_addr.addr >> 16) & 0xff), (gnetif.ip_addr.addr >> 24));

		// Connect to the broker
		MQTTDisconnect(&mqttClient);
		net_disconnect(&mqttNet);
		net_clear();
		if(MqttConnectBroker() < 0)
		{
			net_disconnect(&mqttNet);
			net_clear();
			continue;
		}

		need_to_reconnect = 0;

		uint8_t error = 0;
		do
		{
			// Send a mqtt message
			// Get the PLC holding register value from the other task
			//ulNotifiedValue = ulTaskNotifyTake(pdTRUE, 500);
			// portMAX_DELAY
			if(xTaskNotifyWait(0, 0, &ulNotifiedValue, 250) != pdTRUE)
			{
				osDelay(250);
				continue;
			}

			// Composing the message to be sent
			snprintf(str, sizeof(str), "MQTT message from STM32: %lu", ulNotifiedValue);
			message.payload = (void*)str;
			message.payloadlen = strlen(str);

			// Send the message at topic "2023/test"
			if(MQTTPublish(&mqttClient, "2023/test", &message) != MQTT_SUCCESS)
			{
				MQTTCloseSession(&mqttClient);
				net_disconnect(&mqttNet);
				error = 1;
				continue;
			}

			LOG_DEBUG("[%lu] I've sent a message!\n", ulNotifiedValue);
			leds_blink_on_mqtt_message_sent();

			// The vTaskDelayUntil() suspend a task for up to an absolute amount of time,
			// ensuring precise periodicity even in the case of interruptions.
			// NOTE: update internally xLastWakeTime with the current time
			vTaskDelayUntil(&xLastWakeTime, xFrequency);

			/* no error and i'm connected and i don't need to reconnect */
		}while(!error && mqttClient.isconnected && !need_to_reconnect);
	}
}


/**
 * @brief  Called when interface is brought up/down or address is changed while up
 * @param  netif: the network interface
 * @retval None
 */
void ethernet_status_updated(struct netif *netif)
{
	// Force a reconnect
	need_to_reconnect = 1;
}


/**
 * @brief Connects to an MQTT broker and subscribes to a topic.
 * @param argument: None
 * @retval MQTT_SUCCESS on success, or an MQTT error code on failure.
 */
int MqttConnectBroker()
{
  int ret;

  // Initialize the network interface
  NewNetwork(&mqttNet);
  net_clear();
  ret = ConnectNetwork(&mqttNet, BROKER_IP, MQTT_PORT);

  // Log heap statistics for debugging.
  LOG_DEBUG("freeHeapSize: %u bytes, minimumEverFreeHeapSize: %u bytes\r\n", (unsigned int)xPortGetFreeHeapSize(), (unsigned int)xPortGetMinimumEverFreeHeapSize());

  HeapStats_t pxHeapStats;
  vPortGetHeapStats( &pxHeapStats );

  if(ret != MQTT_SUCCESS)
  {
	  // Handle network connection failure.
	  LOG_DEBUG("\r\nConnectNetwork failed.\r\n");
	  net_disconnect(&mqttNet);
	  return -1;
  }

  // Initialize the MQTT client
  MQTTClientInit(&mqttClient, &mqttNet, 1000, sndBuffer, sizeof(sndBuffer), rcvBuffer, sizeof(rcvBuffer));

  // Set up MQTT connection parameters
  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  data.willFlag = 0;
  data.MQTTVersion = 3;
  data.clientID.cstring = "STM32F4-NUCLEOBOARD";
  //data.username.cstring = "roger";
  //data.password.cstring = "password";
  data.keepAliveInterval = 60;
  data.cleansession = 1;

  ret = MQTTConnect(&mqttClient, &data);
  if(ret != MQTT_SUCCESS)
  {
	  // Handle MQTT connection failure
	  LOG_DEBUG("MQTTConnect failed.\n");
	  MQTTCloseSession(&mqttClient);
	  net_disconnect(&mqttNet);
	  return ret;
  }

  // Subscribe to the desired topic
  ret = MQTTSubscribe(&mqttClient, "2023/test", QOS0, MqttMessageArrived);
  if(ret != MQTT_SUCCESS)
  {
	  // Handle subscription failure
	  LOG_DEBUG("MQTTSubscribe failed.\n");
	  MQTTCloseSession(&mqttClient);
	  net_disconnect(&mqttNet);
	  return ret;
  }

  LOG_DEBUG("MQTT_ConnectBroker O.K.\n");
  return MQTT_SUCCESS;
}



/**
 * @brief Callback function invoked when an MQTT message arrives.
 * @param argument: None
 * @param msg Pointer to the received message data.
 */
void MqttMessageArrived(MessageData* msg)
{
    MQTTMessage* message = msg->message;

    // Clear the message buffer and copy the received payload into it.
    memset(msgBuffer, 0, sizeof(msgBuffer));
    memcpy(msgBuffer, message->payload, message->payloadlen);

    // Log the received message payload and its length.
    LOG_DEBUG("MQTT MSG[%d]:%s\n", (int)message->payloadlen, msgBuffer);
}

