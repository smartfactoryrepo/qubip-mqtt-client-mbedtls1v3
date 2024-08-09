/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "lwip.h"
#include "lwip/api.h"
#include "MQTTClient.h"
#include "MQTTInterface.h"
#include "rng.h"
#include "platform.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TLS V1.2
#if defined(TLS_1V2) && !defined(TLS_1V3)
#define MQTT_PORT		"1885"
#endif
// TLS V1.3
#if !defined(TLS_1V2) && defined(TLS_1V3)
#define MQTT_PORT		"1883"
#endif

#define BROKER_IP		"192.168.101.63"
#define MQTT_BUFSIZE	1024

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern struct netif gnetif; //extern gnetif

osThreadId mqttClientSubTaskHandle;  //mqtt client task handle
osThreadId mqttClientPubTaskHandle;  //mqtt client task handle

Network net; //mqtt network
MQTTClient mqttClient; //mqtt client

uint8_t sndBuffer[MQTT_BUFSIZE]; //mqtt send buffer
uint8_t rcvBuffer[MQTT_BUFSIZE]; //mqtt receive buffer
uint8_t msgBuffer[MQTT_BUFSIZE]; //mqtt message buffer

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

// __attribute__((section(".ccmram")))
//__attribute__((section(".ccmram"))) volatile uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];


/* Allocate two blocks of RAM for use by the heap.  The first is a block of
0x10000 bytes starting from address 0x80000000, and the second a block of
0xa0000 bytes starting from address 0x90000000.  The block starting at
0x80000000 has the lower start address so appears in the array fist. */

#define RAM_REGION_HEAP_SIZE 49152

volatile uint8_t heap[RAM_REGION_HEAP_SIZE] = { 0 }; // 192kb / 4

const HeapRegion_t xHeapRegions[] =
{
	// Point to ccmram
    { ( uint8_t * ) 0x10000000UL, 0xFFFE },
	{ ( uint8_t * ) &heap, RAM_REGION_HEAP_SIZE },
    { NULL, 0 } /* Terminates the array. */
};



/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MqttClientSubTask(void const *argument); //mqtt client subscribe task function
void MqttClientPubTask(void const *argument); //mqtt client publish task function
int  MqttConnectBroker(void); 				//mqtt broker connect function
void MqttMessageArrived(MessageData* msg); //mqtt message callback function

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
/* vApplicationGetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); //turn on red led when detects stack overflow
  LOG_DEBUG("\r\n!!! stack overflow detected in task: %s !!!\r\n", pcTaskName);
  size_t freeHeapSize = xPortGetFreeHeapSize();
  size_t minimumEverFreeHeapSize = xPortGetMinimumEverFreeHeapSize();
  LOG_DEBUG("freeHeapSize: %u bytes, minimumEverFreeHeapSize: %u bytes\r\n", (unsigned int)freeHeapSize, (unsigned int)minimumEverFreeHeapSize);
  while(1)
  {
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	  HAL_Delay(125);
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	  HAL_Delay(125);
  }
}

void vApplicationMallocFailedHook(void)
{
	// configUSE_MALLOC_FAILED_HOOK
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	LOG_DEBUG("\r\n!!! Malloc Failed !!!\r\n");
	size_t freeHeapSize = xPortGetFreeHeapSize();
	size_t minimumEverFreeHeapSize = xPortGetMinimumEverFreeHeapSize();
	LOG_DEBUG("freeHeapSize: %u bytes, minimumEverFreeHeapSize: %u bytes\r\n", (unsigned int)freeHeapSize, (unsigned int)minimumEverFreeHeapSize);
	while(1)
	{
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
	}
}

void vApplicationGetRandomHeapCanary( portPOINTER_SIZE_TYPE * pxHeapCanary )
{
	uint32_t randomValue = 0;
	if(HAL_RNG_GenerateRandomNumber(&hrng, &randomValue) != HAL_OK)
	{
		randomValue = ((HAL_GetTick() * HAL_GetUIDw0()) + HAL_GetUIDw1()) ^ HAL_GetUIDw2();
		LOG_DEBUG("RNG failed, manual rng num randomValue: %u\r\n", (unsigned int)randomValue);
	}
	*pxHeapCanary = randomValue;
}


/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
   implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
   used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    /* If the buffers to be provided to the Idle task are declared inside this
       function then they must be declared static - otherwise they will be allocated on
       the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
       state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
       Note that, as the array is necessarily of type StackType_t,
       configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}

/*-----------------------------------------------------------*/

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
   application must provide an implementation of vApplicationGetTimerTaskMemory()
   to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
    /* If the buffers to be provided to the Timer task are declared inside this
       function then they must be declared static - otherwise they will be allocated on
       the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
       task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
       Note that, as the array is necessarily of type StackType_t,
      configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}


void* my_calloc(size_t nmemb, size_t size)
{
	void *ptr = pvPortCalloc(nmemb, size);
	if (ptr == NULL)
	{
		LOG_DEBUG("Failed to allocate %u bytes\r\n", (unsigned int)(nmemb * size));
	}
	return ptr;
}

void my_free(void *ptr)
{
	if (ptr != NULL)
	{
	    vPortFree(ptr);
	}
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* Pass the array into vPortDefineHeapRegions(). */
  int ret = mbedtls_platform_set_calloc_free(my_calloc, my_free);
  if (ret != 0)
  {
	  // Gestione dell'errore
	  LOG_DEBUG("\r\nmbedtls_platform_set_calloc_free failed\r\n");
  }
  vPortDefineHeapRegions( xHeapRegions );
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  osThreadDef(mqttClientSubTask, MqttClientSubTask, osPriorityNormal, 0, (8.5 * 1024 ) / 4 ); //subscribe task
  osThreadDef(mqttClientPubTask, MqttClientPubTask, osPriorityNormal, 0, (8 * 1024) / 4 ); //publish task
  mqttClientSubTaskHandle = osThreadCreate(osThread(mqttClientSubTask), NULL);
  mqttClientPubTaskHandle = osThreadCreate(osThread(mqttClientPubTask), NULL);

  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void MqttClientSubTask(void const *argument)
{
  while(1)
  {
    //waiting for valid ip address
    if (gnetif.ip_addr.addr == 0 || gnetif.netmask.addr == 0 || gnetif.gw.addr == 0) //system has no valid ip address
    {
      osDelay(1000);
      continue;
    }
    else
    {
      LOG_DEBUG("DHCP/Static IP O.K.\n");
      const uint32_t local_IP = gnetif.ip_addr.addr;
      LOG_DEBUG("IP %lu.%lu.%lu.%lu\n\r",(local_IP & 0xff), ((local_IP >> 8) & 0xff), ((local_IP >> 16) & 0xff), (local_IP >> 24));
      break;
    }
  }

  while(1)
  {
    if(!mqttClient.isconnected)
    {
      //try to connect to the broker
      MQTTDisconnect(&mqttClient);
      MqttConnectBroker();
      osDelay(1000);
    }
    else
    {
      MQTTYield(&mqttClient, 1000); //handle timer
      osDelay(100);
    }
  }
}

void MqttClientPubTask(void const *argument)
{
  char str[50];
  //const char* str = "MQTT message from STM32";
  MQTTMessage message;
  uint32_t n = 0;

  while(1)
  {
    if(mqttClient.isconnected)
    {
      ++n;
      snprintf(str, sizeof(str), "MQTT message from STM32: %lu", n);
      message.payload = (void*)str;
      message.payloadlen = strlen(str);

      if(MQTTPublish(&mqttClient, "2023/test", &message) != MQTT_SUCCESS)
      {
        MQTTCloseSession(&mqttClient);
        net_disconnect(&net);
      }
      LOG_DEBUG("[%lu] Ho inviato un messaggio!\n", n);
      osDelay(1000);
    }
    else
    {
    	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    	osDelay(250);
    	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
    	osDelay(250);
    }
  }
}

int MqttConnectBroker()
{
  int ret;

  NewNetwork(&net);
  net_clear();
  ret = ConnectNetwork(&net, BROKER_IP, MQTT_PORT);

  size_t freeHeapSize = xPortGetFreeHeapSize();
  size_t minimumEverFreeHeapSize = xPortGetMinimumEverFreeHeapSize();
  LOG_DEBUG("freeHeapSize: %u bytes, minimumEverFreeHeapSize: %u bytes\r\n", (unsigned int)freeHeapSize, (unsigned int)minimumEverFreeHeapSize);

  HeapStats_t pxHeapStats;
  vPortGetHeapStats( &pxHeapStats );

  if(ret != MQTT_SUCCESS)
  {
    LOG_DEBUG("\r\nConnectNetwork failed.\r\n");
    net_disconnect(&net);
    return -1;
  }

  MQTTClientInit(&mqttClient, &net, 1000, sndBuffer, sizeof(sndBuffer), rcvBuffer, sizeof(rcvBuffer));

  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  data.willFlag = 0;
  data.MQTTVersion = 3;
  data.clientID.cstring = "STM32F4-cazzuta";
  //data.username.cstring = "roger";
  //data.password.cstring = "password";
  data.keepAliveInterval = 60;
  data.cleansession = 1;

  ret = MQTTConnect(&mqttClient, &data);
  if(ret != MQTT_SUCCESS)
  {
    LOG_DEBUG("MQTTConnect failed.\n");
    MQTTCloseSession(&mqttClient);
    net_disconnect(&net);
    return ret;
  }

  ret = MQTTSubscribe(&mqttClient, "2023/test", QOS0, MqttMessageArrived);
  if(ret != MQTT_SUCCESS)
  {
    LOG_DEBUG("MQTTSubscribe failed.\n");
    MQTTCloseSession(&mqttClient);
    net_disconnect(&net);
    return ret;
  }

  LOG_DEBUG("MQTT_ConnectBroker O.K.\n");
  return MQTT_SUCCESS;
}

void MqttMessageArrived(MessageData* msg)
{
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //toggle pin when new message arrived

  MQTTMessage* message = msg->message;
  memset(msgBuffer, 0, sizeof(msgBuffer));
  memcpy(msgBuffer, message->payload,message->payloadlen);

  LOG_DEBUG("MQTT MSG[%d]:%s\n", (int)message->payloadlen, msgBuffer);
}

/* USER CODE END Application */
