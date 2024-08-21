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
#include "leds.h"
#include "nanomodbus_interface.h"
#include "iperf_server.h"
#include "mqtt_task.h"
#include "modbus_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODBUS_CLIENT_TASK_STACK_SIZE (2 * 1024)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern struct netif gnetif; //extern gnetif
/* Stack task size*/
static StackType_t modbusClientTask_stack[ MODBUS_CLIENT_TASK_STACK_SIZE / sizeof( StackType_t ) ];
/* Task Control Block */
static StaticTask_t modbusClientTask_tcb;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

// __attribute__((section(".ccmram")))
//__attribute__((section(".ccmram"))) volatile uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];


/* Allocate two blocks of RAM for use by the heap.  The first is a block of
0x10000 bytes starting from address 0x80000000, and the second a block of
0xa0000 bytes starting from address 0x90000000.  The block starting at
0x80000000 has the lower start address so appears in the array fist. */

//#define RAM_REGION_HEAP_SIZE 49152
#define RAM_REGION_HEAP_SIZE 98304

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
  LOG_DEBUG("\r\n!!! stack overflow detected in task: %s !!!\r\n", pcTaskName);
  size_t freeHeapSize = xPortGetFreeHeapSize();
  size_t minimumEverFreeHeapSize = xPortGetMinimumEverFreeHeapSize();
  LOG_DEBUG("freeHeapSize: %u bytes, minimumEverFreeHeapSize: %u bytes\r\n", (unsigned int)freeHeapSize, (unsigned int)minimumEverFreeHeapSize);
  while(1)
  {
	  leds_blink_on_stackoverflow();
  }
}

void vApplicationMallocFailedHook(void)
{
	// configUSE_MALLOC_FAILED_HOOK
	LOG_DEBUG("\r\n!!! Malloc Failed !!!\r\n");
	size_t freeHeapSize = xPortGetFreeHeapSize();
	size_t minimumEverFreeHeapSize = xPortGetMinimumEverFreeHeapSize();
	LOG_DEBUG("freeHeapSize: %u bytes, minimumEverFreeHeapSize: %u bytes\r\n", (unsigned int)freeHeapSize, (unsigned int)minimumEverFreeHeapSize);
	while(1)
	{
		leds_flash_error_on_malloc_failure();
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
void MX_FREERTOS_Init(void)
{
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512 );
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

#ifdef IPERF_SERVER_ENABLED
  LOG_DEBUG("IPERF server enabled\n");
  iperf_server_init();
#endif


  // Stack size calculated with static stack analyzer
  // Publish mqtt task
  osThreadDef(mqttClientPubTask, MqttClientPubTask, osPriorityNormal, 0, (8.5 * 1024) / sizeof( StackType_t ) );
  mqttClientPubTaskHandle = osThreadCreate(osThread(mqttClientPubTask), NULL);

  // Stack size calculated with static stack analyzer
  // Subscribe mqtt task
  osThreadDef(mqttClientSubTask, MqttClientSubTask, osPriorityNormal, 0, (8 * 1024 ) / sizeof( StackType_t ) );
  mqttClientSubTaskHandle = osThreadCreate(osThread(mqttClientSubTask), NULL);


  // Wait the connection with the mqtt broker before attempt the PLC connection.
  if(!mqttClient.isconnected)
  {
	  osDelay(250);
  }

  // To add another task by dynamically allocating its stack, the heap space must be increased.
  // Or you statically allocate the stack.
  // Modbus task
  //osThreadDef(modbusClientTask, ModbusClientTask, osPriorityNormal, 0, (1 * 1024) / sizeof( StackType_t ));
  /* Creazione del task utilizzando l'allocazione statica di FreeRTOS */
  xTaskCreateStatic(
		  ModbusClientTask,       		/* Function that implements the task. */
          "ModbusClientTask",          	/* Text name for the task. */
		  MODBUS_CLIENT_TASK_STACK_SIZE / sizeof( StackType_t ), /* Number of indexes in the xStack array. */
          NULL,                 		/* Parameter passed into the task. */
		  osPriorityNormal,     		/* Priority at which the task is created. */
		  modbusClientTask_stack,       /* Array to use as the task's stack. */
          &modbusClientTask_tcb );  	/* Variable to hold the task's data structure. */


  /* Infinite loop */
  for(;;)
  {
//	  MQTTDisconnect(&mqttClient);
//	  net_disconnect(&mqttNet);
//	  net_clear();
//	  MqttConnectBroker();
//
	  osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */




/* USER CODE END Application */
