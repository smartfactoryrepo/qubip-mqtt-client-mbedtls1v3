/*
 * modbus_task.c
 *
 *  Created on: Aug 21, 2024
 *      Author: federico
 */
/* Private includes ----------------------------------------------------------*/
#include "modbus_task.h"
#include "mqtt_task.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "limits.h"
#include "lwip.h"

#include "MQTTInterface.h"
#include "leds.h"
#include "nanomodbus_interface.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern struct netif gnetif; //extern gnetif

/* Private function prototypes -----------------------------------------------*/

/* Private application code --------------------------------------------------*/
/**
 * @brief  Function implementing the ModbusClientTask thread.
 * @param  argument: Not used
 * @retval None
 */
void ModbusClientTask(void *argument)
{
	int fd = -1;
	nmbs_t nmbs;


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

		// Wait until connection with MQTT broker is established
		while(!mqttClient.isconnected)
		{
			osDelay(250);
		}

		// Setup Modbus library and connect to PLC
		if(nmbs_platform_setup(&fd, &nmbs) == -1)
		{
			// Error
			LOG_DEBUG("Error in nmbs_platform_setup, I'll try again in 1 second \n");
			osDelay(1000);
			continue;
		}

		// Connected to PLC via Modbus TCP!
		LOG_DEBUG("Connected to plc!\n");
		nmbs_error err = NMBS_ERROR_NONE;

		do
		{
		    // Read holding registers
		    uint16_t r_regs[1] = { 0 };
		    err = nmbs_read_holding_registers(&nmbs, MODBUS_PLC_REGISTER, 1, r_regs);
		    if (err != NMBS_ERROR_NONE)
		    {
		        LOG_DEBUG("Error reading holding register at address %d - %s\n", MODBUS_PLC_REGISTER, nmbs_strerror(err));
		        if (!nmbs_error_is_exception(err))
		        {
		        	LOG_DEBUG("Exception occurred in nmbs_read_holding_registers\n");
		        	continue;
		        }
		    }
		    else
		    {
		    	LOG_DEBUG("Register at address %d: %d\n", MODBUS_PLC_REGISTER, r_regs[0]);
		    }

		    // Send the new received data to the mqtt publisher task
		    xTaskNotify(mqttClientPubTaskHandle, r_regs[0], eSetValueWithOverwrite);

		    // Write holding register. Cycle from 0 to 65535.
		    uint16_t w_regs[1] = { (r_regs[0] >= INT_MAX) ? 0 : ++r_regs[0] };
		    err = nmbs_write_multiple_registers(&nmbs, MODBUS_PLC_REGISTER, 1, w_regs);
		    if (err != NMBS_ERROR_NONE)
		    {
		    	LOG_DEBUG("Error writing register at address %d - %s", MODBUS_PLC_REGISTER, nmbs_strerror(err));
		        if (!nmbs_error_is_exception(err))
		        {
		        	LOG_DEBUG("Exception occurred in nmbs_write_multiple_registers\n");
		        	continue;
		        }
		    }

			osDelay(1000);

		// If an error occur or connection with MQTT broker is lost, restart from the beginning
		}while(err == NMBS_ERROR_NONE && mqttClient.isconnected);
	}
}
