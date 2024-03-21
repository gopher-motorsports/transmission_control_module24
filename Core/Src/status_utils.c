/*
 * status_utils.c
 *
 *  Created on: Mar 20, 2024
 *      Author: chris
 */

#include "status_utils.h"
#include "main_task.h"
#include "main.h"
#include "gopher_sense.h"
#include "pulse_sensor.h"
#include "utils.h"
#include <stdio.h>
#include <stdbool.h>
#include "shift_parameters.h"
#include <string.h>
#include <stdbool.h>
#include <cmsis_os.h>


// Array for the amount of time each error should latch the
U16 error_led_on_times[] = {
	OVERCURRENT_FAULT_LED_MS, // SENSE_OUT_OVERCURRENT_3V3
	OVERCURRENT_FAULT_LED_MS, // SENSE_OUT_OVERCURRENT_5V
	SHIFT_SAFETY_FAULT_LED_MS,	//
	SHIFT_SAFETY_FAULT_LED_MS,	//
	SHIFT_TIMEOUT_FAULT_LED_MS,
	SHIFT_TIMEOUT_FAULT_LED_MS,
	0,
	0
};


void update_task_utilization(){
	static uint32_t lastTaskUtilizationUpdate = 0;
		if (HAL_GetTick() -  lastTaskUtilizationUpdate > 1000){
			lastTaskUtilizationUpdate = HAL_GetTick();
		}
}

void send_uart_tick(){
	static U32 lastPrintHB = 0;
	if (HAL_GetTick() - lastPrintHB >= PRINTF_HB_MS_BETWEEN)
		{
			printf("Current tick: %lu\n", HAL_GetTick());
			lastPrintHB = HAL_GetTick();
		}
}

void toggle_heart_beat(){
	static U32 lastHeartbeat= 0;
	if (HAL_GetTick() - lastHeartbeat > HEARTBEAT_MS_BETWEEN)
		{
			lastHeartbeat = HAL_GetTick();
			HAL_GPIO_TogglePin(HBEAT_GPIO_Port, HBEAT_Pin);
		}
}




void checkForErrors(void) {
	static U32 led_on_start_time = 0;
	static U32 time_on_ms = 0;
	static bool led_on = false;

	// Can expand into for loop if more pins to read from in the future, but for now just check the 2 fault input pins.
	if (!HAL_GPIO_ReadPin(SWITCH_FAULT_3V3_GPIO_Port, SWITCH_FAULT_3V3_Pin)) {
		error(SENSE_OUT_OVERCURRENT_3V3, &error_byte);
	}

	if (!HAL_GPIO_ReadPin(SWITCH_FAULT_5V_GPIO_Port, SWITCH_FAULT_5V_Pin)) {
		error(SENSE_OUT_OVERCURRENT_5V, &error_byte);
	}

	if (error_byte > 0 && !led_on) {
		U8 index = 0;
		// Get the index of the lowest bit. Cannot become infinite because we're only here if error_byte >0
		while (!(error_byte & (1 << index))) index++;

		time_on_ms = error_led_on_times[index];

		led_on_start_time = HAL_GetTick();
		led_on = true;
	}

	if(led_on) {
		if (HAL_GetTick() - led_on_start_time >= time_on_ms) {
			HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, 0);
			led_on = false;
		} else {
			HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, 1);
		}
	}
}


// can_callback_function example

// change_led_state
//  a custom function that will change the state of the LED specified
//  by parameter to remote_param. In this case parameter is a U16*, but
//  any data type can be pointed to, as long as it is configured and casted
//  correctly
void change_led_state(U8 sender, void* parameter, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
	//HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, !!remote_param);
	return;
}


// init_error
//  This function will stay in an infinite loop, blinking the LED in a 0.5sec period. Should only
//  be called from the init function before the RTOS starts
void init_error(void)
{
	while (1)
	{
		HAL_GPIO_TogglePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin);
		HAL_Delay(250);
	}
}
