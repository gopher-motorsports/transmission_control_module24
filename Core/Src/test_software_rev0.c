/*
 * test_soft_rev0.c
 *
 *  Created on: Nov 30, 2023
 *      Author: adamjensen
 */


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
#define HEARTBEAT_MS_BETWEEN 500


//input state variables
uint8_t power_3v3_fault_state = 0;
uint8_t power_5V_fault_state = 0;


//transmission side output state variables
uint8_t last_trans_out_state_all_on = 0;
uint8_t trans_out_all_on = 0;
uint8_t fault_led_state = 0;
uint8_t upshift_state = 0;
uint8_t downshift_state = 0;
uint8_t fast_clutch_state = 0;
uint8_t slow_clutch_state = 0;
uint8_t drs_state = 0;
uint8_t extra_state = 0;

//pulse sensor data
float easy_access_trans_speed;

//car side output
uint8_t spk_out_state = 0;
uint8_t aux1_c_state = 0;
uint8_t aux2_c_pass_state = 0;



void test_all_loop(void){
	//heartbeat
	static U32 lastHeartbeat = 0;

	if (HAL_GetTick() - lastHeartbeat > HEARTBEAT_MS_BETWEEN)
		{
			lastHeartbeat = HAL_GetTick();
			HAL_GPIO_TogglePin(HBEAT_GPIO_Port, HBEAT_Pin);
		}

	//MCU specific outputs
	HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, fault_led_state);

	//inputs
	power_3v3_fault_state = HAL_GPIO_ReadPin(SWITCH_FAULT_3V3_GPIO_Port, SWITCH_FAULT_3V3_Pin);
	power_5V_fault_state = HAL_GPIO_ReadPin(SWITCH_FAULT_5V0_GPIO_Port, SWITCH_FAULT_5V_Pin);

	//outputs transmisison side
	HAL_GPIO_WritePin(UPSHIFT_OUT_GPIO_Port, UPSHIFT_OUT_Pin, upshift_state);
	HAL_GPIO_WritePin(DOWNSHIFT_OUT_GPIO_Port, DOWNSHIFT_OUT_Pin, downshift_state);
	HAL_GPIO_WritePin(FAST_CLUTCH_OUT_GPIO_Port, FAST_CLUTCH_OUT_Pin, fast_clutch_state);
	HAL_GPIO_WritePin(SLOW_CLUTCH_OUT_GPIO_Port, SLOW_CLUTCH_OUT_Pin, slow_clutch_state);
	HAL_GPIO_WritePin(DRS_OUT_GPIO_Port, DRS_OUT_Pin, drs_state);
	HAL_GPIO_WritePin(EXTRA_OUT_GPIO_Port, EXTRA_OUT_Pin, extra_state);


	//outputs car side
	HAL_GPIO_WritePin(SPK_CUT_OUT_GPIO_Port, SPK_CUT_OUT_Pin, spk_out_state);
	HAL_GPIO_WritePin(AUX1_C_OUT_GPIO_Port, AUX1_C_OUT_Pin, aux1_c_state);
	HAL_GPIO_WritePin(AUX2_C_PASS_GPIO_Port, AUX2_C_PASS_Pin, aux2_c_pass_state);

	//pulse sensor
	check_pulse_sensors();
	easy_access_trans_speed = tcm_data.trans_speed;

}
