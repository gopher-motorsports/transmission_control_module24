/*
 * drs.c
 *
 *  Created on: Mar 21, 2024
 *      Author: chris
 */
#include "drs.h"
#include "main_task.h"
#include "main.h"
#include "gopher_sense.h"
#include "pulse_sensor.h"
#include "utils.h"
#include "shift_parameters.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdbool.h>
#include <cmsis_os.h>

TIM_HandleTypeDef* DRS_Timer;
U32 DRS_Channel;
int rot_dial_timer_val = 0; //keeping this in here if we want to use rotary dial
U8 drs_button_state;

void init_DRS_servo(TIM_HandleTypeDef* timer_address, U32 channel){
	DRS_Timer = timer_address;
	DRS_Channel = channel;
	HAL_TIM_PWM_Start(DRS_Timer, DRS_Channel); //turn on PWM generation
	HAL_GPIO_WritePin(DRS_POWER_EN_GPIO_Port, DRS_POWER_EN_Pin, 1); //turn DRS servo on
}

void set_DRS_Servo_Position(){
	//duty cycle lookup table for each DRS position, optional if we are using the rotary dial
	static int DRS_POS_LUT[] = {DRS_POS_0, DRS_POS_1, DRS_POS_2, DRS_POS_3, DRS_POS_4,
	                            DRS_POS_5, DRS_POS_6, DRS_POS_7, DRS_POS_8, DRS_POS_9,
	                            DRS_POS_10, DRS_POS_11, DRS_POS_12, DRS_POS_13,
	                            DRS_POS_14, DRS_POS_15};
	rot_dial_timer_val = DRS_POS_LUT[swDial_a_ul.data]; //instead of setting open or closed we could set to a specific location


	drs_button_state = swButon3_state.data;; //place holder button

	if(drs_button_state == 1){
		__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
	}
	else{
		__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, CLOSED_POS);
	}
}

