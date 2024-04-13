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

TIM_OC_InitTypeDef sConfigOC = {0};

void init_timer3();

void pass_on_timer_info(TIM_HandleTypeDef* timer_address, U32 channel1){
	DRS_Timer = timer_address;
	DRS_Channel = channel1;
	init_timer3();

}

void init_timer3(){
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_TIM_Base_Start_IT(DRS_Timer);
	HAL_TIM_OC_Start_IT(DRS_Timer, TIM_CHANNEL_3);

	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
}
U8 drs_button_state = 0;
U8 rot_dial = 3;

uint16_t openn = 25919;
uint16_t closed = 18959;
void set_DRS_Servo_Position(){
	//duty cycle lookup table for each DRS position
	//static U8 rot_dial = 3;//change macro to actual g-can variable
	static U16 rot_dial_timer_val = 0;
	static int DRS_POS_LUT[] = {DRS_POS_0, DRS_POS_1, DRS_POS_2, DRS_POS_3, DRS_POS_4,
	                            DRS_POS_5, DRS_POS_6, DRS_POS_7, DRS_POS_8, DRS_POS_9,
	                            DRS_POS_10, DRS_POS_11, DRS_POS_12, DRS_POS_13,
	                            DRS_POS_14, DRS_POS_15};
	//add rot_dial = gopher can variable
	rot_dial_timer_val = DRS_POS_LUT[rot_dial];
    if (drs_button_state == 1){
    	sConfigOC.Pulse = rot_dial_timer_val;
    	HAL_TIM_PWM_ConfigChannel(DRS_Timer, &sConfigOC, DRS_Channel);
    	HAL_TIM_OC_Start(DRS_Timer, DRS_Channel);
    }
    else{
    	//sConfigOC.Pulse = 1;
    	//HAL_TIM_PWM_ConfigChannel(DRS_Timer, &sConfigOC, DRS_Channel);
    	HAL_TIM_PWM_ConfigChannel(DRS_Timer, &sConfigOC, DRS_Channel);
    	HAL_TIM_OC_Stop(DRS_Timer,DRS_Channel); //is it better to just leave it on with duty 0?
    	HAL_GPIO_WritePin(DRS_PWM_GPIO_Port, DRS_PWM_Pin, 0);

    }
}
