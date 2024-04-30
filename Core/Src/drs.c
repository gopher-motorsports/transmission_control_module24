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
U8 drs_steering_angle_limit_state = 0; //SA = steering angle
U8 next_steering_angle_limit_state = 0;

POWER_CHANNEL drs_power_channel = {
    .parameter = &channelCurrentDRS_A,
    .enable_switch_port = DRS_POWER_EN_GPIO_Port,
    .enable_switch_pin = DRS_POWER_EN_Pin,
    .enabled = 0,
    .amp_max = 10.0f,
    .ampsec_max = 8.0f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 2000,
    .last_update = 0,
	.overcurrent_count = 0,
	.max_overcurrent_count = 5
};

//local function prototypes
bool drs_shutoff_conditions_reached();
void update_power_channel(POWER_CHANNEL* channel);

void init_DRS_servo(TIM_HandleTypeDef* timer_address, U32 channel){
	DRS_Timer = timer_address;
	DRS_Channel = channel;
	HAL_TIM_PWM_Start(DRS_Timer, DRS_Channel); //turn on PWM generation
	HAL_GPIO_WritePin(DRS_POWER_EN_GPIO_Port, DRS_POWER_EN_Pin, GPIO_PIN_SET); //turn DRS servo on
	drs_power_channel.enabled = 1; //update drs_power_channel to enabled
}


void update_power_channel(POWER_CHANNEL* channel){
    uint32_t tick = HAL_GetTick();
    uint32_t elapsed_ms = tick - channel->last_update;
    channel->last_update = tick;
    float delta_max = channel->parameter->data - channel->amp_max;

    // integrate Amps*sec
    channel->ampsec_sum += delta_max * (elapsed_ms / 1000.0);

    // bound current integral above 0
    if (channel->ampsec_sum <= 0) channel->ampsec_sum = 0;
}

void check_DRS_current(POWER_CHANNEL* channel){
	update_power_channel(channel);

	if (channel->ampsec_sum > channel->ampsec_max && channel->enabled) {
		// channel has reached Amp*sec threshold, open switch
		channel->trip_time = HAL_GetTick();
		channel->enabled = 0;
		channel->overcurrent_count++;
		HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_RESET);
	} else if (!channel->enabled) {
		// check if it's time to re-enable this channel
		uint32_t ms_since_trip = HAL_GetTick() - channel->trip_time;
		if (ms_since_trip >= channel->reset_delay_ms && (channel->overcurrent_count < channel->max_overcurrent_count)) {
			channel->ampsec_sum = 0;
			channel->enabled = 1;
			HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_SET);
		} else if (channel->overcurrent_count > channel->max_overcurrent_count) {
			HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_RESET);
		}
	}
}

void set_DRS_Servo_Position(){
	//duty cycle lookup table for each DRS position, optional if we are using the rotary dial
	static int DRS_POS_LUT[] = {DRS_POS_0, DRS_POS_1, DRS_POS_2, DRS_POS_3, DRS_POS_4,
	                            DRS_POS_5, DRS_POS_6, DRS_POS_7, DRS_POS_8, DRS_POS_9,
	                            DRS_POS_10, DRS_POS_11, DRS_POS_12, DRS_POS_13,
	                            DRS_POS_14, DRS_POS_15};
	rot_dial_timer_val = DRS_POS_LUT[swDial_a_ul.data]; //instead of setting open or closed we could set to a specific location


	drs_button_state = swButon3_state.data; //place holder button
	check_DRS_current(&drs_power_channel);
	if(drs_button_state == 1){
		if(drs_shutoff_conditions_reached()){
			__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, CLOSED_POS);
		}
		else{
		__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
		}
	}
	else{
		__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, CLOSED_POS);
	}

	drs_steering_angle_limit_state = next_steering_angle_limit_state;
}

bool drs_shutoff_conditions_reached(){
	static long current_tick = 0;
	current_tick = HAL_GetTick();

	//if you haven't received brake or steering angle data in .2 seconds don't close drs
	if(current_tick - brakePressureFront_psi.info.last_rx < CAN_VALUE_TRUST_THRESHOLD ||
	   current_tick - brakePressureRear_psi.info.last_rx  < CAN_VALUE_TRUST_THRESHOLD){
	   if(brakePressureFront_psi.data > BRAKE_SHUTOFF_THRESHOLD ||
	   	   brakePressureRear_psi.data > BRAKE_SHUTOFF_THRESHOLD){
	   		return true;
	   	}
	}

	if(current_tick - steeringAngle_deg.info.last_rx < CAN_VALUE_TRUST_THRESHOLD){
		switch (drs_steering_angle_limit_state)
			{
			case NORMAL_STATE:
				if(steeringAngle_deg.data < STEERING_ANGLE_LEFT_SHUTOFF){
					next_steering_angle_limit_state = LEFT_LIMIT_BREACHED;
					return true;
				}

				if(steeringAngle_deg.data > STEERING_ANGLE_RIGHT_SHUTOFF){
					next_steering_angle_limit_state = RIGHT_LIMIT_BREACHED;
					return true;
				}
				else{return false;}
				break;


			case LEFT_LIMIT_BREACHED:
				if(steeringAngle_deg.data > STEERING_ANGLE_LEFT_RETURN){
					next_steering_angle_limit_state = NORMAL_STATE;
					return false;
				}
				else{return true;}
				break;

			case RIGHT_LIMIT_BREACHED:
				if(steeringAngle_deg.data < STEERING_ANGLE_RIGHT_RETURN){
					next_steering_angle_limit_state = NORMAL_STATE;
					return false;
				}
				else{return true;}
				break;
		}
	}
	//reaches here if all of the data is not being updated on the can bus
	return false;

}

