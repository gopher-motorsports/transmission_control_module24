/*
 * auto_shift.c
 *
 *  Created on: May 5, 2024
 *      Author: chris
 */

#include "main_task.h"
#include "main.h"
#include "gopher_sense.h"
#include "pulse_sensor.h"
#include "debug_defines.h"
#include "shift_parameters.h"
#include "utils.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <cmsis_os.h>

#define SHIFT_1_RPM 9000
#define SHIFT_2_RPM 11000
#define SHIFT_3_RPM 11500
#define SHIFT_4_RPM 10000
#define SHIFT_5_RPM 9000
#define MIN_CORRECT_RPM_TIME_MS 50
#define SHIFT_COOLDOWN_TIME 500

int Autoshift_array[5] = {
	SHIFT_1_RPM,
	SHIFT_2_RPM,
	SHIFT_3_RPM,
	SHIFT_4_RPM,
	SHIFT_5_RPM
};

void autoshift_task() {
static int current_shift = 0;
static bool already_entered = false;
static long start_time = 0;
static long cooldown_start_time = 0;
static U16 initial_auto_shift_gear = NEUTRAL;


if ((HAL_GetTick() - cooldown_start_time > SHIFT_COOLDOWN_TIME) && (tcm_data.current_RPM > Autoshift_array[current_shift])) {
	if (!already_entered) {
		start_time = HAL_GetTick();
		already_entered = true;
		return;
	}
	if (HAL_GetTick() - start_time > MIN_CORRECT_RPM_TIME_MS) {
		tcm_data.num_shifts += 1;
		already_entered = false;
		cooldown_start_time = HAL_GetTick();

#ifdef NO_GEAR_POT
			current_shift++;
#else
		if(tcm_data.current_gear > initial_auto_shift_gear + 2){
			current_shift++;
			tcm_data.successful_shift = tcm_data.current_gear >= initial_auto_shift_gear + 2;
			tcm_data.gear_established = tcm_data.successful_shift;
		} else{return;}
#endif
	}
} else {
	already_entered = false;
}

/*
if (!tcm_data.time_shift_only && (tcm.current_gear > initial_auto_shift_gear + 2)){
	initial_auto_shift_gear = initial_auto_shift_gear + 2;
	current_shift++;
}*/

}


/*
void run_auto_sm(void){
	switch (auto_shift_state){
	case ERROR_GEAR
	case NEUTRAL_AUTO_STATE:
		break;
	}
	case GEAR_1_AUTO_STATE:
		break;



}*/
