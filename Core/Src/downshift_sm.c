/*
 * downshift_sm.c
 *
 *  Created on: Mar 20, 2024
 *      Author: chris
 */
#include "downshift_sm.h"
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

// run_downshift_sm
//  The big boi downshift state machine
void run_downshift_sm(void)
{
	static uint32_t initial_gear;
	static uint32_t begin_shift_tick;
	static uint32_t begin_exit_gear_tick;
	static uint32_t begin_enter_gear_tick;
	static uint32_t begin_extra_push_time_tick;
	static uint32_t begin_hold_clutch_tick;

	// calculate the target rpm at the start of each cycle
	tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);

	switch (downshift_state)
	{
	case ST_D_BEGIN_SHIFT: // **********************************************
		// at the beginning of a shift reset all of the variables and start
		// pushing on the clutch and downshift solenoid. Loading the shift
		// lever does not seem to be as important for downshifts, but still
		// give some time for it
		begin_shift_tick = HAL_GetTick();

#ifdef NO_GEAR_POT
		tcm_data.current_gear = get_current_gear();
#endif
		initial_gear = tcm_data.current_gear;
		tcm_data.successful_shift = false;

		set_downshift_solenoid(SOLENOID_ON);

		tcm_data.using_clutch = !tcm_data.clutchless_downshift; // EXPIREMENTAL: Uncomment the next line to only clutch during a downshift if the clutch is held during the start of the shift
		//tcm_data.using_clutch = (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button);

		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);

		// move on to loading the shift lever
		next_downshift_state = ST_D_LOAD_SHIFT_LVR;

#ifdef SHIFT_DEBUG
		// Debug
		printf("=== Downshift State: LOAD_SHIFT_LVR\n");
		printf("How: Completed begin shift steps\n");
		printf("Current Tick: %lu\n", HAL_GetTick());
		printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
		lastShiftingChangeTick = HAL_GetTick();
#endif
		break;

	case ST_D_LOAD_SHIFT_LVR: // **********************************************
		// load the shift lever. This is less important as we do not seem to
		// have issues leaving the gear during a downshift
		set_downshift_solenoid(SOLENOID_ON);
		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);

		// EXPIREMENTAL: spark cut during this preload time and tell drivers
		// to blip when they start the shift. This will allow drivers to worry
		// less about timing their blips perfectly because the TCM will do it
		safe_spark_cut(true);

		if ((HAL_GetTick() - begin_shift_tick > DOWNSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS(shift_mode_2)))
		{
			// done with preloading. Start allowing blips and move on to trying
			// to exit the gear
			safe_spark_cut(false);
			begin_exit_gear_tick = HAL_GetTick();
			next_downshift_state = ST_D_EXIT_GEAR;

#ifdef SHIFT_DEBUG
			// Debug
			printf("=== Downshift State: EXIT_GEAR\n");
			printf("How: Preloading time completed\n");
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();
#endif
		}
		break;

	case ST_D_EXIT_GEAR: // **********************************************
		// this is the region to blip in to leave the gear. Usually we dont
		// have much of an issue leaving. Dont spark cut under any circumstances
		// as we need the blip to bring the RPM up if we have not left the gear
		// yet
		set_downshift_solenoid(SOLENOID_ON);
		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);
		safe_spark_cut(false);

		if (!tcm_data.time_shift_only)
		{
#ifdef NO_GEAR_POT
			// wait for the shift lever to be below the downshift exit threshold
			if (get_shift_pot_pos() < DOWNSHIFT_EXIT_POS_MM)
			{
#else
			// Check if we've moved out of the initial gear and are now in between in the correct direction
			if (tcm_data.current_gear < initial_gear)
			{
#endif
				// we have left the last gear and are in a false neutral. Move on to
				// the next part of the shift
				next_downshift_state = ST_D_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Downshift State: ENTER_GEAR\n");
#ifdef NO_GEAR_POT
				printf("Shift Lever below Downshift Exit Threshold\n");
#else
				printf("Gear state moved below initial gear\n");
#endif
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
				break;
			}

			// check if this state has timed out
			if (HAL_GetTick() - begin_exit_gear_tick > DOWNSHIFT_EXIT_TIMEOUT_MS(shift_mode_2))
			{
				error(SHIFT_STATE_TIMEOUT, &error_byte);

				// We could not release the gear for some reason. Keep trying anyway
				next_downshift_state = ST_D_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

				// if we were not using the clutch before, start using it now because
				// otherwise we're probably going to fail the shift. Don't use if we're
				// using clutchless downshift though
				tcm_data.using_clutch = !tcm_data.clutchless_downshift;

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Downshift State: ENTER_GEAR\n");
				printf("How: Last State Timeout - Use Clutch\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		else
		{
			if (HAL_GetTick() - begin_exit_gear_tick > DOWNSHIFT_EXIT_GEAR_TIME_MS(shift_mode_2)) {
				next_downshift_state = ST_D_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Time Shift Only Downshift State: ENTER_GEAR\n");
				printf("How: Exit Gear time completed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		break;

	case ST_D_ENTER_GEAR: // **********************************************
		// now we are in a false neutral position we want to keep pushing the
		// shift solenoid, but now dynamically spark cut if the blip is too big
		// and the RPM goes too high to enter the next gear
		set_downshift_solenoid(SOLENOID_ON);
		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);

		if (!tcm_data.time_shift_only)
		{
			// Spark cut to make sure the driver is reaching the right rpm while blipping
			reach_target_RPM_spark_cut(tcm_data.target_RPM);

#ifdef NO_GEAR_POT
			// check if the shift position has moved enough to consider the shift
			// finished
			if (get_shift_pot_pos() < DOWNSHIFT_ENTER_POS_MM)
			{
#else
			if (tcm_data.current_gear == (initial_gear - 2))
			{
#endif
				// the clutch lever has moved enough to finish the shift. Turn off
				// any spark cutting and move on to finishing the shift
				safe_spark_cut(false);
				begin_extra_push_time_tick = HAL_GetTick();
				next_downshift_state = ST_D_EXTRA_PUSH;

#ifdef SHIFT_DEBUG
//				// Debug
				printf("=== Downshift State: FINISH_SHIFT\n");
#ifdef NO_GEAR_POT
				printf("Shift Lever below Downshift Exit Threshold\n");
#else
				printf("Gear has decreased 2 states and now in the desired gear\n");
#endif
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
				break;
			}

			// check for a timeout entering the gear
			if (HAL_GetTick() - begin_enter_gear_tick > DOWNSHIFT_ENTER_TIMEOUT_MS(shift_mode_2))
			{
				tcm_data.successful_shift = false;
				error(SHIFT_STATE_TIMEOUT, &error_byte);

				// the shift failed to enter the gear. We want to keep the clutch
				// open for some extra time to try and give the driver the chance
				// to rev around and find a gear. Call this shift a failure
				tcm_data.using_clutch = true;
				set_slow_clutch_drop(SOLENOID_OFF);
				set_clutch_solenoid(SOLENOID_ON);
				safe_spark_cut(false);
				next_downshift_state = ST_D_HOLD_CLUTCH;
				begin_hold_clutch_tick = HAL_GetTick();

#ifdef SHIFT_DEBUG
				//Debug
				printf("=== Downshift State: HOLD_CLUTCH\n");
				printf("How: Waiting for shift pot timed out\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		else
		{
			// No spark cut without being confident in the trans speed

			if (HAL_GetTick() - begin_enter_gear_tick > DOWNSHIFT_ENTER_GEAR_TIME_MS(shift_mode_2)) {
				begin_extra_push_time_tick = HAL_GetTick();
				next_downshift_state = ST_D_EXTRA_PUSH;
				safe_spark_cut(false);

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Time Shift Only Downshift State: FINISH_SHIFT\n");
				printf("How: Enter Gear time completed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		break;

	case ST_D_HOLD_CLUTCH: // **********************************************
		// some extra time to hold the clutch open. This is in the case that
		// the shift lever does not hit the threshold and might needs some
		// input from the driver to hit the right revs
		// Should still happen in clutchless downshift because we only use
		// mode to save air but would prefer not to fail shift
		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(SOLENOID_ON);
		set_downshift_solenoid(SOLENOID_ON);
		safe_spark_cut(false);

		// check if we are done giving the extra time TODO Make sure this gets tuned (currently long apparently)
		if (HAL_GetTick() - begin_hold_clutch_tick > DOWNSHIFT_FAIL_EXTRA_CLUTCH_HOLD(shift_mode_2))
		{
			// done giving the extra clutch. Move on to finishing the shift
			begin_extra_push_time_tick = HAL_GetTick();
			next_downshift_state = ST_D_EXTRA_PUSH;

#ifdef SHIFT_DEBUG
			// Debug
			printf("=== Downshift State: FINISH_SHIFT\n");
			printf("How: Hold Clutch time completed\n");
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();
#endif
		}
		break;

	case ST_D_EXTRA_PUSH:
		safe_spark_cut(false);

		set_downshift_solenoid(SOLENOID_ON);

		if (HAL_GetTick() - begin_extra_push_time_tick > DOWNSHIFT_EXTRA_PUSH_TIME_MS(shift_mode_2))
		{
			next_downshift_state = ST_D_FINISH_SHIFT;
		}
		break;

	case ST_D_FINISH_SHIFT: // **********************************************
		// winding down the downshift. Make sure enough time has passed with
		// the clutch open if we are using the clutch, otherwise end the shift
		// but keep pushing on the shift solenoid for a little bit longer to
		// ensure the shift completes
		set_downshift_solenoid(SOLENOID_ON);

		safe_spark_cut(false);
		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(SOLENOID_OFF);
		tcm_data.using_clutch = false;
		tcm_data.num_shifts += 1;

#ifdef NO_GEAR_POT
		tcm_data.current_gear = get_current_gear();
#endif

		// Determine if the shift was successful by checking if we changed gear correctly
		tcm_data.successful_shift = tcm_data.current_gear <= initial_gear - 2;
		tcm_data.gear_established = tcm_data.successful_shift;
		if (tcm_data.successful_shift) {
			tcm_data.num_successful_shifts += 1;
		}

		// done with the downshift state machine
		set_downshift_solenoid(SOLENOID_OFF);
		main_state = ST_IDLE;

#ifdef SHIFT_DEBUG
		// Debug
		printf("=== Main State: Idle\n");
		printf("How: Finish Shift functions completed\n");
		printf("Current Tick: %lu\n", HAL_GetTick());
		printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
		lastShiftingChangeTick = HAL_GetTick();
#endif
	}
}
