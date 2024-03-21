/*
 * upshift_sm.c
 *
 *  Created on: Mar 20, 2024
 *      Author: chris
 */

#include "main_task.h"
#include "main.h"
#include "gopher_sense.h"
#include "pulse_sensor.h"
#include "utils.h"
#include "status_utils.h"
#include <stdio.h>
#include <stdbool.h>
#include "shift_parameters.h"
#include <string.h>
#include <stdbool.h>
#include <cmsis_os.h>

// run_upshift_sm
//  The big boi upshift state machine
void run_upshift_sm(void)
{
	static uint32_t initial_gear;
	static uint32_t begin_shift_tick;
	static uint32_t begin_exit_gear_tick;
	static uint32_t begin_enter_gear_tick;
	static uint32_t begin_extra_push_tick;
	static uint32_t begin_exit_gear_spark_return_tick;

	// calculate the target RPM at the start of each cycle through the loop
	tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);

	switch (upshift_state)
	{
	case ST_U_BEGIN_SHIFT:
		// at the beginning of the upshift, start pushing on the shift lever
		// to "preload". This means that there will be force on the shift
		// lever when the spark is cut and the gear can immediately disengage
		// NOTE: We really dont care about what the clutch is doing in an
		// upshift

		begin_shift_tick = HAL_GetTick(); // Log that first begin shift tick

#ifdef NO_GEAR_POT
		tcm_data.current_gear = get_current_gear();
#endif

		initial_gear = tcm_data.current_gear;
		tcm_data.successful_shift = false;

		set_upshift_solenoid(SOLENOID_ON); // start pushing upshift

		// move on to waiting for the "preload" time to end
		next_upshift_state = ST_U_LOAD_SHIFT_LVR;

#ifdef SHIFT_DEBUG
		// Debug
		printf("=== Upshift State: LOAD_SHIFT_LVR\n");
		printf("How: Completed begin shift steps\n");
		printf("Current Tick: %lu\n", HAL_GetTick());
		printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
		lastShiftingChangeTick = HAL_GetTick();
#endif
		break;

	case ST_U_LOAD_SHIFT_LVR:
		// we want to push on the solenoid, but not spark cut to preload the shift lever
		set_upshift_solenoid(SOLENOID_ON);
		safe_spark_cut(false);

		// wait for the preload time to be over
		if (HAL_GetTick() - begin_shift_tick > UPSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS(shift_mode_2))
		{
			// preload time is over. Start spark cutting to disengage the
			// gears and moving the RPM to match up with the next gearS
			begin_exit_gear_tick = HAL_GetTick();
			safe_spark_cut(true);

			// move on to waiting to exit gear
			next_upshift_state = ST_U_EXIT_GEAR;

#ifdef SHIFT_DEBUG
			// Debug
			printf("=== Upshift State: EXIT_GEAR\n");
			printf("How: Preload time completed\n");
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();
#endif
		}
		break;

	case ST_U_EXIT_GEAR: // **************************************************
		// wait until the threshold for the lever leaving the last gear has
		// been met or the timeout hits. During this time spark should be cut
		// to change engine driving the wheels to the wheels driving the
		// engine. If all goes well, the gear should be left at that midpoint
		set_upshift_solenoid(SOLENOID_ON);

		safe_spark_cut(true);

		// check which shift mode we're in (to determine whether sensors
		//  should be used)
		if (!tcm_data.time_shift_only)
		{
#ifdef NO_GEAR_POT
			// wait for the shift lever to reach the leaving threshold
			if (get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM)
			{
#else
			// Check if we're now in between gears
			if (tcm_data.current_gear > initial_gear)
			{
#endif
				// the shifter position is above the threshold to exit. The
				// transmission is now in a false neutral position
				next_upshift_state = ST_U_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Upshift State: ENTER_GEAR\n");
#ifdef NO_GEAR_POT
				printf("Shift lever moved far enough\n");

#else
				printf("Gear state increased since initial signaling in between gears\n");
#endif
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
				break;
			}

			// the shift lever has not moved far enough. Check if it has been
			// long enough to timeout yet
			if (HAL_GetTick() - begin_exit_gear_tick > UPSHIFT_EXIT_TIMEOUT_MS(shift_mode_2))
			{
				// the shift lever did not exit the previous gear. Attempt to
				// return spark to attempt to disengage
				error(SHIFT_STATE_TIMEOUT, &error_byte);
				begin_exit_gear_spark_return_tick = HAL_GetTick();
				safe_spark_cut(false);
				next_upshift_state = ST_U_SPARK_RETURN;

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Upshift State: SPARK_RETURN\n");
				printf("How: Shift lever movement timeout passed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		else
		{
			if (HAL_GetTick() - begin_exit_gear_tick > UPSHIFT_EXIT_GEAR_TIME_MS(shift_mode_2)) {
				next_upshift_state = ST_U_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Time Shit Only Upshift State: ENTER_GEAR\n");
				printf("How: Exit gear time completed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		break;

	// Unused in time based shifting
	case ST_U_SPARK_RETURN: // ********************************************
		// A common cause of a failed disengagement is the transition from
		// the engine driving the wheels to the wheels driving the engine
		// too quickly, meaning there is no way to exit the gear while
		// the spark is cut. In order to leave the gear we must return spark
		// briefly
		set_upshift_solenoid(SOLENOID_ON);
		safe_spark_cut(true);

#ifdef NO_GEAR_POT
		// the shifter has moved above the threshold of exiting the gear. Spark
		// must be cut again to reach the correct RPM for the next gear. If
		// enough time has passed return spark anyway
		if (get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM
#else
		if (tcm_data.current_gear > initial_gear
#endif
			|| HAL_GetTick() - begin_exit_gear_spark_return_tick > UPSHIFT_EXIT_SPARK_RETURN_MS(shift_mode_2))
		{

#ifdef SHIFT_DEBUG
			// Debug
			printf("=== Upshift State: ENTER_GEAR\n");

#ifdef NO_GEAR_POT
			if(get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM) {
				printf("How: Shifter moved above threshold of exiting\n");
			} else {
				printf("How: Spark return timed out\n");
			}
#else
			printf("Gear state increased since initial signaling in between gears\n");
#endif
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();
#endif

			// If spark return successfully releases then continue onto the
			// next phase of the shift. If it was not successful (timeout) move
			// on anyway
			safe_spark_cut(true);
			next_upshift_state = ST_U_ENTER_GEAR;
			begin_enter_gear_tick = HAL_GetTick();

		}
		break;

	case ST_U_ENTER_GEAR: // *********************************************
		// we are in a false neutral and waiting to enter the gear. Likely
		// the RPMs will need to drop a little more to make it to the next
		// gear
		set_upshift_solenoid(SOLENOID_ON);

		if (!tcm_data.time_shift_only)
		{
			reach_target_RPM_spark_cut(tcm_data.target_RPM);

#ifdef NO_GEAR_POT
			// check if the shifter position is above the threshold to complete
			// a shift
			if (get_shift_pot_pos() > UPSHIFT_ENTER_POS_MM)
			{
#else
			// Check if we are in the next up gear (+2 gear states as +1 is in-between state)
			if (tcm_data.current_gear == (initial_gear + 2))
			{
#endif

				begin_extra_push_tick = HAL_GetTick();
				next_upshift_state = ST_U_EXTRA_PUSH;
				// shift position says we are done shifting
//				next_upshift_state = ST_U_FINISH_SHIFT;

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Upshift State: FINISH_SHIFT\n");
#ifdef NO_GEAR_POT
				printf("How: Shifter position moved past threshold\n");
#else
				printf("Gear has increased 2 states and now in the desired gear\n");
#endif
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
				break;
			}

			// check if we are out of time for this state
			if (HAL_GetTick() - begin_enter_gear_tick > UPSHIFT_ENTER_TIMEOUT_MS(shift_mode_2))
			{
				error(SHIFT_STATE_TIMEOUT, &error_byte);
				// at this point the shift was probably not successful. Note this so we
				// dont increment the gears and move on
				tcm_data.successful_shift = false;
				begin_extra_push_tick = HAL_GetTick();
				next_upshift_state = ST_U_EXTRA_PUSH;
//				next_upshift_state = ST_U_FINISH_SHIFT;

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Upshift State: FINISH_SHIFT\n");
				printf("How: Enter Gear timed out\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		else
		{
			// Continuous spark cutting is less desirable because the revs
			// may drop below what they need to be, but it is ok as they
			// will come back up when this section is over
			set_spark_cut(true);

			if (HAL_GetTick() - begin_enter_gear_tick > UPSHIFT_ENTER_GEAR_TIME_MS(shift_mode_2))
			{
				begin_extra_push_tick = HAL_GetTick();
				next_upshift_state = ST_U_EXTRA_PUSH;

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Timed Shift Only Upshift State: FINISH_SHIFT\n");
				printf("How: Enter Gear time completed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		break;

	case ST_U_EXTRA_PUSH:
		safe_spark_cut(false);

		set_upshift_solenoid(SOLENOID_ON);

		if (HAL_GetTick() - begin_extra_push_tick > UPSHIFT_EXTRA_PUSH_TIME_MS(shift_mode_2))
		{
			next_upshift_state = ST_U_FINISH_SHIFT;
		}
		break;

	case ST_U_FINISH_SHIFT: // **********************************************
		// The shift is over, make sure we're no longer
		// spark cutting and aren't using the clutch
		set_upshift_solenoid(SOLENOID_ON);

		safe_spark_cut(false);
		tcm_data.using_clutch = false;
		tcm_data.num_shifts += 1;

#ifdef NO_GEAR_POT
		// Only grab the current gear if we don't have a gear pot
		// because if we have one its already updating every cycle
		tcm_data.current_gear = get_current_gear();
#endif

		// Determine if the shift was successful by checking if we changed gear correctly
		tcm_data.successful_shift = tcm_data.current_gear >= initial_gear + 2;
		tcm_data.gear_established = tcm_data.successful_shift;
		if (tcm_data.successful_shift) {
			tcm_data.num_successful_shifts += 1;
		}

		// done with the upshift state machine
		set_upshift_solenoid(SOLENOID_OFF);
		main_state = ST_IDLE;

#ifdef SHIFT_DEBUG
		// Debug
		printf("=== Main State: IDLE\n");
		printf("How: Finish Shift functions completed\n");
		printf("Current Tick: %lu\n", HAL_GetTick());
		printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
		lastShiftingChangeTick = HAL_GetTick();
#endif
	}
}
