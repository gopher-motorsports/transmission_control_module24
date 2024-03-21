// GopherCAN_devboard_example.c
//  This is a bare-bones module file that can be used in order to make a module main file
//TODO - Fix clutch activating upshift bug

//library includes
#include "gopher_sense.h"
#include "pulse_sensor.h"

//TCM specific file includes
#include "main_task.h"
#include "main.h"
#include "utils.h"
#include "status_utils.h"
#include "upshift_sm.h"
#include "downshift_sm.h"
#include "shift_parameters.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdbool.h>
#include <cmsis_os.h>

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;

//shifting states/variables
Main_States_t main_state = ST_IDLE;
float shift_mode_2 = 0; //time based or closed loop, shared between upshift_sm.c and downshift_sm.c
Upshift_States_t upshift_state, next_upshift_state; //shared between maintask.c and upshift_sm.c
Downshift_States_t downshift_state, next_downshift_state; //shared between maintask.c and downshift_sm.c

U32 initialization_start_time_ms = 0;
U8 last_button_state = 0;
bool initial_input_skip = true;

U8 error_byte = 0; //shared between status_utils, upshift_sm.c, downshift_sm.c
#ifdef SHIFT_DEBUG
// All upper shifting debug statements
static U32 lastShiftingChangeTick = 0;
#endif

//function headers
static void updateAndQueueParams(void);
static void check_driver_inputs(void);
static void clutch_task();
static void shifting_task();


// init
//  What needs to happen on startup in order to run GopherCAN
void init(CAN_HandleTypeDef* hcan_ptr)
{
	example_hcan = hcan_ptr;

	// initialize CAN
	// NOTE: CAN will also need to be added in CubeMX and code must be generated
	// Check the STM_CAN repo for the file "F0xx CAN Config Settings.pptx" for the correct settings
	if (init_can(GCAN0, example_hcan, THIS_MODULE_ID, BXTYPE_MASTER))
	{
		init_error();
	}

	// Set the function pointer of SET_LED_STATE. This means the function change_led_state()
	// will be run whenever this can command is sent to the module
	if (add_custom_can_func(SET_LED_STATE, &change_led_state, TRUE, NULL))
	{
		init_error();
	}

	initialization_start_time_ms = HAL_GetTick();

	// Lock param sending for everything - uncomment for stress testing CAN
//	lock_param_sending(&counterShaftSpeed_rpm.info);
//	lock_param_sending(&tcmTargetRPM_rpm.info);
//	lock_param_sending(&tcmCurrentGear_state.info);
//	lock_param_sending(&tcmCurrentlyMoving_state.info);
//	lock_param_sending(&tcmAntiStallActive_state.info);
//	lock_param_sending(&tcmError_state.info);
//	lock_param_sending(&tcmUsingClutch_state.info);
//	lock_param_sending(&tcmUsingClutch_state.info);

	lock_param_sending(&tcmTimeShiftOnly_state.info);
#ifdef CAN_CLUTCHLESS_DOWNSHIFT
	lock_param_sending(&tcmClutchlessDownshift_state.info);
#endif

	tcm_data.time_shift_only = true;
}


// can_buffer_handling_loop
//  This loop will handle CAN RX software task and CAN TX hardware task. Should be
//  called every 1ms or as often as received messages should be handled
void can_buffer_handling_loop()
{
	// handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// an error has occurred
	}

	// handle the transmission hardware for each CAN bus
	service_can_tx(example_hcan);
}

// main_loop
//  another loop. This includes logic for sending a CAN command. Designed to be
//  called every 10ms
void main_loop()
{
#ifdef NO_GEAR_POT
	static uint32_t last_gear_update = 0;
#endif

#ifdef RUN_TIME_STATS
	static char taskBuffer[250];
#endif

#ifdef RUN_TIME_STATS
		// TESTING: Get runtime stats
		vTaskGetRunTimeStats(taskBuffer);
		printf("%s\n", taskBuffer);
#endif
		//status checks, in status_utils.c
		update_task_utilization();
		send_uart_tick(); // send the current tick over UART every second
		toggle_heart_beat();
		checkForErrors();

		//send CAN params
		updateAndQueueParams();

#ifdef NO_GEAR_POT
	if (HAL_GetTick() - last_gear_update >= GEAR_UPDATE_TIME_MS)
	{
		last_gear_update = HAL_GetTick();
		tcm_data.current_gear = get_current_gear();
	}
#else
		tcm_data.current_gear = get_current_gear();
#endif

		check_pulse_sensors();
		update_tcm_data();

		if (!initial_input_skip) {
			check_driver_inputs();
		} else if (HAL_GetTick() - initialization_start_time_ms > 2000) {
			initial_input_skip = false;
		}

		shifting_task();
		clutch_task();

		// Clear the error byte, so it has to keep being triggered if the error is persistent (and doesn't require a function to turn it off again)
		error_byte = 0;
}


// Updates gcan variables
static void updateAndQueueParams(void) {
	update_and_queue_param_float(&counterShaftSpeed_rpm, tcm_data.trans_speed);
	update_and_queue_param_float(&tcmCurrentGear_state, tcm_data.current_gear);
	update_and_queue_param_u16(&tcmTargetRPM_rpm, tcm_data.target_RPM);
	update_and_queue_param_u8(&tcmCurrentlyMoving_state, tcm_data.currently_moving);
	update_and_queue_param_u8(&tcmAntiStallActive_state, tcm_data.anti_stall);
	update_and_queue_param_u8(&tcmUsingClutch_state, tcm_data.using_clutch);
	update_and_queue_param_u8(&tcmTimeShiftOnly_state, tcm_data.time_shift_only);
	update_and_queue_param_u8(&tcmClutchlessDownshift_state, shift_mode_2); // tcm_data.clutchless_downshift);
	update_and_queue_param_u8(&tcmTargetGear_state, tcm_data.target_gear);
	update_and_queue_param_u8(&tcmSpkCut_state, tcm_data.spark_cut);
	update_and_queue_param_u8(&tcmClutchlessDownshift_state, tcm_data.clutchless_downshift);
	update_and_queue_param_u8(&tcmSuccessfulShift_state, tcm_data.successful_shift);
	update_and_queue_param_u8(&tcmError_state, error_byte);
	update_and_queue_param_u16(&tcmNumShifts_ul, tcm_data.num_shifts);
	update_and_queue_param_u16(&tcmNumSuccessfulShifts_ul, tcm_data.num_successful_shifts);

	switch (main_state)
	{
	default:
	case ST_IDLE:
		// not shifting, send 0
		update_and_queue_param_u8(&tcmShiftState_state, 0);
		break;

	case ST_HDL_UPSHIFT:
		// send the upshift state
		update_and_queue_param_u8(&tcmShiftState_state, upshift_state);
		break;

	case ST_HDL_DOWNSHIFT:
		// send the downshift state
		update_and_queue_param_u8(&tcmShiftState_state, downshift_state);
		break;
	}
}

static float last_upshift_button = 0;
static float last_downshift_button = 0;

#ifdef CAN_CHANGE_FROM_TIME_SHIFT
static float last_timeShiftOnly_button = 0;
#endif

#ifdef CAN_CLUTCHLESS_DOWNSHIFT
static float last_clutchlessDownshift_button = 0;
#endif

static void check_driver_inputs() {
	tcm_data.sw_fast_clutch = FAST_CLUTCH_BUTTON;
	tcm_data.sw_slow_clutch = SLOW_CLUTCH_BUTTON;

#ifdef CAN_CHANGE_FROM_TIME_SHIFT
	if((last_timeShiftOnly_button == 0) && (TIME_SHIFT_ONLY_BUTTON == 1)) {
		tcm_data.time_shift_only = !tcm_data.time_shift_only;
	}
	last_timeShiftOnly_button = TIME_SHIFT_ONLY_BUTTON;
#endif

#ifdef CAN_CLUTCHLESS_DOWNSHIFT
	if((last_clutchlessDownshift_button == 0) && (CLUTCHLESS_DOWNSHIFT_BUTTON == 1)) {
		// tcm_data.clutchless_downshift = !tcm_data.clutchless_downshift; // commented out for shift testing and instead used for 2nd shift parameter mode
		shift_mode_2 = !shift_mode_2;
	}
	last_clutchlessDownshift_button = CLUTCHLESS_DOWNSHIFT_BUTTON;
#endif

	// Check button was released before trying shifting again - falling edge
	if ((last_upshift_button == 0) && (UPSHIFT_BUTTON == 1)) {
		if (tcm_data.pending_shift == DOWNSHIFT) {
			tcm_data.pending_shift = NONE;
		} else {
			tcm_data.pending_shift = UPSHIFT;
		}
	}
	last_upshift_button = UPSHIFT_BUTTON;

	// Check button was released before trying shifting again - falling edge
	if ((last_downshift_button == 0) && (DOWNSHIFT_BUTTON == 1)) {
		if(tcm_data.pending_shift == UPSHIFT) {
			tcm_data.pending_shift = NONE;
		} else {
			tcm_data.pending_shift = DOWNSHIFT;
		}
	}
	last_downshift_button = DOWNSHIFT_BUTTON;
}

static void shifting_task() {
	switch (main_state)
	{
	case ST_IDLE:
		// in idle state, make sure we are not spark cutting and not pushing
		// the solenoid
		set_spark_cut(false);

		// NOTE: potentially use the opposite solenoid to push the shift lever
		// back into a neutral position quicker. This would use more air but
		// would allow us to shift again much sooner. We would need to make
		// sure the shift position is valid before doing this to prevent
		// always pushing the shift lever into a shifting position during
		// idle state

#ifdef AUTO_SHIFT_LEVER_RETURN
		// TODO WARNING this will mean shifting will not work if the shift
		// pot gets disconnected as the lever will push one way
		if (!tcm_data.time_shift_only) {
			if (get_shift_pot_pos() > LEVER_NEUTRAL_POS_MM + LEVER_NEUTRAL_TOLERANCE)
			{
				// shifter pos is too high. Bring it back down
				set_upshift_solenoid(SOLENOID_OFF);
				set_downshift_solenoid(SOLENOID_ON);
			}
			else if (get_shift_pot_pos() < LEVER_NEUTRAL_POS_MM - LEVER_NEUTRAL_TOLERANCE)
			{
				// shifter pos is too low. Bring it up
				set_upshift_solenoid(SOLENOID_ON);
				set_downshift_solenoid(SOLENOID_OFF);
			}
			else
			{
				// we good. Levers off
				set_upshift_solenoid(SOLENOID_OFF);
				set_downshift_solenoid(SOLENOID_OFF);
			}
		}
#endif

		// start a shift if there is one pending. This means that a new
		// shift can be queued during the last shift. Always allow shifts in time shifting mode
		if(tcm_data.pending_shift == UPSHIFT) {
			if (calc_validate_upshift(tcm_data.current_gear, tcm_data.sw_fast_clutch,
					                  tcm_data.sw_slow_clutch) ||
					tcm_data.time_shift_only)
			{
				main_state = ST_HDL_UPSHIFT;
				upshift_state = ST_U_BEGIN_SHIFT;
			}
		} else if (tcm_data.pending_shift == DOWNSHIFT) {
			if (calc_validate_downshift(tcm_data.current_gear, tcm_data.sw_fast_clutch,
					                    tcm_data.sw_slow_clutch) ||
					tcm_data.time_shift_only)
			{
				main_state = ST_HDL_DOWNSHIFT;
				downshift_state = ST_D_BEGIN_SHIFT;
			}
		}
		tcm_data.pending_shift = NONE;
		break;

	case ST_HDL_UPSHIFT:
		run_upshift_sm();
		upshift_state = next_upshift_state; // Set state to next state variable per Sebastion's request
		break;

	case ST_HDL_DOWNSHIFT:
		run_downshift_sm();
		downshift_state = next_downshift_state;
		break;
	}
}

// clutch_task
//  for use in the main task. Sets the slow and fast drop accordingly and handles
//  clutch position if in ST_IDLE
static void clutch_task() {
	static bool using_slow_drop = false;
	// normal clutch button must not be pressed when using slow drop. Fast drop is
	// given priority
	if (tcm_data.sw_fast_clutch) using_slow_drop = false;

	// if the slow drop button is pressed latch the slow drop
	else if (tcm_data.sw_slow_clutch) using_slow_drop = true;

	// If either clutch button pressed then enable solenoid. Always turn it on regardless of
	// if we are shifting or not
	if (tcm_data.sw_fast_clutch || tcm_data.sw_slow_clutch)
	{
		set_clutch_solenoid(SOLENOID_ON);
		return;
	} else if (main_state == ST_IDLE && !tcm_data.anti_stall) {
		// If neither clutch button pressed and we are in IDLE and not in anti stall
		// close clutch solenoid. This will cause clutch presses to latch to the end
		// of a shift
		set_clutch_solenoid(SOLENOID_OFF);

#ifdef SMART_SLOW_CLUTCH
		// if we are using slow drop enable or disable the slow release valve depending
		// on if we are near the bite point
		if (using_slow_drop)
		{
			// TODO: Check if we want a way to make this not closed loop
			// when slow dropping, we want to start by fast dropping until the bite point
			if (get_clutch_pot_pos() > (CLUTCH_OPEN_POS_MM + CLUTCH_SLOW_DROP_FAST_TO_SLOW_EXTRA_MM))
			{
				set_slow_clutch_drop(false);
			}
			else
			{
				set_slow_clutch_drop(true);
			}
		}
		else
		{
			// not using slow drop
			set_slow_clutch_drop(false);

		}
#else
		if (using_slow_drop) {
			set_slow_clutch_drop(true);
		} else {
			set_slow_clutch_drop(false);
		}
#endif
	}
}
