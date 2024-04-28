/*
 * drs.h
 *
 *  Created on: Mar 21, 2024
 *      Author: chris
 */

#ifndef INC_DRS_H_
#define INC_DRS_H_
#include "GopherCAN.h"
#include "gopher_sense.h"

/*DRS duty cycle periods: 500us-2500us pulses with a max period of 3000ms
	corresponds to 2500-24999 compare match value and 29999 overflow value
	for timer 3 */

//proportional steps from 2500-24999, each step is 1250
#define DRS_POS_0  4999
#define DRS_POS_1  6249
#define DRS_POS_2  7499
#define DRS_POS_3  8749
#define DRS_POS_4  9999
#define DRS_POS_5  11249
#define DRS_POS_6  12499
#define DRS_POS_7  13749
#define DRS_POS_8  14999
#define DRS_POS_9  16249
#define DRS_POS_10 17499
#define DRS_POS_11 18749
#define DRS_POS_12 19999
#define DRS_POS_13 21249
#define DRS_POS_14 22499
#define DRS_POS_15 23749
#define DRS_POS_16 24999

#define OPEN_POS 18959 //63.2% duty cycle
#define CLOSED_POS 25919 //86.4% duty cycle

void init_DRS_servo(TIM_HandleTypeDef* timer_address, U32 channel);
void set_DRS_Servo_Position();
#endif /* INC_DRS_H_ */
