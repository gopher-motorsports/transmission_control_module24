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

/*DRS duty cycle periods: 500ms-2500ms pulses with a max period of 3000ms
	corresponds to 10,000-50,000 compare match value and 59999 overflow value
	for timer 3 */

//proportional steps from 10,000-50,000, each step is 1764
#define DRS_POS_0 10000
#define DRS_POS_1 11764
#define DRS_POS_2 13529
#define DRS_POS_3 15294
#define DRS_POS_4 17058
#define DRS_POS_5 18823
#define DRS_POS_6 20588
#define DRS_POS_7 22352
#define DRS_POS_8 24117
#define DRS_POS_9 25882
#define DRS_POS_10 27647
#define DRS_POS_11 29411
#define DRS_POS_12 31176
#define DRS_POS_13 32941
#define DRS_POS_14 34705
#define DRS_POS_15 50000

#define ROT_DIAL_POS 2 //arbitrary positon, should be changed to steering wheel macro
void pass_on_timer_info(TIM_HandleTypeDef* timer_address, U32 channel);


#endif /* INC_DRS_H_ */
