/*
 * status_utils.h
 *
 *  Created on: Mar 20, 2024
 *      Author: chris
 */

#ifndef INC_STATUS_UTILS_H_
#define INC_STATUS_UTILS_H_
#include "GopherCAN.h"
#include "gopher_sense.h"

void update_task_utilization();
void send_uart_tick();
void toggle_heart_beat();
void checkForErrors(void);
void change_led_state(U8 sender, void* UNUSED_LOCAL_PARAM, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
void init_error(void);


#endif /* INC_STATUS_UTILS_H_ */
