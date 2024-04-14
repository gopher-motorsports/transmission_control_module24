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
void init_error(void);


#endif /* INC_STATUS_UTILS_H_ */
