/*
 * software_timer.h
 *
 *  Created on: Sep 16, 2024
 *      Author: DELL
 */

#ifndef INC_SOFTWARE_TIMER_H_
#define INC_SOFTWARE_TIMER_H_

#define TICK 10
#define NUM_TIM 4

void setTimer(int index, int counter);
int getTimerFlag(int index);
void timerRun();


#endif /* INC_SOFTWARE_TIMER_H_ */
