/*
 * timer.h
 *
 *  Created on: 14 déc. 2023
 *      Author: ebt
 */

#ifndef APP_TIMER_H_
#define APP_TIMER_H_
#include "em_timer.h"
#include "em_cmu.h"
#include "stdint.h"

#define TIMER_GEN_COURANT_FRQ 10000000
#define TIMER_ENV_FRQ         1000000

#define TIMER_GEN_COURANT     TIMER0
#define TIMER_GEN_COURANT_CLK cmuClock_TIMER0

#define TIMER_ENV             TIMER1
#define TIMER_ENV_CLK         cmuClock_TIMER1

void initTIMER(void);
/*@param : time en µs*/
void set_timer0_time(uint32_t time);
void set_timer1_time(uint32_t time);
void TIMER0_IRQHandler(void);
#endif /* APP_TIMER_H_ */
