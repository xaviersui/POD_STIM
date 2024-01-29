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

#define FREQ_TIM0_CNT 1000000
#define FREQ_TIM1_CNT 1000000
#define GEN_TIMER TIMER1
void initTIMER(void);
/*@param : time en µs*/
void set_timer0_time(uint32_t time);
void set_timer1_time(uint32_t time);
void TIMER0_IRQHandler(void);
#endif /* APP_TIMER_H_ */
