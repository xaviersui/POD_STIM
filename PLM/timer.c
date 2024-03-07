/*
 * timer.c
 *
 *  Created on: 14 déc. 2023
 *      Author: ebt
 */
#include "timer.h"
#include "siue.h"
#include "stdbool.h"
#include "stim_management.h"
#include "stimulation.h"
#define N_MODULATION_MAX 4 /**< Maximum number of modulation by envelope */



TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
extern void (*pStimGenCallback[gStimPatternMax_c])(void);
extern Stimulation_t gStim_t;    /**< */

/**************************************************************************//**
 * @brief
 *    TIMER initialization
 *****************************************************************************/
void initTIMER(void)
{

  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);
  // Do not start counter upon initialization
  timerInit.enable = false;

  // Run in one-shot mode and toggle the pin on each compare match
  timerInit.oneShot = true;
  timerInit.prescale = (CMU_ClockFreqGet(cmuClock_TIMER0)/FREQ_TIM0_CNT) - 1;
  TIMER_Init(TIMER0, &timerInit);

  TIMER_IntEnable(TIMER0,TIMER_IF_OF);


  timerInit.prescale = (CMU_ClockFreqGet(cmuClock_TIMER1)/FREQ_TIM1_CNT) - 1;
  // Run in one-shot mode and toggle the pin on each compare match
    timerInit.oneShot = false;
  TIMER_Init(TIMER1, &timerInit);

  TIMER_IntEnable(TIMER1,TIMER_IF_OF);

  //TIMER_TopSet(TIMER0,cnt);
  NVIC_ClearPendingIRQ(TIMER0_IRQn);
  NVIC_EnableIRQ(TIMER0_IRQn);

  NVIC_ClearPendingIRQ(TIMER1_IRQn);
  NVIC_EnableIRQ(TIMER1_IRQn);
}

void set_timer0_time(uint32_t time)
{
	TIMER_Enable(TIMER0, false);
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
	TIMER_CounterSet(TIMER0,0);
	uint32_t cnt = time - 1;
	if(TIMER_TopGet(TIMER0) != cnt)
	{
		TIMER_TopSet(TIMER0, cnt);
	}
	TIMER_Enable(TIMER0, true);
}

void set_timer1_time(uint32_t time)
{
  TIMER_Enable(TIMER1, false);
  TIMER_IntClear(TIMER1, TIMER_IF_OF);
  TIMER_CounterSet(TIMER1,0);
  uint32_t cnt = time*1000000 - 1;
  if(TIMER_TopGet(TIMER1) != cnt)
  {
    TIMER_TopSet(TIMER1, cnt);
  }
  TIMER_Enable(TIMER1, true);
}

void TIMER0_IRQHandler(void)
{
// MeSS_GestionCourantBiphasiquePositif();
 //MeSS_GestionCourantMonophasiquePositif();
  //MeSS_GestionCourantBiphasiqueNegatif();
 // MeSS_GestionCourantMonophasiqueNegatif();
// MeSS_GestionCourantBiphasiqueAlterne();
//  VeineuxBiphas();
 (void)pStimGenCallback[gStim_t.tConfig.patternId]();
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
}

