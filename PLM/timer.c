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

  CMU_ClockEnable(TIMER_GEN_COURANT_CLK, true);
  CMU_ClockEnable(TIMER_ENV_CLK, true);

  /////// Init TIMER_GEN_COURANT
  // Do not start counter upon initialization
  timerInit.enable = false;

  // Run in one-shot mode and toggle the pin on each compare match
  timerInit.oneShot = true;
  timerInit.prescale = (CMU_ClockFreqGet(TIMER_GEN_COURANT_CLK)/TIMER_GEN_COURANT_FRQ) - 1;
  TIMER_Init(TIMER0, &timerInit);

  TIMER_IntEnable(TIMER_GEN_COURANT,TIMER_IF_OF);

  /////// Init TIMER_ENV
  timerInit.prescale = (CMU_ClockFreqGet(TIMER_ENV_CLK)/TIMER_ENV_FRQ) - 1;
  // Run in one-shot mode and toggle the pin on each compare match
    timerInit.oneShot = false;
  TIMER_Init(TIMER_ENV, &timerInit);

  TIMER_IntEnable(TIMER_ENV,TIMER_IF_OF);
//
  NVIC_ClearPendingIRQ(TIMER0_IRQn);
  NVIC_EnableIRQ(TIMER0_IRQn);
//
  NVIC_ClearPendingIRQ(TIMER1_IRQn);
  NVIC_EnableIRQ(TIMER1_IRQn);
}

void set_timer0_time(uint32_t time)
{
	TIMER_Enable(TIMER_GEN_COURANT, false);
	TIMER_IntClear(TIMER_GEN_COURANT, TIMER_IF_OF);
	TIMER_CounterSet(TIMER_GEN_COURANT,0);
	uint32_t cnt = time - 1;
	if(TIMER_TopGet(TIMER_GEN_COURANT) != cnt)
	{
		TIMER_TopSet(TIMER_GEN_COURANT, cnt);
	}
	TIMER_Enable(TIMER_GEN_COURANT, true);
}

void set_timer1_time(uint32_t time)
{
  TIMER_Enable(TIMER_ENV, false);
  TIMER_IntClear(TIMER_ENV, TIMER_IF_OF);
  TIMER_CounterSet(TIMER_ENV,0);
  uint32_t cnt = time - 1;
  if(TIMER_TopGet(TIMER_ENV) != cnt)
  {
    TIMER_TopSet(TIMER_ENV, cnt);
  }
  TIMER_Enable(TIMER_ENV, true);
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
	TIMER_IntClear(TIMER_GEN_COURANT, TIMER_IF_OF);
}

