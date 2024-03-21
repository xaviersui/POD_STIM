/**
* @file   Stim_Management.h
* @ingroup  GrpBSP
* @brief  Stim Management header file.
*
* Copyright : 2009 Vivaltis.\n
*         All Rights Reserved
*/

/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/


#ifndef STIM_MANAGEMENT_H_INCLUDED
#define STIM_MANAGEMENT_H_INCLUDED

/***********************************************************************************
System Includes
***********************************************************************************/

/***********************************************************************************
User Includes
***********************************************************************************/
/* sfr_r827.h provides a structure to access all of the device registers. */
// #include "sfr_r827.h"
/* RSKR8C27def.h provides common defines for widely used items. */
#include "rskR8C27def.h"
#include "srl_comm_management.h"

//#include "timer.h"
#include "gpio.h"
#include "siue.h"
#include "timer.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

/* CMD_DEMAG Management */
#define CMD_DEMAG_PIN       p4_5

#define CMD_DEMAG_DIS       CMD_DEMAG_PIN = gGpioPinStateHigh_c
#define CMD_DEMAG_EN        CMD_DEMAG_PIN = gGpioPinStateLow_c

/* Switching Management */
#define SWITCH_FORCE_COUNT_STOP   TMR_RA_FORCE_COUNT_STOP

/* CMD_Mi Management */


#define CMD_M_PORT          p0
#define CMD_M_PIN_MASK        0xF0


/* DAC Chip Select Management */
#define DAC_CS_PIN          p5_3

#define DAC_CS_DIS          DAC_CS_PIN = gGpioPinStateHigh_c;
#define DAC_CS_EN         DAC_CS_PIN = gGpioPinStateLow_c;

#define DAC_VALUE_MAX       4095

/* Current Measurement Chip Select Management */
#define MEAS_CS_PIN         p5_4

#define MEAS_CS_DIS         MEAS_CS_PIN = gGpioPinStateHigh_c;
#define MEAS_CS_EN          MEAS_CS_PIN = gGpioPinStateLow_c;

/* Load Resistance Chip Select Management */
#define LOAD_RES_CS_PIN       p1_6

#define LOAD_RES_CS_DIS       LOAD_RES_CS_PIN = gGpioPinStateLow_c;
#define LOAD_RES_CS_EN        LOAD_RES_CS_PIN = gGpioPinStateHigh_c;

/* CMD Galv Management */
#define CMD_GALV_NEG        0x02
#define CMD_GALV_NEG_M        0x10
#define CMD_GALV_POS        0x04
#define CMD_GALV_POS_M        0x20
#define CMD_GALV_NONE       0x00

// A Modif
#define CMD_GALV_PORT       //p1
#define CMD_GALV_PIN_MASK     0x06

/* Detection Resistance Chip Select Management */
//#define DETECT_RES_CS_PIN     p3_4

#define DETECT_RES_CS_DIS    GPIO->P_CLR[SW_DETECT_PORT].DOUT = (1 << SW_DETECT_PIN)
#define DETECT_RES_CS_EN     GPIO->P_SET[SW_DETECT_PORT].DOUT = (1 << SW_DETECT_PIN)

/* Stim Superviser Management */


#define STIM_SUPERVIS_NTICK_IN_S      40    // Timer RB counter

#define STIM_GEN_TEST(mask)     TMR_RC_TEST_STATUS(mask)
#define STIM_GEN_RESET(mask)    TMR_RC_RESET_STATUS(mask)

#define STIM_GEN_MASK       BIT0

#define STIM_GEN_TRM_CLK_SOURCE_MHz   f1_CLK_SPEED_MHZ
#define STIM_GEN_TRM_CLK_SOURCE_10MHz   f1_CLK_SPEED_10MHZ

#define STIM_GEN_TRM_PERIOD_MAX   32768     // in �s*10^(-1)  //( ((uint32_t)(TMR_RC_COUNTER_MAX + 1)*DEF_TIME_NBR_uS_PER_SEC)/STIM_GEN_TRM_CLK_SOURCE )

#define STIM_GEN_AMPLITUDE_MIN    0       // in A*(10^-4)
#define STIM_GEN_AMPLITUDE_MAX    1000      // in A*(10^-4)


#define CMD_M_SET_POSITIVE_PULSE  Gpio_SetElectrostimulation(eETAPE3) // Set Haut L2-H1  CLR L1 /*CMD_M_PORT = POSITIVE_PULSE_CMD*/
#define CMD_M_SET_NEGATIVE_PULSE  Gpio_SetElectrostimulation(eETAPE8)//set Bas H2-L1  CLR L2 /*CMD_M_PORT = NEGATIVE_PULSE_CMD*/
#define CMD_M_SET_NO_PULSE     Gpio_SetElectrostimulation(eETAPE2) //Set L1-L2     /*CMD_M_PORT = NO_PULSE_CMD*/
#define CMD_M_DISCONNECT     Gpio_SetElectrostimulation(eETAPE1)   //CLR L1-L2-H1-H2     /*CMD_M_PORT = POSITIVE_PULSE_CMD*/



#define CLR_BRIDGE GPIO->P_CLR[CMD_H1_PORT].DOUT = ((1 << CMD_L2_PIN) | (1 << CMD_L1_PIN) | (1 << CMD_H2_PIN) |  (1 << CMD_H1_PIN))  //#define CLR_BRIDGE GPIO->P_CLR[CMD_H1_PORT].DOUT = ((1 << CMD_L2_PIN) | (1 << CMD_L1_PIN) | (1 << CMD_H2_PIN) |  (1 << CMD_H1_PIN))
#define POS_PULSE GPIO->P_SET[CMD_H1_PORT].DOUT = ((1 << CMD_H1_PIN) | (1 << CMD_L2_PIN))
#define NEG_PULSE GPIO->P_SET[CMD_H1_PORT].DOUT = ((1 << CMD_H2_PIN) | (1 << CMD_L1_PIN))
#define CMD_GALV_SEL_NONE GPIO->P_CLR[CMD_GV_P_PORT].DOUT = ((1 << CMD_GV_P_PIN) | (1 << CMD_GV_N_PIN))
#define CMD_GALV_SEL_NEG GPIO->P_SET[CMD_GV_N_PORT].DOUT = (1 << CMD_GV_N_PIN)
#define CMD_GALV_SEL_POS GPIO->P_SET[CMD_GV_P_PORT].DOUT = (1 << CMD_GV_P_PIN)

/* Switching Management */
#define SWITCH_FORCE_COUNT_STOP   TMR_RA_FORCE_COUNT_STOP
#define SWITCH_START       {/*TIMER_CounterSet(TIMER0,0);*/TIMER_Enable(TIMER_GEN_COURANT,true);}
#define SWITCH_STOP        {TIMER_Enable(TIMER_GEN_COURANT,false);/*TIMER_CounterSet(TIMER0,0);*/}

#define STIM_GEN_RELOAD_NEXT_COUNT(val)   Timer_SetMft1Timming(val)
#define STIM_GEN_RESET_COUNT         TIMER_CounterSet(TIMER_GEN_COURANT,0)   // Timer RB counter


#define DAC_VALUE_MAX       4095
#define STIM_GEN_AMPLITUDE_MIN    0       // in A*(10^-4)
#define STIM_GEN_AMPLITUDE_MAX    1000      // in A*(10^-4)

/* Stim Superviser Management */
#define STIM_SUPERVIS_FORCE_COUNT_STOP    TIMER_Enable(TIMER_ENV,false)
#define STIM_SUPERVIS_START         TIMER_Enable(TIMER_ENV,true)
#define STIM_SUPERVIS_STOP         TIMER_Enable(TIMER_ENV,false)

#define STIM_SUPERVIS_RELOAD_COUNT(val)   set_timer1_time(val)
#define STIM_SUPERVIS_RESET_COUNT     set_timer1_time(255)  // Timer RB counter

/* Stim Generation Management */
#define STIM_GEN_START        TIMER_Enable(TIMER_GEN_COURANT,true)
#define STIM_GEN_STOP       TIMER_Enable(TIMER_GEN_COURANT,false)

#define STIM_OUT_0            0x02
#define STIM_OUT_1            0x40
#define STIM_OUT_SEL(cmd)     {\
  if(cmd == STIM_OUT_0){\
      STIM_OUT_SEL_NONE;\
      GPIO_PinOutSet(CS_VOIE1_PORT,CS_VOIE1_PIN);\
  }\
  else{\
      STIM_OUT_SEL_NONE;\
      GPIO_PinOutSet(CS_VOIE2_PORT,CS_VOIE2_PIN);\
  }\
}\

#define STIM_OUT_CLR(cmd)     {\
  if(cmd == STIM_OUT_0){\
      STIM_OUT_SEL_NONE;\
            GPIO_PinOutClear(CS_VOIE1_PORT,CS_VOIE1_PIN);\
  }\
  else{\
      STIM_OUT_SEL_NONE;\
      GPIO_PinOutClear(CS_VOIE2_PORT,CS_VOIE2_PIN);\
  }\
}\

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/
/** Stim Generation Error Id */
typedef enum
{
  gStimGenErrNoError_c = 0,
  gStimGenErrPulseWidth_c,
  gStimGenErrSetDigAmplOutIdMax_c,
  gStimGenErrSetDigAmplitudeMax_c,
  gStimGenErrAdhElectrod_c,
  gStimGenErrMax_c
} StimGenErr_t;

/** Stimulation Out Id */
typedef enum
{
  gStimOut0_c = 0,
  gStimOut1_c,
  gStimOutMax_c
} StimOutId_t;

/** Stim Pattern Id */
typedef enum
{
  gStimPatternMonophasic_c = 1,
  gStimPatternGalvanic_c = 2,
//  gStimPatternSemiSinusMonophasic_c,
//  gStimPatternSemiSinusDiphas_c,
//  gStimPatternSinus_c,
  gStimPatternNeuro_c = 0x9,
  gStimPatternBiphasic_c = 0xB,
//  gStimPatternVeineux_c = 0x10,
  gStimPatternVeineuxBiphasic_c = 0x11,
//  gStimPatternMonophasicSingle_c = 0x12,
  gStimPatternBiphasicAltern_c = 0x13,
  gStimPatternBiphasicNegative_c = 0x14,

  gStimPatternMax_c,
} StimPatternId_t;

/** Stimulation Pattern Structure. */
typedef struct
{
  StimOutId_t   outId;            /**< Active stimulation output Id */
  uint16_t    width;            /**< Pattern width in �s */
  uint16_t    amplitude;          /**< Pattern amplitude in A*10-4 */

} Pattern_t;

/** Stimulation Generation Parameters */
typedef struct
{
  StimPatternId_t   patternId;        /**< Stimulation pattern Id */
  Pattern_t     tPattern[gStimOutMax_c];  /**< Stimulation pattern structure */
  uint8_t       nStim;          /**< Number of active stimulation outputs */
  uint32_t      frequency;        /**< Stimulation pattern frequency (Hz). */
  int8_t        FreqDiff;       /**< Frequencies are different. */
  uint8_t       Ratio;          /**< Frequencies ratio. */

} StimulationConfiguration_t;

/** Task Action Return. */
typedef struct
{
	uint8_t fsmState;
	bool_t bDataReturn;
	bool_t bReturn;
	uint8_t data[SRL_COMM_DATA_SIZE_MAX];
} FsmTaskActionReturn_t;

/** Stimulation Pattern Structure. */
typedef struct
{
  StimOutId_t   outId;
  uint16_t    cntWidth;
  uint16_t    digitalAmplitude;
  uint16_t    digitalMeasAmplitude;
} Pulse_t;

/** Stimulation Generation Structure. */
typedef struct
{
  StimPatternId_t   stimGenPatternId;
  Pulse_t       tPulse[gStimOutMax_c];
  uint8_t       nPulse;
  uint16_t      nTimerLoop;         /**< */
  uint16_t      cntTr;            /**< PulseLow */
  int8_t        FreqDiff;         /**< Frequencies are different */
  uint8_t       Ratio;            /**< Frequencies ratio. */

} StimulationGeneration_t;
/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
extern uint16_t gDigAmplMeas[2];
extern bool_t gflag[2];

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/
extern void StimManagementHacheurInit(void);
extern StimGenErr_t StimManagementSetDigitalAmplitude(uint16_t amplitude, uint8_t pulseId);
extern StimGenErr_t StimManagementConfigPulse(StimulationConfiguration_t* pStimConfig_t);
bool_t ElectrodeAdhesionDetection(StimOutId_t OutId);


#endif    /*  STIM_CONFIG_H_INCLUDED  */
