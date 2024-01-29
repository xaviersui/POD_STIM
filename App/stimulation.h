/**
* @file   Stimulation.h
* @ingroup  GrpAppl
* @brief  Stimulation header file.
*
* Copyright : 2009 Vivaltis.\n
*         All Rights Reserved
*/

/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/


#ifndef STIMULATION_H_INCLUDED
#define STIMULATION_H_INCLUDED

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
#include "siue.h"
#include "stim_management.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/


/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define N_MODULATION_MAX 4 /**< Maximum number of modulation by envelope */

#define STIM_ELECTRODE_ADH_DETECT_ON 1
#define STIM_ELECTRODE_ADH_DETECT_OFF 0

#define STIM_MNGMNT_GET_RESOURCE DISABLE_IRQ
#define STIM_MNGMNT_RELEASE_RESOURCE ENABLE_IRQ

#define CONFIG_DAC_SPI              \
	do                              \
	{                               \
		SET_DAC_SPI_CLOCK_PHASE;    \
		SET_DAC_SPI_CLOCK_POLARITY; \
	} while (0)

#define STIM_OUT_NONE 0x00
#define STIM_OUT_PORT p3
#define STIM_OUT_PIN_MASK 0x42
#define STIM_OUT_SEL_NONE GPIO_PinOutClear(CS_VOIE1_PORT, CS_VOIE1_PIN);

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
/** Modulation Types Id */
typedef enum
{
	gModulDelay_c = 0,
	gModulLinIncrease_c, /**< Amplitude linear increase */
	gModulMaxAmp_c,		 /**< Maximal amplitude */
	gModulLinDecrease_c, /**< Amplitude linear decrease */
	gModulNullAmp_c,	 /**< Null amplitude */
	gModulMax_c			 /**< undefined */
} ModulationID_t;

/** Modulation Structure. Defines amplitude modulation parameters */
typedef struct
{
	ModulationID_t id; /**< Modulation Type */
	uint16_t period;   /**< Modulation Period */
	uint16_t curTime;  /**< Modulation Current Time during execution */
} Modulation_t;

/** Envelope Structure. Define envelope parameters */
typedef struct
{
	uint8_t nModulation; /**< Number of Modulation. */
	bool_t wobulationEnabled;
	Modulation_t tModulation[N_MODULATION_MAX]; /**< Modulation array. Contains the modulations list */
	uint16_t patternFrequencyA;
	uint16_t pulseWidthA;
	uint16_t patternFrequencyB;
	uint16_t pulseWidthB;
	uint16_t patternFrequencyC;
	uint16_t pulseWidthC;
	uint16_t delay;		   /**< Envelope Delay. Delay before the envelope starts */
	uint16_t curAmplitude; /**< Amplitude. */
	uint16_t maxAmplitude; /**< Maximum Amplitude. */
	uint16_t amplitudeIncrement;
#ifdef V016
	uint8_t nRepetition; /**< Number of envelope repetition. */
#endif
} Envelope_t;

/** Stimulation Structure. */
typedef struct
{
	Envelope_t tEnvelope;
	bool_t electrodeAdhesionDetect;

} StimulationControl_t;

/** Stimulation Control Structure */
typedef struct
{
	bool_t startEn;
	bool_t pauseEn;
	StimulationControl_t tControl[gStimOutMax_c];
	StimulationConfiguration_t tConfig;
} Stimulation_t;

typedef struct
{
	uint8_t OutId;
	uint8_t Step;
} StimStep_c;
/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/
/** Stim Error Id */
typedef enum
{
  gStimErrNoError_c = 0,
  gStimErrConfigStimMax_c,
  gStimErrConfigStimOutIdMax_c,
  gStimErrConfigStimOutIdActive_c,
  gStimErrConfigStimMode_c,
  gStimErrConfigStimPatternIdMax_c,
  gStimErrConfigStimPatternIdDiff_c,
  gStimErrConfigStimFrequencyDiff_c,
  gStimErrConfigStimPulseWidthMax_c,
  gStimErrConfigStimNoModulation_c,
  gStimErrConfigStimInvalidPulse_c,
  gStimErrConfigStimWobulationDiff_c,
  gStimErrConfigStimNoWobulation_c,
  gStimErrSetLevelOutIdMax_c,
  gStimErrSetLevelIncreaseId_c,
  gStimErrMax_c
} StimErr_t;

/**
 * \defgroup StimFrame Stimulation Frame
 */
/*@{*/

/** Stimulation Configuration Datas Structure */
typedef struct
{
  uint8_t   stimOutId;
  uint8_t   stimPatternId;
  bool_t    bElectrodeAdhesionDetect; /**< Electrode adhesion test. Active or not the electrode adhesion detection */
  uint8_t   delayPeriod;        /**< Delay Period */
  uint8_t   increasePeriod;       /**< Increase Period */
  uint8_t   plateauPeriod;        /**< Plateau Period */
  uint8_t   decreasePeriod;       /**< Decrease Period */
  uint8_t   reposePeriod;       /**< Repose Period */
  uint16_t  patternFrequencyA;      /**< Stimulation pattern frequency (Hz). */
  uint16_t  pulseWidthA;
  uint16_t  patternFrequencyB;      /**< Stimulation pattern frequency (Hz). */
  uint16_t  pulseWidthB;
  uint16_t  patternFrequencyC;      /**< Stimulation pattern frequency (Hz). */
  uint16_t  pulseWidthC;
  uint16_t  curAmplitude;       /**< Amplitude. */
  uint16_t  maxAmplitude;       /**< Maximum Amplitude. */
  uint8_t   amplitudeIncrement;
#ifdef V016
  uint8_t   envelopeRepeat;       /**< Number of envelope repetition. */
#endif

} StimConfigData_t;

/** Stimulation level Setting Datas Structure */
typedef struct
{
  uint8_t   stimOutId;
  bool_t    level;

} StimSetLevelData_t;
#ifdef V016
/** Stimulation level Setting Datas Structure */
#pragma pack(1)
typedef struct
{
  uint8_t   stimOutId;
  uint16_t  frequencyA;
  uint16_t  frequencyB;
  uint16_t  frequencyC;

} StimSetFrequencyData_t;

/** Stimulation level Setting Datas Structure */
#pragma pack(1)
typedef struct
{
  uint8_t   stimOutId;
  uint16_t  pulseWidthA;
  uint16_t  pulseWidthB;
  uint16_t  pulseWidthC;

} StimSetPulseWidthData_t;
#endif

/*@}*/


/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/
void StimulationInit(void);
StimErr_t StimulationConfig(StimConfigData_t* pConfigData);
StimErr_t StimulationStart(void);
StimErr_t StimulationStop(void);
StimErr_t StimulationSetLevel(uint8_t pulseId, bool_t bIncrease);
#ifdef V016
  StimErr_t StimulationSetFrequency(uint8_t pulseId, uint16_t FreqA, uint16_t FreqB, uint16_t FreqC);
  StimErr_t StimulationSetPulseWidth(uint8_t pulseId, uint16_t PWA, uint16_t PWB, uint16_t PWC);
#endif
StimErr_t StimulationResume(void);
StimErr_t StimulationPause(void);

bool_t StimulationExecute(FsmTaskActionReturn_t fsmTaskNotif_t[]);

//bool_t ElectrodeAdhesionDetection(StimOutId_t OutId);

bool_t ChannelTest(void);

#endif    /*  STIMULATION_H_INCLUDED  */
