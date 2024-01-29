/**
* @file   Biofeedback.h
* @ingroup  GrpAppl
* @brief  Biofeedback header file.
*
* Copyright : 2009 Vivaltis.\n
*         All Rights Reserved
*/

/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/


#ifndef BIOFEEDBACK_H_INCLUDED
#define BIOFEEDBACK_H_INCLUDED

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


/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/
//#define N_VOLTAGE_DATA_MAX    5
#define BIO_SIZE_OF_VOLTAGE     2

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/
/** Biofeedback Error Id */
typedef enum
{
  gBioErrNoError_c = 0,
  gBioErrConfigBioMax_c,
  gBioErrConfigInIdMax_c,
  gBioErrConfigInIdEnabled_c,
  gBioErrConfigAcquisitonFrequencyMax_c,
//  gBioErrConfigFrequencyMax_c,
  gBioErrConfigEnableInput_c,
  gBioErrConfigAbsDiff_c,
  gBioErrConfigFrequencyDiff_c,
  gBioErrConfigBuffSizeDiff_c,

  gBioErrStartNotConfigured_c,

  gBioErrSelGainInIdMax_c,
  gBioErrSelGainMax_c,

  gBioErrSelCutoffFreqInIdMax_c,
  gBioErrSelCutoffFreqMax_c,

  gBioErrSelOffsetInIdMax_c,

  gBioErrMax_c
} BioErr_t;

/** Biofeedback Configuration Datas Structure */
typedef struct
{
  uint16_t  acquisitionFrequency;
  uint8_t   inputId;
  bool_t    bAbs;

} BioConfigData_t;

/** Biofeedback Gain Selection Datas Structure */
typedef struct
{
  uint8_t   gain;
  uint8_t   inputId;

} BioSelGainData_t;

/** Biofeedback Filter Cut-Off Frequency Selection Datas Structure */
typedef struct
{
  uint16_t  cutoffFrequency;
  uint8_t   inputId;
} BioSelCutoffFrequencyData_t;

/** Biofeedback offset selection datas structure */
typedef struct
{ uint16_t  offset;
  uint8_t   inputId;
} BioSelOffsetData_t;

/** Biofeedback Voltages Datas Structure */
/*typedef struct
{
//  uint8_t   nInput;
  uint8_t   nVoltage;
  int16_t   bioVoltage[gBioInMax_c * N_VOLTAGE_DATA_MAX];

} BioReturnData_t;
*/


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
void BiofeedbackInit(void);
BioErr_t BiofeedbackConfig(const BioConfigData_t* pConfigData);
BioErr_t BiofeedbackStart(uint8_t* pNBio);
BioErr_t BiofeedbackStop(void);
BioErr_t BiofeedbackSelectGain(const BioSelGainData_t* pBioSelGainData);
BioErr_t BiofeedbackSelectCutOffFrequency(const BioSelCutoffFrequencyData_t* pBioSelCutoffFreqData_t);
BioErr_t BiofeedbackSelectOffset(const BioSelOffsetData_t* pBioSelOffsetData_t);
//bool_t BiofeedbackExecute(BioReturnData_t* pBioData_t, uint8_t nDataMax);
bool_t BiofeedbackExecute(int16_t* pValue, uint8_t* pNValue, uint8_t nValueMax);

#endif    /*  BIOFEEDBACK_H_INCLUDED  */
