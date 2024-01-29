/**
 * @file   Biofeedback.c
 * @ingroup  GrpAppl
 * @brief  Stimulation tasks.
 *
 * Copyright : 2009 Vivaltis.\n
 *         All Rights Reserved
 */

/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/

/**********************************************************************************
System Includes
***********************************************************************************/

/**********************************************************************************
User Includes
***********************************************************************************/
#include "biofeedback.h"
#include "bsp.h"
#include "flash.h"
#include "bio_management.h"

// #include "Flash.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define BIO_ABS_ON DEF_ON
#define BIO_ABS_OFF DEF_OFF

#define BIO_ACQUISITION_FREQUENCY_MAX 100 /**< in Hz */
#define BIO_RETURN_FREQUENCY_MAX 20       /**< in Hz */

#define BIO_CUTOFF_FREQUENCY_MAX 1000 /**< in Hz */
#define BIO_CUTOFF_FREQUENCY_DEFAULT BIO_CUTOFF_FREQUENCY_MAX
#define BIO_FACTOR_ADJUST 100000 /**< Adjustment factor of the filter coefficient */ // 100000

#ifdef TEST_MODE_1_BIO2
#define BIO_FILTERED_VOLTAGE_BUFF_SIZE_MAX 20
#endif
#define BIO_PROCESSED_VOLTAGE_BUFF_SIZE_MAX 50

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
/** Biofeedback gain Id */
typedef enum BioGainId_tag
{
  gBioGain1_c = 0,
  gBioGain2_c,
  gBioGain3_c,
  gBioGainMax_c

} BioGainId_t;

/** Biofeedback state Id */
typedef enum BioState_tag
{
  gBioStart_c = 0,
  gBioRun_c,
  gBioStop_c,
  gBioStateMax_c

} BioState_t;

/** Biofeedback acquisition structure. Defines acquisition parameters */
typedef struct BioAcquisition_tag
{
  uint32_t filterCoef;   /**< Digital filter coefficient */
  uint8_t voltageFactor; /**< Voltage factor adjustment. Adjust the filter accuracy */
#ifdef TEST_MODE_1_BIO2
  int32_t filteredVoltage[BIO_FILTERED_VOLTAGE_BUFF_SIZE_MAX]; /**< Filtered voltage */
#endif
  int16_t processedVoltage[BIO_PROCESSED_VOLTAGE_BUFF_SIZE_MAX]; /**< Processed voltage to return */

  uint16_t offset; /**< Biofeedback offset to apply */
} BioAcquisition_t;

/** Biofeedback structure */
typedef struct
{
  uint16_t acquisitionPeriod; /**< Acquisition period of voltage to return */
  bool_t bAbs;                /**< Rectification enabler */
  BioAcquisition_t acquisition[gBioInMax_c];
  uint8_t nBio;                 /**< Number of enabled acquisition input. */
  uint16_t returnPeriod;        /**< Return period of processed voltage packet */
  BioState_t bioState_c;        /**< State of biofeedback */
  uint8_t inputId[gBioInMax_c]; /**< Biofeedback input identifier list */

} Biofeedback_t;

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
Biofeedback_t gBiofeedback_t;
#ifdef TEST_MODE_1_BIO1
uint8_t nTest0 = 0;
#endif

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/************************************************************************************
 * Name :  BiofeedbackInit */
/**
 * @brief  Init the biofeedback structure.
 * @param  .
 * @return .
 ************************************************************************************/
void BiofeedbackInit(void)
{
  flash_user_data_t flashData = {0};

#ifdef TEST_MODE_1_BIO_OFFSET
  uint16_t BioOffset[gBioInMax_c];
#endif

  /** Reset the number of active bio inputs */
  gBiofeedback_t.acquisitionPeriod = 0;
  gBiofeedback_t.bAbs = BIO_ABS_ON;
  gBiofeedback_t.returnPeriod = 1;

#ifdef TEST_MODE_1_BIO_OFFSET
  flash_Open(BLOCK_DEF);
  for (i = 0; i < gBioInMax_c; i++)
  {
    BioOffset[i] = 3;
  }
  flash_WriteData(BLOCK_DEF, OFFSET, (BUF_PTR_TYPE)&BioOffset, sizeof(uint32_t));
  flash_Close(BLOCK_DEF);
#endif

  flash_init();
  flash_read_user_data(&flashData);
  gBiofeedback_t.acquisition[0].offset = flashData.offSetBio[0];
  gBiofeedback_t.acquisition[1].offset = flashData.offSetBio[1];

  if (gBiofeedback_t.acquisition[0].offset == 0xFFFF)
    gBiofeedback_t.acquisition[0].offset = 100;
  if (gBiofeedback_t.acquisition[1].offset == 0xFFFF)
    gBiofeedback_t.acquisition[1].offset = 100;
  BiofeedbackStop();
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :  BiofeedbackConfig */
/**
 * @brief  Configures the biofeedback acquisition.
 * @param  pConfigData a pointer refers to the configuration datas
 * @return The error of operation.
 ************************************************************************************/
BioErr_t BiofeedbackConfig(const BioConfigData_t *pConfigData)
{
  BioSelCutoffFrequencyData_t bioSelCutoffFreqData_t = {0};

  gBiofeedback_t.bioState_c = gBioStop_c;

  /** Asserts that at least one biofeedback input is available */
  if (gBiofeedback_t.nBio >= gBioInMax_c)
    gBiofeedback_t.nBio--;

  /** Gets the bio input Id */
  if ((BioInputId_t)pConfigData->inputId >= gBioInMax_c)
    return gBioErrConfigInIdMax_c;

  gBiofeedback_t.inputId[gBiofeedback_t.nBio] = pConfigData->inputId;

  /** If no Biofeedback input is configured */
  if (gBiofeedback_t.nBio == 0)
  {
    /** - Tests if bio acquisition is rectified */
    if (pConfigData->bAbs)
      gBiofeedback_t.bAbs = BIO_ABS_ON;
    else
      gBiofeedback_t.bAbs = BIO_ABS_OFF;

    /** - Gets acquisition frequency */
    if (pConfigData->acquisitionFrequency > BIO_ACQUISITION_FREQUENCY_MAX) // TODO : Fix critical value !!!!!
      return gBioErrConfigAcquisitonFrequencyMax_c;
    else
      gBiofeedback_t.acquisitionPeriod = (uint16_t)BIO_SAMPLING_FREQUENCY / pConfigData->acquisitionFrequency;

    /** - Calculates acquisition return period */
    if (pConfigData->acquisitionFrequency > BIO_RETURN_FREQUENCY_MAX)
      gBiofeedback_t.returnPeriod = pConfigData->acquisitionFrequency / BIO_RETURN_FREQUENCY_MAX;
    else
      gBiofeedback_t.returnPeriod = 1;
  }
  else
  {
    /** - Asserts bio acquisition rectifications are the same */
    if (gBiofeedback_t.bAbs != pConfigData->bAbs)
      return gBioErrConfigAbsDiff_c;

    /** - Asserts bio acquisition frequencies are the same */
    if (gBiofeedback_t.acquisitionPeriod != BIO_SAMPLING_FREQUENCY / pConfigData->acquisitionFrequency)
      return gBioErrConfigFrequencyDiff_c;
  }

  bioSelCutoffFreqData_t.cutoffFrequency = BIO_CUTOFF_FREQUENCY_DEFAULT;
  bioSelCutoffFreqData_t.inputId = pConfigData->inputId;

  BiofeedbackSelectCutOffFrequency(&bioSelCutoffFreqData_t);

  /** Valids configuration and actives biofeedback channel */
  if (BioManagementEnableInput(gBiofeedback_t.inputId, gBiofeedback_t.nBio + 1))
    return gBioErrConfigEnableInput_c;

  gBiofeedback_t.nBio++;

  return gBioErrNoError_c;
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :  BiofeedbackStart  */
/**
 * @brief  Starts the biofeedback acquisition.
 * @param
 * @return The error of operation.
 ************************************************************************************/
BioErr_t BiofeedbackStart(uint8_t *pNBio)
{
  uint16_t k = 0;
  int16_t curVoltage[gBioInMax_c] = {0};
  /** Tests if biofeedback is configured */
  if (gBiofeedback_t.nBio)
  {
    *pNBio = gBiofeedback_t.nBio;
    gBiofeedback_t.bioState_c = gBioStart_c;

    BIO_SAMPLING_START;

    for (k = 0; k < 40000; k++)
    {
      if (BioManagementGetMeas(curVoltage))
      {
      }
    }

    return gBioErrNoError_c;
  }
  else
  {
    gBiofeedback_t.bioState_c = gBioStop_c;
    return gBioErrStartNotConfigured_c;
  }
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :  BiofeedbackStop */
/**
 * @brief  Stops the biofeedback acquisition.
 * @param
 * @return The error of operation.
 ************************************************************************************/
BioErr_t BiofeedbackStop(void)
{
  gBiofeedback_t.bioState_c = gBioStop_c;
  gBiofeedback_t.nBio = 0;

  BIO_SAMPLING_STOP;
  BioManagementDisableAllInput();
  // BioManagementInit();

  return gBioErrNoError_c;
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :  BiofeedbackSelectGain */
/**
 * @brief  Selects the biofeedback gain. Adapts the maximal voltage.
 * @param  pBioSelGainData a pointer refers to the gain selection datas
 * @return The error of operation.
 ************************************************************************************/
BioErr_t BiofeedbackSelectGain(const BioSelGainData_t *pBioSelGainData)
{
  /** Tests the bio input Id */
  if ((BioInputId_t)pBioSelGainData->inputId >= gBioInMax_c)
    return gBioErrSelGainInIdMax_c;

  /** Tests the bio gain Id */
  if ((BioGainId_t)pBioSelGainData->gain >= gBioGainMax_c)
    return gBioErrSelGainMax_c;

  switch (pBioSelGainData->gain)
  {
  case gBioGain1_c:
    BIO_CMD_SET_G1;
    break;

  case gBioGain2_c:
    BIO_CMD_SET_G2;
    break;

  case gBioGain3_c:
    BIO_CMD_SET_G3;
    break;

  default:
    break;
  }

  return gBioErrNoError_c;
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :  BiofeedbackSelectCutOffFrequency  */
/**
 * @brief  Sets the biofeedback Filter Frequency for the specified Input. Calculate the filter coefficient.
 * @param  pBioSelCutoffFreqData_t a pointer refers to the gain selection datas
 * @return The error of operation.
 ************************************************************************************/
BioErr_t BiofeedbackSelectCutOffFrequency(const BioSelCutoffFrequencyData_t *pBioSelCutoffFreqData_t)
{
  uint8_t bioInputId_t = 0;

  /** Tests the bio input Id. */
  if ((BioInputId_t)pBioSelCutoffFreqData_t->inputId >= gBioInMax_c)
    return gBioErrSelCutoffFreqInIdMax_c;

  bioInputId_t = pBioSelCutoffFreqData_t->inputId;

  /** Tests the bio cutoff frequency Id. */
  if (pBioSelCutoffFreqData_t->cutoffFrequency > BIO_CUTOFF_FREQUENCY_MAX)
    return gBioErrSelCutoffFreqMax_c;

  gBiofeedback_t.acquisition[bioInputId_t].filterCoef = ((uint32_t)2 * 314 * pBioSelCutoffFreqData_t->cutoffFrequency) / (uint32_t)BIO_SAMPLING_FREQUENCY_kHZ;

  if (pBioSelCutoffFreqData_t->cutoffFrequency < 10)
    gBiofeedback_t.acquisition[bioInputId_t].voltageFactor = 100;
  else
  {
    if (pBioSelCutoffFreqData_t->cutoffFrequency < 100)
      gBiofeedback_t.acquisition[bioInputId_t].voltageFactor = 10;
    else
      gBiofeedback_t.acquisition[bioInputId_t].voltageFactor = 1;
  }

  return gBioErrNoError_c;
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :  BiofeedbackSelectOffset */
/**
 * @brief  Sets the biofeedback offset for the specified Input.
 * @param  pBioSelOffsetData_t a pointer refers to the offset selection datas
 * @return The error of operation.
 ************************************************************************************/
BioErr_t BiofeedbackSelectOffset(const BioSelOffsetData_t *pBioSelOffsetData_t)
{
  uint8_t bioInputId_t = 0;
  uint8_t k = 0;
  uint16_t BioOffset[gBioInMax_c] = {0};
  flash_user_data_t flashData = {0};

  /** Tests the bio input Id. */
  if ((BioInputId_t)pBioSelOffsetData_t->inputId >= gBioInMax_c)
    return gBioErrSelOffsetInIdMax_c;

  bioInputId_t = pBioSelOffsetData_t->inputId;

  BioOffset[bioInputId_t] = pBioSelOffsetData_t->offset;
  gBiofeedback_t.acquisition[bioInputId_t].offset = BioOffset[bioInputId_t];

  flash_init();
  flash_read_user_data(&flashData);                            // Read stored data
  flashData.offSetBio[bioInputId_t] = BioOffset[bioInputId_t]; // Store new offset for desired input

  k = (bioInputId_t + 1) % 2;
  BioOffset[k] = flashData.offSetBio[k]; // Get offset for the other channel
  flash_write_data(flashData);           // Store new value
  return gBioErrNoError_c;
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :  BiofeedbackExecute  */
/**
 * @brief  Gets the biofeedback voltages.
 * @param  pData8 a pointer refers to .
 * @param  nData a pointer refers to .
 * @param  nDataMax .
 * @return a boolean indicating the function returns datas.
 ************************************************************************************/
bool_t BiofeedbackExecute(int16_t *pValue, uint8_t *pNValue, uint8_t nValueMax)
{
  uint16_t i = 0;
  uint16_t j = 0;
  bool_t b = FALSE;

  uint8_t nVoltageMax = 0;
  uint8_t inputId = 0;
  uint8_t voltageFactor = 0;
  uint16_t BioOffset[gBioInMax_c] = {0};

  /* Filter input an output values */
  int16_t curVoltage[gBioInMax_c] = {0};
  static int16_t prevVoltage[gBioInMax_c] = {0};
  static int32_t curFilteredVoltage[gBioInMax_c] = {0};
  static uint16_t remainingAcquisitionPeriod = 0;

  static uint16_t nAvailableProcessedVoltage = 0;
  static uint16_t readIdxProcessedVoltage = 0;
  static uint16_t writeIdxProcessedVoltage = 0;

#ifdef TEST_MODE_1_BIO2
  static uint16_t writeIdxFilteredVoltage;
#endif

  switch (gBiofeedback_t.bioState_c)
  {
  /** Start State */
  case gBioStart_c:

    /** - Initializes variables */
    remainingAcquisitionPeriod = gBiofeedback_t.acquisitionPeriod;

    nAvailableProcessedVoltage = 0;
    readIdxProcessedVoltage = 0;
    writeIdxProcessedVoltage = 0;

#ifdef TEST_MODE_1_BIO2
    writeIdxFilteredVoltage = 0;
#endif
    for (i = 0; i < gBiofeedback_t.nBio; i++)
    {
      inputId = gBiofeedback_t.inputId[i];
      prevVoltage[inputId] = 0;

      for (j = 0; j < BIO_PROCESSED_VOLTAGE_BUFF_SIZE_MAX; j++)
        gBiofeedback_t.acquisition[inputId].processedVoltage[j] = 0;

#ifdef TEST_MODE_1_BIO2
      for (j = 0; j < BIO_FILTERED_VOLTAGE_BUFF_SIZE_MAX; j++)
        gBiofeedback_t.acquisition[inputId].filteredVoltage[j] = 0;
#endif
    }

    /** - Goes to run state */
    gBiofeedback_t.bioState_c = gBioRun_c;
    break;

  /** Run State */
  case gBioRun_c:

    if (gBiofeedback_t.nBio) /** asserts at least one biofeedback is configured */
    {
      if (BioManagementGetMeas(curVoltage))
      {
#ifdef PIN_TEST_MODE_1_BIO
        p0_1 = 1; // pin test
#endif
        /** Processing */
        for (i = 0; i < gBiofeedback_t.nBio; i++)
        {
          inputId = gBiofeedback_t.inputId[i];
          voltageFactor = gBiofeedback_t.acquisition[inputId].voltageFactor;
          BioOffset[inputId] = gBiofeedback_t.acquisition[inputId].offset;

          if (gBiofeedback_t.bAbs)
          {
            if (curVoltage[inputId] < 0)
              curVoltage[inputId] = (-curVoltage[inputId]);

            /* filtering */
            curFilteredVoltage[inputId] = (int32_t)(curFilteredVoltage[inputId] + ((int32_t)gBiofeedback_t.acquisition[inputId].filterCoef * ((int32_t)voltageFactor * prevVoltage[inputId] - curFilteredVoltage[inputId]) / (int32_t)BIO_FACTOR_ADJUST));

            /* save current voltages values */
            prevVoltage[inputId] = curVoltage[inputId];
#ifdef TEST_MODE_1_BIO2
            gBiofeedback_t.acquisition[inputId].filteredVoltage[writeIdxFilteredVoltage] = (int16_t)(curFilteredVoltage[inputId]);
#endif
          }
          else
          {
          }
        }

#ifdef TEST_MODE_1_BIO2
        writeIdxFilteredVoltage = (writeIdxFilteredVoltage + 1) % BIO_FILTERED_VOLTAGE_BUFF_SIZE_MAX;
        if (writeIdxFilteredVoltage == BIO_FILTERED_VOLTAGE_BUFF_SIZE_MAX - 1)
          writeIdxFilteredVoltage = 0;
#endif

        /** Processed voltage acquisition  */
        remainingAcquisitionPeriod--;
        if (remainingAcquisitionPeriod == 0)
        {
          /* Reloads Counter */
          remainingAcquisitionPeriod = gBiofeedback_t.acquisitionPeriod;

          /* Gets data processed */
          if (nAvailableProcessedVoltage < BIO_PROCESSED_VOLTAGE_BUFF_SIZE_MAX)
          {
#ifdef TEST_MODE_1_BIO1
            if (nTest0 < nAvailableProcessedVoltage)
              nTest0 = nAvailableProcessedVoltage;
#endif
            for (i = 0; i < gBiofeedback_t.nBio; i++)
            {
              inputId = gBiofeedback_t.inputId[i];
              voltageFactor = gBiofeedback_t.acquisition[inputId].voltageFactor;

              if (gBiofeedback_t.bAbs)
              {
                if (curFilteredVoltage[inputId] < 0)
                {
                  gBiofeedback_t.acquisition[inputId].processedVoltage[writeIdxProcessedVoltage] = 0;
                }
                else
                {
                  gBiofeedback_t.acquisition[inputId].processedVoltage[writeIdxProcessedVoltage] = (curFilteredVoltage[inputId] / voltageFactor);
                }
              }
              else
              {
                gBiofeedback_t.acquisition[inputId].processedVoltage[writeIdxProcessedVoltage] = curVoltage[inputId];
              }
            }
            writeIdxProcessedVoltage = (writeIdxProcessedVoltage + 1) % BIO_PROCESSED_VOLTAGE_BUFF_SIZE_MAX;
            nAvailableProcessedVoltage++;
          }
        }
      }

#ifdef PIN_TEST_MODE_1_BIO
      p0_1 = 0; // pin test
#endif

      /** Processed voltages packet return  */
#ifndef TEST_MODE_1_BIO3
      if (EFM32_STOP_IS_DIS)
      {
#endif
        if (nAvailableProcessedVoltage >= gBiofeedback_t.returnPeriod)
        {
          nVoltageMax = nValueMax / gBiofeedback_t.nBio;
          if (nAvailableProcessedVoltage > nVoltageMax)
          {
            *pNValue = nVoltageMax;
            nAvailableProcessedVoltage -= nVoltageMax;
          }
          else
          {
            *pNValue = nAvailableProcessedVoltage;
            nAvailableProcessedVoltage = 0;
          }
          for (j = 0; j < *pNValue; j++)
          {
            for (i = 0; i < gBiofeedback_t.nBio; i++)
            {
              if (gBiofeedback_t.bAbs)
              {
                if (gBiofeedback_t.acquisition[gBiofeedback_t.inputId[i]].processedVoltage[readIdxProcessedVoltage] > BioOffset[gBiofeedback_t.inputId[i]])
                {
                  pValue[(j * gBiofeedback_t.nBio) + i] = gBiofeedback_t.acquisition[gBiofeedback_t.inputId[i]].processedVoltage[readIdxProcessedVoltage] - BioOffset[gBiofeedback_t.inputId[i]];
                }
                else
                {
                  pValue[(j * gBiofeedback_t.nBio) + i] = gBiofeedback_t.acquisition[gBiofeedback_t.inputId[i]].processedVoltage[readIdxProcessedVoltage];
                }
              }
              else
              {
                pValue[(j * gBiofeedback_t.nBio) + i] = gBiofeedback_t.acquisition[gBiofeedback_t.inputId[i]].processedVoltage[readIdxProcessedVoltage];
              }
            }
            readIdxProcessedVoltage = (readIdxProcessedVoltage + 1) % BIO_PROCESSED_VOLTAGE_BUFF_SIZE_MAX;
          }
          *pNValue *= gBiofeedback_t.nBio;
          b = TRUE;
        }
#ifndef TEST_MODE_1_BIO3
      }
      else
        b = FALSE;
#endif
    }
    break;

  case gBioStop_c:

    break;

  default:

    break;
  }

  return b;
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :  fct   */
/**
 * @brief  Description
 * @param  p description
 * @return description
 ************************************************************************************/
void fct(void)
{
}

/**********************************************************************************
End of function
***********************************************************************************/
