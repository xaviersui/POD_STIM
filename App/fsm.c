/**
 * @file   fsm.c
 * @ingroup  GrpAppl
 * @brief  Finite State Machine drivers.
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
#include "includes.h"
#include "fsm.h"
#include "siue.h"
#include "stimulation.h"
#include "flash.h"
#include "spi.h"
#include <string.h>
/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define FSM_STATE_DEFAULT gFsmStateInitialization_c

#define FSM_ID_SIZE 1
#define REQUEST_ID_SIZE 1

#define BIO_CONFIGURATION_DATA_SIZE 5
#define BIO_SET_GAIN_DATA_SIZE 2

#define N_REQUEST_GET_MAX 10

#define FSM_INIT_FRAME_SIZE_MAX 10
#define FSM_STIM_FRAME_SIZE_MAX 40
#define FSM_BIO_FRAME_SIZE_MAX 10
#define FSM_MNGT_FRAME_SIZE_MAX 10

#define GET_FSM_STATE_ID(u8buff) 1

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
/** Main Finite State Machine States.*/
typedef enum
{
  gFsmStateInitialization_c = 0, /**< FSM Initialization State */
  gFsmStateManagement_c,         /**< FSM Management State */
  gFsmStateStimulation_c,        /**< FSM Stimulation State */
  gFsmStateBiofeedback_c         /**< FSM Biofeedback State */
} FsmState_t;

/** Task Actions. Successive executed actions in a task. */
typedef enum
{
  gFsmTaskStart_c = 0,    /**< Start Action. Get from Serial Comm. the FSM State Id */
  gFsmTaskGetRequest_c,   /**< Get from Serial Comm. the Request Id */
  gFsmTaskExecRequest_c,  /**< Execute Request Action. Get from Serial Comm. the Request Datas and execute the request */
  gFsmTaskExecMainTask_c, /**< Execute Main Task Action. Execute the main task at every loop */
  gFsmTaskStop_c          /**< Stop Action. Stop task execution loop and exit. */
} FsmTaskAction_t;

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
#ifdef TEST_MODE_2_BIO2
nTxRqst = 0;
#endif

/***********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
static void FsmTaskInitialization(FsmState_t *fsmStateId, bool_t *bFSMStateChangePending);
static void FsmTaskStimulation(FsmState_t *fsmStateId, bool_t *bFSMStateChangePending);
static void FsmTaskBiofeedback(FsmState_t *fsmStateId, bool_t *bFSMStateChangePending);
static void FsmTaskManagement(FsmState_t *fsmStateId, bool_t *bFSMStateChangePending);
static bool_t FsmGetExternCommData(uint8_t *pFsmTaskFrame, uint8_t nDataSize);

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
bool_t hasNotifToSendV1 = FALSE;
bool_t hasNotifToSendV2 = FALSE;
bool_t hasNotifElectToSend = FALSE;
bool_t hasNotifDelayToSend = FALSE;
/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/************************************************************************************
 * Name :  FsmRun  */
/**
 * @brief  Runs the state machine.
 * @param  .
 * @return .
 ************************************************************************************/
void FsmRun(void)
{
  bool_t bFSMStateChangePending;
  FsmState_t fsmState;

#ifdef TEST_MODE_2_BIO1
  fsmState = gFsmStateBiofeedback_c;
#else
#ifdef TEST_MODE_2_STIM01
  fsmState = gFsmStateStimulation_c;

// #ifdef TEST_MODE_2_MANAGEMENT
//   fsmState = gFsmStateManagement_c;
#else
  fsmState = gFsmStateInitialization_c;
#endif
#endif
  bFSMStateChangePending = FALSE;

  // Finite state machine is an "endless" loop.
  // It only exits if the FSM_STOP state is required.
  for (;;)
  {
    switch (fsmState)
    {
    case gFsmStateInitialization_c:

      FsmTaskInitialization(&fsmState, &bFSMStateChangePending);
      break;

    case gFsmStateManagement_c:

      FsmTaskManagement(&fsmState, &bFSMStateChangePending);
      break;

    case gFsmStateStimulation_c:

      FsmTaskStimulation(&fsmState, &bFSMStateChangePending);
      break;

    case gFsmStateBiofeedback_c:

      FsmTaskBiofeedback(&fsmState, &bFSMStateChangePending);
      break;

    default:

      fsmState = gFsmStateInitialization_c;
      bFSMStateChangePending = FALSE;
    }
  }
  // If a state change is pending, execute it.
  // This can happen because the callback has found an external event
  // that causes a state change, or because a state has decided it
  // lasted long enough
}
/**********************************************************************************
End of function
***********************************************************************************/

/******************************************************************************/ /**
 FsmTaskInitialization Function Name
 Description:  Executes the Fsm Task Initialization function.
 Parameters:   none
 Return value:   none
 ***********************************************************************************/
void FsmTaskInitialization(FsmState_t *fsmStateId, bool_t *bFSMStateChangePending)
{
  SrlCommErr_t srlCommErr = gSrlCommErrNoError_c;
  uint8_t fsmTaskFrame[FSM_INIT_FRAME_SIZE_MAX];
  uint8_t nRetry = 0;
  FsmTaskAction_t fsmTaskAction, fsmTaskNextAction;

  bool_t bRetry = FALSE;

  typedef enum
  {
    gTaskInitRqstTest_c = 0,

  } TaskInitRequest_t;

  TaskInitRequest_t tTaskInitRequest;

  if (*bFSMStateChangePending)
  {
    fsmTaskAction = gFsmTaskGetRequest_c;
  }
  else
  {
    fsmTaskAction = gFsmTaskStart_c;
  }

  fsmTaskNextAction = gFsmTaskStart_c;
  tTaskInitRequest = gTaskInitRqstTest_c;

  // Init if necessary
  for (;;)
  {
    // p0_1 = !p0_1;
    switch (fsmTaskAction)
    {
    case gFsmTaskStart_c:

      // TODO : WatchDog Rearm

      // Wait for a frame start (FSM_STATE_ID)
      if (!FsmGetExternCommData(fsmTaskFrame, FSM_ID_SIZE))
        fsmTaskAction = gFsmTaskExecMainTask_c; // Remain in this action and Wait for more data
      else
      {
        if (*fsmStateId != (FsmState_t)(*(uint8_t *)fsmTaskFrame)) // See if Received data is the good one
        {
          *fsmStateId = (FsmState_t)(*(uint8_t *)fsmTaskFrame);
          fsmTaskAction = gFsmTaskStop_c;
        }
        else
        {
          fsmTaskAction = gFsmTaskGetRequest_c;
          nRetry = 0;
        }
      }
      break;

    case gFsmTaskGetRequest_c:

      if (FsmGetExternCommData(fsmTaskFrame, REQUEST_ID_SIZE))
      {
        tTaskInitRequest = (TaskInitRequest_t)(*(uint8_t *)fsmTaskFrame);
        fsmTaskAction = gFsmTaskExecRequest_c;
        nRetry = 0;
      }
      else
      {
        nRetry++;
        if (nRetry < N_REQUEST_GET_MAX)
          fsmTaskNextAction = gFsmTaskGetRequest_c;
        else
        {
          fsmTaskNextAction = gFsmTaskStart_c;
          SrlCommManagmntFlushBuffRx();
          // TODO : SIGNAL error
        }

        fsmTaskAction = gFsmTaskExecMainTask_c;
      }

      break;

    case gFsmTaskExecRequest_c:

      switch (tTaskInitRequest)
      {
      case gTaskInitRqstTest_c:

        break;

      default:

        SrlCommManagmntFlushBuffRx();
        ErrorManagement((uint16_t)srlCommErr);

        break;
      }

      if (bRetry)
      {
        bRetry = FALSE;
        nRetry++;
        if (nRetry < N_REQUEST_GET_MAX)
          fsmTaskNextAction = gFsmTaskExecRequest_c;
        else
        {
          fsmTaskNextAction = gFsmTaskStart_c;
          SrlCommManagmntFlushBuffRx();
          // TODO : SIGNAL error
        }

        fsmTaskAction = gFsmTaskExecMainTask_c;
      }
      else
      {
        fsmTaskNextAction = gFsmTaskStart_c;    // Remain in this action and Wait for more data
        fsmTaskAction = gFsmTaskExecMainTask_c; // Remain in this action and Wait for more data
      }

      break;

    case gFsmTaskExecMainTask_c:

      /** if error go to stop. */
      if (0)
      {
        *fsmStateId = FSM_STATE_DEFAULT;
        fsmTaskAction = gFsmTaskStop_c;
        break;
      }

      fsmTaskAction = fsmTaskNextAction;

      break;

    case gFsmTaskStop_c:

      *bFSMStateChangePending = TRUE;
      return; // Exit FSM State

    default:
      ErrorManagement((uint16_t)gSrlCommErrNoError_c);
    }
  }
}
/**********************************************************************************
End of function
***********************************************************************************/

/**********************************************************************************
Function Name:  FsmTaskStimulation
Description:  Executes Fsm State Stimulation function.
Parameters:   none
Return value:   none
***********************************************************************************/
void FsmTaskStimulation(FsmState_t *fsmStateId, bool_t *bFSMStateChangePending)
{
	SrlCommErr_t srlCommErr = gSrlCommErrNoError_c;
	uint8_t fsmTaskFrame[FSM_STIM_FRAME_SIZE_MAX];
	uint8_t len, nRetry = 0;
	bool_t bRetry = FALSE, ActivityFlag = FALSE;
	StimConfigData_t *pStimConfigData_t;
	StimSetLevelData_t *pStimSetLevelData_t;
#ifdef V016
	StimSetFrequencyData_t *pStimSetFrequencyData_t;
	StimSetPulseWidthData_t *pStimSetPulseWidthData_t;
#endif
	FsmTaskAction_t fsmTaskAction, fsmTaskNextAction;
	FsmTaskActionReturn_t fsmTaskReturn_t;
	FsmTaskActionReturn_t fsmTaskNotif_t[4];

	StimErr_t stimErr_c;
	StimErr_t *pStimErr_c;

	typedef enum
	{
		gTaskStimRqstConfig_c = 0,
		gTaskStimRqstStart_c,
		gTaskStimRqstSetLevel_c,
		gTaskStimRqstStop_c,
		gTaskStimRqstResume_c,
		gTaskStimRqstPause_c,
#ifdef V016
		gTaskStimRqstSetFrequency_c = 7,
		gTaskStimRqstSetPulseWidth_c = 8,
#endif
		gTaskStimRqstNotif = 9

	} TaskStimRequest_t;

	TaskStimRequest_t taskStimRequest_c;

	fsmTaskReturn_t.fsmState = (uint8_t)*fsmStateId;

	if (*bFSMStateChangePending)
		fsmTaskAction = gFsmTaskGetRequest_c;
	else
		fsmTaskAction = gFsmTaskStart_c;

	fsmTaskNextAction = gFsmTaskStart_c;

	taskStimRequest_c = gTaskStimRqstStop_c;
	// Init if necessary

	for (;;)
	{
		switch (fsmTaskAction)
		{
		case gFsmTaskStart_c:

			// TODO : WatchDog Rearm

			// Wait for a frame start (FSM_STATE_ID)
			if (!FsmGetExternCommData(fsmTaskFrame, FSM_ID_SIZE))
				fsmTaskAction = gFsmTaskExecMainTask_c; // Remain in this action and Wait for more data
			else
			{
				if (*fsmStateId != (FsmState_t)(*(uint8_t *)fsmTaskFrame)) // See if Received data is the good one
				{
					*fsmStateId = (FsmState_t)(*(uint8_t *)fsmTaskFrame);
					fsmTaskAction = gFsmTaskStop_c;
				}
				else
				{
					fsmTaskAction = gFsmTaskGetRequest_c;
					nRetry = 0;
				}
			}
			break;

		case gFsmTaskGetRequest_c:

			if (FsmGetExternCommData(fsmTaskFrame, REQUEST_ID_SIZE))
			{
				taskStimRequest_c = (TaskStimRequest_t)(*(uint8_t *)fsmTaskFrame);
				fsmTaskAction = gFsmTaskExecRequest_c;
				nRetry = 0;
			}
			else
			{
				nRetry++;
				if (nRetry < N_REQUEST_GET_MAX)
					fsmTaskNextAction = gFsmTaskGetRequest_c;
				else
				{
					fsmTaskNextAction = gFsmTaskStart_c;
					SrlCommManagmntFlushBuffRx();
					// TODO : SIGNAL error
				}

				fsmTaskAction = gFsmTaskExecMainTask_c;
			}
			break;

		case gFsmTaskExecRequest_c:

			switch (taskStimRequest_c)
			{
			case gTaskStimRqstNotif:
				if (hasNotifElectToSend == TRUE)
				{
					SrlCommManagmntWriteData((uint8_t *)&fsmTaskNotif_t[2], 5);
					hasNotifElectToSend = FALSE;
					break;
				}
				if (hasNotifDelayToSend == TRUE)
				{
					SrlCommManagmntWriteData((uint8_t *)&fsmTaskNotif_t[3], 5);
					hasNotifElectToSend = FALSE;
					break;
				}
				if (hasNotifToSendV1 == TRUE)
				{
					SrlCommManagmntWriteData((uint8_t *)&fsmTaskNotif_t[0], 5);
					hasNotifToSendV1 = FALSE;
					break;
				}
				if (hasNotifToSendV2 == TRUE)
				{
					SrlCommManagmntWriteData((uint8_t *)&fsmTaskNotif_t[1], 5);
					hasNotifToSendV2 = FALSE;
					break;
				}
				break;
			case gTaskStimRqstConfig_c:

				if (FsmGetExternCommData(fsmTaskFrame, sizeof(StimConfigData_t)))
				{
					pStimConfigData_t = (StimConfigData_t *)fsmTaskFrame;
					ENDIAN_UINT16(pStimConfigData_t->patternFrequencyA);
					ENDIAN_UINT16(pStimConfigData_t->pulseWidthA);
					ENDIAN_UINT16(pStimConfigData_t->patternFrequencyB);
					ENDIAN_UINT16(pStimConfigData_t->pulseWidthB);
					ENDIAN_UINT16(pStimConfigData_t->patternFrequencyC);
					ENDIAN_UINT16(pStimConfigData_t->pulseWidthC);
					ENDIAN_UINT16(pStimConfigData_t->curAmplitude);
					ENDIAN_UINT16(pStimConfigData_t->maxAmplitude);

					stimErr_c = StimulationConfig(pStimConfigData_t);

					if (stimErr_c == gStimErrNoError_c)
					{
						fsmTaskReturn_t.bReturn = FALSE;
						fsmTaskReturn_t.data[0] = (uint8_t)taskStimRequest_c;
					}
					else
					{
						fsmTaskReturn_t.bReturn = TRUE;
						pStimErr_c = (StimErr_t *)fsmTaskReturn_t.data;
						*pStimErr_c = stimErr_c;
						ENDIAN_UINT16(*pStimErr_c);
					}

					fsmTaskReturn_t.bDataReturn = FALSE;

					len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX + 2;
					SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, len);
				}
				else
					bRetry = TRUE;
				break;

			case gTaskStimRqstStart_c:

				StimulationStart();

				ActivityFlag = TRUE;

				fsmTaskReturn_t.bDataReturn = FALSE;
				fsmTaskReturn_t.bReturn = FALSE;
				fsmTaskReturn_t.data[0] = (uint8_t)taskStimRequest_c;
				len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX; // MODIF XSU len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX + 1
				SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, len);
				break;

			case gTaskStimRqstSetLevel_c:

				if (FsmGetExternCommData(fsmTaskFrame, sizeof(StimSetLevelData_t)))
				{
					pStimSetLevelData_t = (StimSetLevelData_t *)fsmTaskFrame;
					if (pStimSetLevelData_t->level)
						stimErr_c = StimulationSetLevel(pStimSetLevelData_t->stimOutId, TRUE);
					else
					{
						stimErr_c = StimulationSetLevel(pStimSetLevelData_t->stimOutId, FALSE);
					}

					if (stimErr_c == gStimErrNoError_c)
						fsmTaskReturn_t.bReturn = FALSE;
					else
					{
						fsmTaskReturn_t.bReturn = TRUE;
						pStimErr_c = (StimErr_t *)fsmTaskReturn_t.data;
						*pStimErr_c = stimErr_c;
						ENDIAN_UINT16(*pStimErr_c);
					}

					fsmTaskReturn_t.bDataReturn = FALSE;

					len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX;
					SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, len);
				}
				else
					bRetry = TRUE;
				break;
#ifdef V016
			case gTaskStimRqstSetFrequency_c:

				if (FsmGetExternCommData(fsmTaskFrame, sizeof(StimSetFrequencyData_t)))
				{
					pStimSetFrequencyData_t = (StimSetFrequencyData_t *)fsmTaskFrame;
					stimErr_c = StimulationSetFrequency(pStimSetFrequencyData_t->stimOutId,
														pStimSetFrequencyData_t->frequencyA,
														pStimSetFrequencyData_t->frequencyB,
														pStimSetFrequencyData_t->frequencyC);

					if (stimErr_c == gStimErrNoError_c)
						fsmTaskReturn_t.bReturn = FALSE;
					else
					{
						fsmTaskReturn_t.bReturn = TRUE;
						pStimErr_c = (StimErr_t *)fsmTaskReturn_t.data;
						*pStimErr_c = stimErr_c;
						ENDIAN_UINT16(*pStimErr_c);
					}

					fsmTaskReturn_t.bDataReturn = FALSE;

					len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX;
					SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, len);
				}
				else
					bRetry = TRUE;

				break;

			case gTaskStimRqstSetPulseWidth_c:

				if (FsmGetExternCommData(fsmTaskFrame, sizeof(StimSetPulseWidthData_t)))
				{
					pStimSetPulseWidthData_t = (StimSetPulseWidthData_t *)fsmTaskFrame;
					stimErr_c = StimulationSetPulseWidth(pStimSetPulseWidthData_t->stimOutId,
														 pStimSetPulseWidthData_t->pulseWidthA,
														 pStimSetPulseWidthData_t->pulseWidthB,
														 pStimSetPulseWidthData_t->pulseWidthC);

					if (stimErr_c == gStimErrNoError_c)
						fsmTaskReturn_t.bReturn = FALSE;
					else
					{
						fsmTaskReturn_t.bReturn = TRUE;
						pStimErr_c = (StimErr_t *)fsmTaskReturn_t.data;
						*pStimErr_c = stimErr_c;
						ENDIAN_UINT16(*pStimErr_c);
					}

					fsmTaskReturn_t.bDataReturn = FALSE;

					len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX;
					SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, len);
				}
				else
					bRetry = TRUE;
				break;
#endif
			case gTaskStimRqstStop_c:

				StimulationStop();

				ActivityFlag = FALSE;

				fsmTaskReturn_t.bDataReturn = FALSE;
				fsmTaskReturn_t.bReturn = FALSE;
				fsmTaskReturn_t.data[0] = (uint8_t)taskStimRequest_c;
				len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX; // MODIF XSU len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX + 1;
				SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, len);
				break;

			case gTaskStimRqstResume_c:

				StimulationResume();

				fsmTaskReturn_t.bDataReturn = FALSE;
				fsmTaskReturn_t.bReturn = FALSE;
				fsmTaskReturn_t.data[0] = (uint8_t)taskStimRequest_c;
				len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX; // MODIF XSU len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX + 1;
				SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, len);
				break;

			case gTaskStimRqstPause_c:

				StimulationPause();

				fsmTaskReturn_t.bDataReturn = FALSE;
				fsmTaskReturn_t.bReturn = FALSE;
				fsmTaskReturn_t.data[0] = (uint8_t)taskStimRequest_c;
				len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX; // MODIF XSU len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX + 1;
				SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, len);
				break;

			default:

				SrlCommManagmntFlushBuffRx();
				ErrorManagement((uint16_t)srlCommErr);

				break;
			}

			if (bRetry)
			{
				bRetry = FALSE;
				nRetry++;
				if (nRetry < N_REQUEST_GET_MAX)
					fsmTaskNextAction = gFsmTaskExecRequest_c;
				else
				{
					fsmTaskNextAction = gFsmTaskStart_c;
					SrlCommManagmntFlushBuffRx();
					// TODO : SIGNAL error
				}

				fsmTaskAction = gFsmTaskExecMainTask_c;
			}
			else
			{
				fsmTaskNextAction = gFsmTaskStart_c;	// Remain in this action and Wait for more data
				fsmTaskAction = gFsmTaskExecMainTask_c; // Remain in this action and Wait for more data
			}

			break;

			/** Go To Next Action. azerty */

		case gFsmTaskExecMainTask_c:

			/** Execute Main Task. */
			if (StimulationExecute(fsmTaskNotif_t))
			{
			}
#ifndef TEST_MODE
			if (!ActivityFlag)
			{
				// A remplacer par I2C -- Debut
				/*CONFIG_DAC_SPI;
				SPI_CONFIG_TRANSMIT;
				DAC_CS_EN;

				SPI_TRANSMIT_DATA(0x00);
				SPI_TRANSMIT_DATA(0x00);

				while (tend_sssr != 1)
					;

				SPI_DISABLE_CONFIG;
				DAC_CS_DIS;*/
				// A remplacer par I2C -- Fin
			}
#endif
			/** if error go to stop. */
			if (0)
			{
				*fsmStateId = FSM_STATE_DEFAULT;
				fsmTaskAction = gFsmTaskStop_c;
				break;
			}

			fsmTaskAction = fsmTaskNextAction;
			break;

		case gFsmTaskStop_c:

			*bFSMStateChangePending = TRUE;
			StimulationStop();

			return; // Exit FSM State

		default:
			ErrorManagement((uint16_t)gSrlCommErrNoError_c);
		}
	}
}
/**********************************************************************************
End of function
***********************************************************************************/

/**********************************************************************************
Function Name:  FsmTaskBiofeedback
Description:  Executes Fsm State Biofeedback function.
Parameters:   none
Return value:   none
***********************************************************************************/
void FsmTaskBiofeedback(FsmState_t *fsmStateId, bool_t *bFSMStateChangePending)
{
  SrlCommErr_t srlCommErr = gSrlCommErrNoError_c;
  uint8_t fsmTaskFrame[FSM_BIO_FRAME_SIZE_MAX] = {0};
  uint8_t i, len, nRetry = 0, nValMax, nBio;
  bool_t bRetry = FALSE;

  FsmTaskAction_t fsmTaskAction, fsmTaskNextAction;
  FsmTaskActionReturn_t fsmTaskReturn_t;

  BioErr_t bioErr;
  BioErr_t *pBioErr_c;

  BioConfigData_t *pBioConfigData_t;
  BioSelCutoffFrequencyData_t *pBioSelCutoffFreqData_t;
  BioSelOffsetData_t *pBioSelOffsetData_t;

  typedef enum
  {
    gTaskBioRqstConfig_c = 0,
    gTaskBioRqstStart_c,
    gTaskBioRqstSetGain_c,
    gTaskBioRqstSetCutoffFrequency_c,
    gTaskBioRqstSetOffset_c,
    gTaskBioRqstStop_c

  } TaskBioRequest_t;

  TaskBioRequest_t taskBioRequest_c;

  typedef struct
  {
    uint8_t nValue;
    int16_t bioVoltage[SRL_COMM_DATA_SIZE_MAX / BIO_SIZE_OF_VOLTAGE];

  } BioReturnData_t;

  BioReturnData_t *pBioReturnData_t;

  fsmTaskReturn_t.fsmState = (uint8_t)*fsmStateId;
  nValMax = (SRL_COMM_DATA_SIZE_MAX - (sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX) - sizeof(nValMax)) / BIO_SIZE_OF_VOLTAGE;

  if (*bFSMStateChangePending)
    fsmTaskAction = gFsmTaskGetRequest_c;
  else
    fsmTaskAction = gFsmTaskStart_c;

  fsmTaskNextAction = gFsmTaskStart_c;
  taskBioRequest_c = gTaskBioRqstStop_c;
  // Init if necessary

#ifdef TEST_MODE_2_BIO_OFFSET
  fsmTaskAction = gFsmTaskExecRequest_c;
  taskBioRequest_c = gTaskBioRqstSetOffset_c;
  bioSelOffsetData_t.inputId = 0;
  bioSelOffsetData_t.offset = 25;
#endif

  for (;;)
  {
    switch (fsmTaskAction)
    {
    case gFsmTaskStart_c:

      // TODO : WatchDog Rearm
      // Wait for a frame start (FSM_STATE_ID)
      if (!FsmGetExternCommData(fsmTaskFrame, FSM_ID_SIZE))
        fsmTaskAction = gFsmTaskExecMainTask_c; // Remain in this action and Wait for more data
      else
      {
        if (*fsmStateId != (FsmState_t)(*(uint8_t *)fsmTaskFrame)) // See if Received data is the good one
        {
          *fsmStateId = (FsmState_t)(*(uint8_t *)fsmTaskFrame);
          fsmTaskAction = gFsmTaskStop_c;
        }
        else
        {
          fsmTaskAction = gFsmTaskGetRequest_c;
          nRetry = 0;
        }
      }
      break;

    case gFsmTaskGetRequest_c:

      if (FsmGetExternCommData(fsmTaskFrame, REQUEST_ID_SIZE))
      {
        taskBioRequest_c = (TaskBioRequest_t)(*(uint8_t *)fsmTaskFrame);
        fsmTaskAction = gFsmTaskExecRequest_c;
        nRetry = 0;
      }
      else
      {
        nRetry++;
        if (nRetry < N_REQUEST_GET_MAX)
          fsmTaskNextAction = gFsmTaskGetRequest_c;
        else
        {
          fsmTaskNextAction = gFsmTaskStart_c;
          SrlCommManagmntFlushBuffRx();
          // TODO : SIGNAL error
        }

        fsmTaskAction = gFsmTaskExecMainTask_c;
      }

      break;

    case gFsmTaskExecRequest_c:

      len = 0;

      switch (taskBioRequest_c)
      {
      case gTaskBioRqstConfig_c:

        if (FsmGetExternCommData(fsmTaskFrame, sizeof(BioConfigData_t)))
        {
          pBioConfigData_t = (BioConfigData_t *)fsmTaskFrame;
          ENDIAN_UINT16(pBioConfigData_t->acquisitionFrequency);
          bioErr = BiofeedbackConfig(pBioConfigData_t);
          if (bioErr == gBioErrNoError_c)
            fsmTaskReturn_t.bReturn = TRUE;
          else
          {
            fsmTaskReturn_t.bReturn = FALSE;
            pBioErr_c = (BioErr_t *)fsmTaskReturn_t.data;
            *pBioErr_c = bioErr;
            len = sizeof(bioErr);
          }
          fsmTaskReturn_t.bDataReturn = FALSE;
        }
        else
          bRetry = TRUE;

        break;

      case gTaskBioRqstStart_c:

        bioErr = BiofeedbackStart(&nBio);
        if (bioErr == gBioErrNoError_c)
        {
          fsmTaskReturn_t.bReturn = TRUE;
          fsmTaskReturn_t.data[0] = nBio;
          len = sizeof(nBio);
        }
        else
        {
          fsmTaskReturn_t.bReturn = FALSE;
          pBioErr_c = (BioErr_t *)fsmTaskReturn_t.data;
          *pBioErr_c = bioErr;
          len = sizeof(bioErr);
        }
        fsmTaskReturn_t.bDataReturn = FALSE;
        break;

      case gTaskBioRqstSetGain_c:

        if (FsmGetExternCommData(fsmTaskFrame, sizeof(BioSelGainData_t)))
        {
          bioErr = BiofeedbackSelectGain((BioSelGainData_t *)fsmTaskFrame);
          if (bioErr == gBioErrNoError_c)
            fsmTaskReturn_t.bReturn = TRUE;
          else
          {
            fsmTaskReturn_t.bReturn = FALSE;
            pBioErr_c = (BioErr_t *)fsmTaskReturn_t.data;
            *pBioErr_c = bioErr;
            len = sizeof(bioErr);
          }
          fsmTaskReturn_t.bDataReturn = FALSE;
        }
        else
          bRetry = TRUE;

        break;

      case gTaskBioRqstSetCutoffFrequency_c:

        if (FsmGetExternCommData(fsmTaskFrame, sizeof(BioSelCutoffFrequencyData_t) - 1))
        {
          pBioSelCutoffFreqData_t = (BioSelCutoffFrequencyData_t *)fsmTaskFrame;
          ENDIAN_UINT16(pBioSelCutoffFreqData_t->cutoffFrequency);
          bioErr = BiofeedbackSelectCutOffFrequency(pBioSelCutoffFreqData_t);
          if (bioErr == gBioErrNoError_c)
            fsmTaskReturn_t.bReturn = TRUE;
          else
          {
            fsmTaskReturn_t.bReturn = FALSE;
            pBioErr_c = (BioErr_t *)fsmTaskReturn_t.data;
            *pBioErr_c = bioErr;
            len = sizeof(bioErr);
          }
          fsmTaskReturn_t.bDataReturn = FALSE;
        }
        else
          bRetry = TRUE;

        break;

      case gTaskBioRqstSetOffset_c:

        if (FsmGetExternCommData(fsmTaskFrame, sizeof(BioSelOffsetData_t) - 1))
        {
          pBioSelOffsetData_t = (BioSelOffsetData_t *)fsmTaskFrame;
          ENDIAN_UINT16(pBioSelOffsetData_t->offset);
          bioErr = BiofeedbackSelectOffset(pBioSelOffsetData_t);
          if (bioErr == gBioErrNoError_c)
            fsmTaskReturn_t.bReturn = TRUE;
          else
          {
            fsmTaskReturn_t.bReturn = FALSE;
            pBioErr_c = (BioErr_t *)fsmTaskReturn_t.data;
            *pBioErr_c = bioErr;
            len = sizeof(bioErr);
          }
          fsmTaskReturn_t.bDataReturn = FALSE;
        }
        else
          bRetry = TRUE;

        break;

      case gTaskBioRqstStop_c:

        bioErr = BiofeedbackStop();
        if (bioErr == gBioErrNoError_c)
          fsmTaskReturn_t.bReturn = TRUE;
        else
        {
          fsmTaskReturn_t.bReturn = FALSE;
          pBioErr_c = (BioErr_t *)fsmTaskReturn_t.data;
          *pBioErr_c = bioErr;
          len = sizeof(bioErr);
        }
        fsmTaskReturn_t.bDataReturn = FALSE;

        break;

      default:

        SrlCommManagmntFlushBuffRx();
        ErrorManagement((uint16_t)srlCommErr);

        break;
      }
#ifdef TEST_MODE_2_BIO2
      nTxRqst++;
#endif
      len = sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX + len;
      SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, len);

      if (bRetry)
      {
        bRetry = FALSE;
        nRetry++;
        if (nRetry < N_REQUEST_GET_MAX)
          fsmTaskNextAction = gFsmTaskExecRequest_c;
        else
        {
          fsmTaskNextAction = gFsmTaskStart_c;
          SrlCommManagmntFlushBuffRx();
          // TODO : SIGNAL error
        }

        fsmTaskAction = gFsmTaskExecMainTask_c;
      }
      else
      {
        fsmTaskNextAction = gFsmTaskStart_c;    // Remain in this action and Wait for more data
        fsmTaskAction = gFsmTaskExecMainTask_c; // Remain in this action and Wait for more data
      }

      break;

    case gFsmTaskExecMainTask_c:

      /** Execute Main Task. */
      pBioReturnData_t = (BioReturnData_t *)fsmTaskReturn_t.data;
      if (BiofeedbackExecute(pBioReturnData_t->bioVoltage, &pBioReturnData_t->nValue, nValMax))
      {
#ifdef PIN_TEST_MODE_2_BIO1
#endif
        for (i = 0; i < fsmTaskReturn_t.data[0]; i++)
        {
          ENDIAN_UINT16(pBioReturnData_t->bioVoltage[i]);
        }
        memcpy(fsmTaskReturn_t.data + 1,pBioReturnData_t->bioVoltage,2*pBioReturnData_t->nValue);
        fsmTaskReturn_t.bDataReturn = TRUE;
        fsmTaskReturn_t.bReturn = TRUE;

        SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX + sizeof(nValMax) + (pBioReturnData_t->nValue * BIO_SIZE_OF_VOLTAGE));
#ifdef PIN_TEST_MODE_2_BIO1
        p0_3 = 0;
#endif
      }

      /** if error go to stop. */
      if (0)
      {
        *fsmStateId = FSM_STATE_DEFAULT;
        fsmTaskAction = gFsmTaskStop_c;
        break;
      }

      fsmTaskAction = fsmTaskNextAction;

      break;

    case gFsmTaskStop_c:

      bioErr = BiofeedbackStop();
      *bFSMStateChangePending = TRUE;
      return; // Exit FSM State

    default:
      ErrorManagement((uint16_t)gSrlCommErrNoError_c);
    }
  }
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :  FsmManagement */
/**
 * @brief  Execute the FSM Management function.
 * @param  None.
 * @return None.
 ************************************************************************************/
void FsmTaskManagement(FsmState_t *fsmStateId, bool_t *bFSMStateChangePending)
{
  uint8_t fsmTaskFrame[FSM_MNGT_FRAME_SIZE_MAX];
  uint8_t nRetry = 0, i = 0;
  uint8_t Number_Channel = 2;

  FsmTaskAction_t fsmTaskAction, fsmTaskNextAction;
  FsmTaskActionReturn_t fsmTaskReturn_t;

  typedef enum
  {
    //    gTaskMngtRqstTarage,
    gTaskMngtRqstId = 0x03,
    //    gTaskMngtSerialStore,
    //    gTaskMngtSerialRead,
    gTaskMngtVersionWrite,
    gTaskMngtVersionRead,
  } TaskMngtRequest_t;

  typedef enum
  {
    gPodType_StimBio = 1,
    gPodType_Stim,
    gPodType_Bio,
  } PodType_t;

  typedef enum
  {
    gChannelType_NoTool = 0,
    gChannelType_StimBio,
  } ChannelType_t;

  flash_user_data_t stFlashData = {0};
  uint8_t versionId_t[6];

  TaskMngtRequest_t taskMngtRequest_c;

  fsmTaskReturn_t.fsmState = (uint8_t)*fsmStateId;

  if (*bFSMStateChangePending)
    fsmTaskAction = gFsmTaskGetRequest_c;
  else
    fsmTaskAction = gFsmTaskStart_c;

  fsmTaskNextAction = gFsmTaskStart_c;

  taskMngtRequest_c = gTaskMngtRqstId;

#ifdef TEST_MODE_1_MANAGEMENT
  fsmTaskAction = gFsmTaskExecRequest_c;
  taskMngtRequest_c = gTaskMngtVersionRead;
#endif

#ifdef SET_VERSION_ID
  fsmTaskAction = gFsmTaskExecRequest_c;
  taskMngtRequest_c = gTaskMngtVersionWrite;

  versionId_t[0] = 0x56; // V
  versionId_t[1] = 0x30; // 0
  versionId_t[2] = 0x31; // 1
  versionId_t[3] = 0x2E; // .
  versionId_t[4] = 0x34; // 1
  versionId_t[5] = 0x30; // 2
#endif

#ifdef GET_VERSION_ID
  fsmTaskAction = gFsmTaskExecRequest_c;
  taskMngtRequest_c = gTaskMngtVersionRead;
#endif

  for (;;)
  {
    switch (fsmTaskAction)
    {
    case gFsmTaskStart_c:
      // Wait for a frame start (FSM_STATE_ID)
      if (!FsmGetExternCommData(fsmTaskFrame, FSM_ID_SIZE))
        fsmTaskAction = gFsmTaskExecMainTask_c; // Remain in this action and Wait for more data
      else
      {
        if (*fsmStateId != (FsmState_t)(*(uint8_t *)fsmTaskFrame)) // See if Received data is the good one
        {
          *fsmStateId = (FsmState_t)(*(uint8_t *)fsmTaskFrame);
          fsmTaskAction = gFsmTaskStop_c;
        }
        else
        {
          fsmTaskAction = gFsmTaskGetRequest_c;
          nRetry = 0;
        }
      }
      break;

    case gFsmTaskGetRequest_c:
      if (FsmGetExternCommData(fsmTaskFrame, REQUEST_ID_SIZE))
      {
        taskMngtRequest_c = (TaskMngtRequest_t)(*(uint8_t *)fsmTaskFrame);
        fsmTaskAction = gFsmTaskExecRequest_c;
        nRetry = 0;
      }
      else
      {
        nRetry++;
        if (nRetry < N_REQUEST_GET_MAX)
          fsmTaskNextAction = gFsmTaskGetRequest_c;
        else
        {
          fsmTaskNextAction = gFsmTaskStart_c;
          SrlCommManagmntFlushBuffRx();
        }
        fsmTaskAction = gFsmTaskExecMainTask_c;
      }
      break;

    case gFsmTaskExecRequest_c:
      switch (taskMngtRequest_c)
      {
      case gTaskMngtRqstId: // Send how many tools are present and their action (Stim, Bio, Stim/Bio).
        fsmTaskReturn_t.bDataReturn = TRUE;
        for (i = 0; i < Number_Channel; i++)
        {
          fsmTaskReturn_t.bReturn = FALSE;            /** Electrodes are disconnected. */
          fsmTaskReturn_t.data[0] = (PodType_t) gPodType_StimBio; /** Error code for electrodes adhesion. */
          fsmTaskReturn_t.data[1] = Number_Channel;   /** Channel i is disconnected. */
          fsmTaskReturn_t.data[i + 2] = (ChannelType_t) gChannelType_NoTool;
        }

        // Send frame via UART.
        SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, 7); // sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX);
        break;

      case gTaskMngtVersionWrite:
    	 flash_init();
    	 flash_read_user_data(&stFlashData);
    	 memcpy(stFlashData.versionId_t,versionId_t,sizeof(versionId_t));
    	 flash_write_data(stFlashData);
    	 break;

      case gTaskMngtVersionRead:
#ifdef TEST_MODE_1_MANAGEMENT
        fsmTaskReturn_t.bReturn = FALSE;            /** Electrodes are disconnected. */
        fsmTaskReturn_t.data[0] = gPodType_StimBio; /** Error code for electrodes adhesion. */
        fsmTaskReturn_t.data[1] = Number_Channel;   /** Channel i is disconnected. */
        fsmTaskReturn_t.data[i + 2] = gChannelType_NoTool;

        SrlCommManagmntWriteData((uint8_t *)&fsmTaskReturn_t, 7); // sizeof(FsmTaskActionReturn_t) - SRL_COMM_DATA_SIZE_MAX);
#else
        flash_init();
        flash_read_user_data(&stFlashData);
#endif
        break;
      }

      fsmTaskNextAction = gFsmTaskStart_c;    // Remain in this action and Wait for more data
      fsmTaskAction = gFsmTaskExecMainTask_c; // Remain in this action and Wait for more data
      break;

    case gFsmTaskExecMainTask_c:
      fsmTaskAction = fsmTaskNextAction;
      break;

    case gFsmTaskStop_c:
      *bFSMStateChangePending = TRUE;
      return;

    default:
      ErrorManagement((uint16_t)gSrlCommErrNoError_c);
    }
  }
}

/**********************************************************************************
End of function
***********************************************************************************/

/**********************************************************************************
Function Name:  FsmGetExternCommData
Description:
Parameters:   none
Return value:   none
***********************************************************************************/
bool_t FsmGetExternCommData(uint8_t *pFsmTaskFrame, uint8_t nDataSize)
{
  SrlCommErr_t srlCommErr;
  uint8_t nDataRcvd;

  srlCommErr = SrlCommManagmntReadData(pFsmTaskFrame, nDataSize, &nDataRcvd);
  if (srlCommErr != gSrlCommErrNoError_c) // Critical Error
  {
    SrlCommManagmntFlushBuffRx();
    ErrorManagement((uint16_t)srlCommErr);
    return FALSE;
  }

  if (nDataRcvd < nDataSize)
    return FALSE;
  else
    return TRUE;
}
/**********************************************************************************
End of function
***********************************************************************************/
