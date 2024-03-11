/**
 * @file		stimulation.c
 * @ingroup	GrpAppl
 * @brief	Stimulation tasks.
 *
 * Copyright : 2009 Vivaltis.\n
 * 				All Rights Reserved
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
#include "stimulation.h"
#include "Bsp.h"
#include "srl_comm_management.h"
#include "stim_management.h"
#include "Spi.h"

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
Stimulation_t gStim_t;	  /**< */
static volatile bool gStimTick; /**< */
uint8_t nElectrodDetachmentCpt[2] = {0};
bool_t bElecTest[2] = {0}, bReprise = FALSE, bInstVesic = FALSE;
extern uint16_t        gDigAmplMeas[gStimOutMax_c];
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
extern bool_t hasNotifToSendV1;
extern bool_t hasNotifToSendV2;
extern bool_t hasNotifElectToSend;
extern bool_t hasNotifDelayToSend;
uint8_t nbVoie = 0;
bool_t voie1Active = FALSE;

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/************************************************************************************
 * Name :	StimulationInit	*/
/**
 * @brief	Init the stimulation structure.
 * @param	.
 * @return	.
 ************************************************************************************/
void StimulationInit(void)
{
	/** Reset the number of active stim ouputs */
	gStim_t.tConfig.nStim = 0;

	/** TODO : Init/Reset stim command */
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :	StimulationStart	*/
/**
* @brief	Starts the stimulation generation and control.

* @return	The error of operation.
************************************************************************************/
StimErr_t StimulationStart(void)
{
	uint8_t i = 0;

	gStim_t.startEn = TRUE;
	gStim_t.pauseEn = FALSE;
	if (gStim_t.tConfig.nStim)
	{
		if (gStim_t.tConfig.patternId != 0x09)
		{
			for (i = 0; i < gStim_t.tConfig.nStim; i++)
				bElecTest[i] = TRUE;
		}

		// A Modif
		if (gStim_t.tConfig.patternId == 0x11)
			gflag[0] = TRUE;

		STIM_SUPERVIS_START;
		//GPIO_PinOutSet(CMD_110V_ON_OFF_PORT, CMD_110V_ON_OFF_PIN);
		//GPIO_PinOutSet(ON_OFF_BOOSTER_PORT, ON_OFF_BOOSTER_PIN); // 110V on
		STIM_GEN_START;
	}

	return gStimErrNoError_c;
}

/************************************************************************************
 * Name :	StimulationStop	*/
/**
 * @brief	Stops the stimulation generation and control.
 * @param
 * @return	The error of operation.
 ************************************************************************************/
StimErr_t StimulationStop(void)
{
	uint8_t i = 0;
// A Modif
	//GPIO_PinOutClear(CMD_110V_ON_OFF_PORT, CMD_110V_ON_OFF_PIN);
	//GPIO_PinOutClear(ON_OFF_BOOSTER_PORT, ON_OFF_BOOSTER_PIN); //  110V Off

	STIM_SUPERVIS_STOP;
	STIM_SUPERVIS_RESET_COUNT;

  STIM_GEN_STOP;
	 STIM_GEN_RESET_COUNT;

	if ((gStim_t.tConfig.patternId == 0x09) || (gStim_t.tConfig.patternId == 0x02))
	{
		// A Modif
		STIM_OUT_SEL_NONE;
		// CMD_GALV_SEL_NONE;
	}
  // Application AOP -> OFF
  Gpio_ClrAop();

  (void)Ad5691r_SetIntensiteStimulation(0); //exprimer en uV

	for (i = 0; i < gStimOutMax_c; i++)
	{
		// A Modif
		StimManagementSetDigitalAmplitude(0, i);

		gStim_t.tConfig.tPattern[i].width = 0;
		// A Modif
		StimManagementConfigPulse(&gStim_t.tConfig);
		bElecTest[i] = FALSE;
	}

	gStim_t.tConfig.nStim = 0;
	if (gStim_t.tConfig.patternId != 0x09)
	//A modif
	if (gStim_t.tConfig.patternId == 0x11)
		gflag[0] = FALSE;

	bReprise = TRUE;

	return gStimErrNoError_c;
}

/************************************************************************************
 * Name :	StimulationSetLevel	*/
/**
 * @brief	Sets the stimulation level.
 * \param[in]	uint8_t pulseId : <em> Output number </em>
 * @param[in]	bool_t bIncrease
 * @return	The error of operation.
 ************************************************************************************/
StimErr_t StimulationSetLevel(uint8_t pulseId, bool_t bIncrease)
{
	uint16_t amplitude;

	gStim_t.pauseEn = TRUE;

	if ((StimOutId_t)pulseId > gStimOutMax_c)
		return gStimErrSetLevelOutIdMax_c;

	if ((gStim_t.tConfig.nStim == 1) && (gStim_t.tConfig.tPattern[0].outId == 1))
	{
		pulseId = 0;
		pulseId = 0;
	}

	if (bIncrease)
	{
		amplitude = gStim_t.tControl[pulseId].tEnvelope.curAmplitude + gStim_t.tControl[pulseId].tEnvelope.amplitudeIncrement;

		if (amplitude > gStim_t.tControl[pulseId].tEnvelope.maxAmplitude)
			amplitude = gStim_t.tControl[pulseId].tEnvelope.maxAmplitude;
	}
	else
	{
		if (gStim_t.tControl[pulseId].tEnvelope.curAmplitude < gStim_t.tControl[pulseId].tEnvelope.amplitudeIncrement)
			amplitude = 0;
		else
			amplitude = gStim_t.tControl[pulseId].tEnvelope.curAmplitude - gStim_t.tControl[pulseId].tEnvelope.amplitudeIncrement;
	}

	gStim_t.tControl[pulseId].tEnvelope.curAmplitude = amplitude;

	return gStimErrNoError_c;
}
#ifdef V016
/************************************************************************************
 * Name :	StimulationSetFrequency	*/
/**
 * @brief	Sets the stimulation frequency.
 * @param
 * @return	The error of operation.
 ************************************************************************************/
StimErr_t StimulationSetFrequency(uint8_t pulseId, uint16_t FreqA, uint16_t FreqB, uint16_t FreqC)
{
	gStim_t.pauseEn = TRUE;

	if ((StimOutId_t)pulseId > gStimOutMax_c)
		return gStimErrSetLevelOutIdMax_c;

	if ((gStim_t.tConfig.nStim == 1) && (gStim_t.tConfig.tPattern[0].outId == 1))
	{
		pulseId = 0;
		pulseId = 0;
	}

	if (((StimOutId_t)pulseId == 1) && (gStim_t.tConfig.FreqDiff != 0))
		return gStimErrConfigStimFrequencyDiff_c;

	gStim_t.tControl[pulseId].tEnvelope.patternFrequencyA = FreqA;
	gStim_t.tControl[pulseId].tEnvelope.patternFrequencyB = FreqB;
	gStim_t.tControl[pulseId].tEnvelope.patternFrequencyC = FreqC;

	if (gStim_t.tControl[pulseId].tEnvelope.wobulationEnabled == FALSE)
	{
		gStim_t.tConfig.frequency = DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[pulseId].tEnvelope.patternFrequencyA;
		gStim_t.tConfig.tPattern[pulseId].width = gStim_t.tControl[pulseId].tEnvelope.pulseWidthA;
		// A Modif
		StimManagementConfigPulse(&gStim_t.tConfig);
	}

	return gStimErrNoError_c;
}

/************************************************************************************
 * Name :	StimulationSetPulseWidth	*/
/**
 * @brief	Sets the stimulation pulsewidth.
 * @param
 * @return	The error of operation.
 ************************************************************************************/
StimErr_t StimulationSetPulseWidth(uint8_t pulseId, uint16_t PWA, uint16_t PWB, uint16_t PWC)
{
	gStim_t.pauseEn = TRUE;

	if ((StimOutId_t)pulseId > gStimOutMax_c)
		return gStimErrSetLevelOutIdMax_c;

	if ((gStim_t.tConfig.nStim == 1) && (gStim_t.tConfig.tPattern[0].outId == 1))
	{
		pulseId = 0;
		pulseId = 0;
	}

	if (PWA >= STIM_GEN_TRM_PERIOD_MAX || PWB >= STIM_GEN_TRM_PERIOD_MAX || PWC >= STIM_GEN_TRM_PERIOD_MAX)
		return gStimErrConfigStimPulseWidthMax_c;

	gStim_t.tControl[pulseId].tEnvelope.pulseWidthA = PWA;
	gStim_t.tControl[pulseId].tEnvelope.pulseWidthB = PWB;
	gStim_t.tControl[pulseId].tEnvelope.pulseWidthC = PWC;

	if (gStim_t.tControl[pulseId].tEnvelope.wobulationEnabled == FALSE)
	{
		gStim_t.tConfig.frequency = DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[pulseId].tEnvelope.patternFrequencyA;
		gStim_t.tConfig.tPattern[pulseId].width = gStim_t.tControl[pulseId].tEnvelope.pulseWidthA;
		// A Modif
		StimManagementConfigPulse(&gStim_t.tConfig);
	}

	return gStimErrNoError_c;
}
#endif
/************************************************************************************
 * Name :	StimulationResume	*/
/**
 * @brief	Resumes the stimulation control.
 * @param
 * @return	The error of operation.
 ************************************************************************************/
StimErr_t StimulationResume(void)
{
	gStim_t.pauseEn = FALSE;

	return gStimErrNoError_c;
}

/************************************************************************************
 * Name :	StimulationResume	*/
/**
 * @brief	Resumes the stimulation control.
 * @param
 * @return	The error of operation.
 ************************************************************************************/
StimErr_t StimulationPause(void)
{
	gStim_t.pauseEn = TRUE;

	return gStimErrNoError_c;
}

/************************************************************************************
 * Name :	StimulationExecute	*/
/**
 * @brief	Execute the stimulation control.
 * @param
 * @return	The end of operation enveloppe.
 ************************************************************************************/
bool_t StimulationExecute(FsmTaskActionReturn_t fsmTaskNotif_t[])
{
	uint8_t i, j, nStim;
	int16_t tempsintermediaire = 0;
	int32_t var = 0;
	int32_t temp[2] = {0, 0};
	bool_t bEnveloppeEnd = FALSE, StepEnd[2] = {0};
	uint32_t amplitude = 0 ,AmpMeas[2];
	int16_t halfperiod = 0, nCurTemp = 0;
	StimStep_c StepReturn[2];

	static struct
	{
		uint16_t nDelay;
		uint8_t nModulation;						 /**< Number of Modulation. */
		Modulation_t modulation_t[N_MODULATION_MAX]; /**< Modulation array. Contains the modulations list */
		int16_t nCur;

		bool_t wobulationEnabled;
		uint16_t patternFrequencyA;
		uint16_t pulseWidthA;
		uint16_t patternFrequencyB;
		uint16_t pulseWidthB;
		uint16_t patternFrequencyC;
		uint16_t pulseWidthC;
		uint16_t delay; /**< Envelope Delay. Delay before the envelope starts */
		uint8_t nRepetition;

	} stimControl_t[gStimOutMax_c];

	fsmTaskNotif_t[0].fsmState = 2;
	fsmTaskNotif_t[0].bDataReturn = TRUE;
	fsmTaskNotif_t[0].bReturn = FALSE;

	fsmTaskNotif_t[1].fsmState = 2;
	fsmTaskNotif_t[1].bDataReturn = TRUE;
	fsmTaskNotif_t[1].bReturn = FALSE;

	if (hasNotifElectToSend == FALSE)
	{
		fsmTaskNotif_t[2].fsmState = 2;
		fsmTaskNotif_t[2].bDataReturn = TRUE;
		fsmTaskNotif_t[2].bReturn = FALSE;
	}
	if (hasNotifDelayToSend == FALSE)
	{
		fsmTaskNotif_t[3].fsmState = 2;
		fsmTaskNotif_t[3].bDataReturn = TRUE;
		fsmTaskNotif_t[3].bReturn = FALSE;
	}

	nStim = gStim_t.tConfig.nStim;
	nbVoie = nStim;

	if (gStim_t.startEn)
	{
		for (i = 0; i < nStim; i++)
		{
			if (gStim_t.tConfig.patternId == 0x09)
			{
				stimControl_t[i].nModulation = gStim_t.tControl[i].tEnvelope.nModulation;
				stimControl_t[i].wobulationEnabled = gStim_t.tControl[i].tEnvelope.wobulationEnabled;
			}
			else
			{
				stimControl_t[i].nDelay = gStim_t.tControl[i].tEnvelope.delay * STIM_SUPERVIS_NTICK_IN_S;
				stimControl_t[i].nModulation = gStim_t.tControl[i].tEnvelope.nModulation;
				stimControl_t[i].wobulationEnabled = gStim_t.tControl[i].tEnvelope.wobulationEnabled;
				stimControl_t[i].patternFrequencyA = gStim_t.tControl[i].tEnvelope.patternFrequencyA;
				stimControl_t[i].pulseWidthA = gStim_t.tControl[i].tEnvelope.pulseWidthA;
				stimControl_t[i].patternFrequencyB = gStim_t.tControl[i].tEnvelope.patternFrequencyB;
				stimControl_t[i].pulseWidthB = gStim_t.tControl[i].tEnvelope.pulseWidthB;
				stimControl_t[i].patternFrequencyC = gStim_t.tControl[i].tEnvelope.patternFrequencyC;
				stimControl_t[i].pulseWidthC = gStim_t.tControl[i].tEnvelope.pulseWidthC;
#ifdef V016
				stimControl_t[i].nRepetition = gStim_t.tControl[i].tEnvelope.nRepetition;
#endif
			}

			for (j = 0; j < stimControl_t[i].nModulation; j++)
			{
				stimControl_t[i].modulation_t[j].id = gStim_t.tControl[i].tEnvelope.tModulation[j].id;
				if (gStim_t.tConfig.patternId == 0x09)
				{
					stimControl_t[i].modulation_t[j].period = gStim_t.tControl[i].tEnvelope.tModulation[j].period;
				}
				else
				{
					stimControl_t[i].modulation_t[j].period = gStim_t.tControl[i].tEnvelope.tModulation[j].period * STIM_SUPERVIS_NTICK_IN_S;
				}
			}
			stimControl_t[i].nCur = 0;
			amplitude = STIM_GEN_AMPLITUDE_MIN;
		}

		gStim_t.startEn = FALSE;
	}

	if (gStimTick)
	{

		gStimTick = FALSE;

		for (i = 0; i < nStim; i++)
		{
			/* Delay */
			if (stimControl_t[i].nDelay > 0)
			{
				stimControl_t[i].nDelay--;
				if (stimControl_t[i].nDelay == 1)
				{
					fsmTaskNotif_t[3].data[0] = i;
					fsmTaskNotif_t[3].data[1] = gModulDelay_c;
				}
			}
			else
			{
				j = gStim_t.tControl[i].tEnvelope.nModulation - stimControl_t[i].nModulation;
				if (gStim_t.pauseEn == FALSE)
					stimControl_t[i].nCur++;
				/* Amplitude Modulation */
				switch (gStim_t.tControl[i].tEnvelope.tModulation[j].id)
				{
				case gModulLinIncrease_c: /* Rising edge. */
					if (gStim_t.tConfig.patternId == 0x02)
					{
						if (gStim_t.tControl[i].tEnvelope.curAmplitude != 0)
						{
							if (bReprise == TRUE)
							{
								amplitude = ((uint32_t)gStim_t.tControl[i].tEnvelope.curAmplitude * stimControl_t[i].nCur) / stimControl_t[i].modulation_t[j].period;
							}
							else
							{
								stimControl_t[i].nCur = 0;
								stimControl_t[i].nModulation--;
							}
						}
						else
						{
							stimControl_t[i].nCur = 0;
							stimControl_t[i].nModulation--;
						}
					}
					else
					{
						amplitude = ((uint32_t)gStim_t.tControl[i].tEnvelope.curAmplitude * stimControl_t[i].nCur) / stimControl_t[i].modulation_t[j].period;
						StepReturn[i].Step = gStim_t.tControl[i].tEnvelope.tModulation[j].id;
					}

					break;

				case gModulMaxAmp_c: /* Maximum step. */
					bReprise = FALSE;
					if (stimControl_t[i].wobulationEnabled == TRUE)
					/* If wobulation is enabled */
					{
						halfperiod = (stimControl_t[i].modulation_t[j].period / 2);
						if (stimControl_t[i].nCur < (stimControl_t[i].modulation_t[j].period) / 2)
						{
							tempsintermediaire = ((stimControl_t[i].modulation_t[j].period / 2) - stimControl_t[i].nCur);
							if (tempsintermediaire > 0)
							{
								/* Frequency wobulation between FrequencyA and FrequencyB. */
								var = ((DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[i].tEnvelope.patternFrequencyB) - (DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[i].tEnvelope.patternFrequencyA));
								var = var / ((stimControl_t[i].modulation_t[j].period) / 2);
								gStim_t.tConfig.frequency = var * stimControl_t[i].nCur + (DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[i].tEnvelope.patternFrequencyA);

								/* Pulse whidth wobulation between pulseWidthA and pulseWidthB. */
								temp[i] = (int32_t)((int16_t)(gStim_t.tControl[i].tEnvelope.pulseWidthB - gStim_t.tControl[i].tEnvelope.pulseWidthA));
								temp[i] = (int32_t)(temp[i] * stimControl_t[i].nCur);
								temp[i] = temp[i] / halfperiod;
								gStim_t.tConfig.tPattern[i].width = temp[i] + gStim_t.tControl[i].tEnvelope.pulseWidthA;
							}
						}
						if (stimControl_t[i].nCur == (stimControl_t[i].modulation_t[j].period) / 2)
						{
							gStim_t.tConfig.frequency = (DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[i].tEnvelope.patternFrequencyB);
							gStim_t.tConfig.tPattern[i].width = gStim_t.tControl[i].tEnvelope.pulseWidthB;
						}
						if (stimControl_t[i].nCur > (stimControl_t[i].modulation_t[j].period) / 2)
						{
							tempsintermediaire = (stimControl_t[i].modulation_t[j].period - stimControl_t[i].nCur);
							if (tempsintermediaire > 0)
							{
								/* Frequency wobulation between FrequencyB and FrequencyC. */
								var = ((DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[i].tEnvelope.patternFrequencyC) - (DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[i].tEnvelope.patternFrequencyB));
								var = var / ((stimControl_t[i].modulation_t[j].period) / 2);
								gStim_t.tConfig.frequency = var * (stimControl_t[i].nCur - (stimControl_t[i].modulation_t[j].period) / 2) + (DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[i].tEnvelope.patternFrequencyB);

								temp[i] = (int32_t)((int16_t)(gStim_t.tControl[i].tEnvelope.pulseWidthC - gStim_t.tControl[i].tEnvelope.pulseWidthB));
								nCurTemp = (stimControl_t[i].nCur - halfperiod);
								temp[i] = (int32_t)(temp[i] * nCurTemp);
								temp[i] = temp[i] / halfperiod;
								gStim_t.tConfig.tPattern[i].width = temp[i] + gStim_t.tControl[i].tEnvelope.pulseWidthB;
							}
						}
						if (stimControl_t[i].nCur == (stimControl_t[i].modulation_t[j].period))
						{
							gStim_t.tConfig.frequency = (DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[i].tEnvelope.patternFrequencyC);
							gStim_t.tConfig.tPattern[i].width = gStim_t.tControl[i].tEnvelope.pulseWidthC;
						}
						//A modif
						StimManagementConfigPulse(&gStim_t.tConfig);
					}

					amplitude = gStim_t.tControl[i].tEnvelope.curAmplitude;
					StepReturn[i].Step = gStim_t.tControl[i].tEnvelope.tModulation[j].id;
					break;

				case gModulLinDecrease_c: /* Falling edge. */
					amplitude = (uint32_t)gStim_t.tControl[i].tEnvelope.curAmplitude - ((uint32_t)gStim_t.tControl[i].tEnvelope.curAmplitude * stimControl_t[i].nCur) / stimControl_t[i].modulation_t[j].period;
					StepReturn[i].Step = gStim_t.tControl[i].tEnvelope.tModulation[j].id;

					break;

				case gModulNullAmp_c: /* Rest period. */
					amplitude = STIM_GEN_AMPLITUDE_MIN;
					StepReturn[i].Step = gStim_t.tControl[i].tEnvelope.tModulation[j].id;
					break;

				default:
					break;
				}
				// A Modif
				StimManagementSetDigitalAmplitude(amplitude, i);

				if ((gStim_t.tConfig.patternId != 0x09) && (gStim_t.tConfig.patternId != 0x11))
				{
					/* Electrode detachment test */
					if (gStim_t.tControl[i].tEnvelope.tModulation[j].id == gModulMaxAmp_c) // Test available only when amplitud is max.
					{
						//A modif
						 if (gflag[i] == TRUE) // Is there a new value available ?
						 {
						 	AmpMeas[i] = ((uint32_t)gDigAmplMeas[i] * 1000);
						 	AmpMeas[i] = AmpMeas[i] / 1023;
						 	if ((amplitude > 100) && ((AmpMeas[i]) < ((amplitude * 70) / 100))) // Electrods are considered detached if measured amplitud is <70% of the command and commanded amplitud is above 10mA.
						 	{
						 		if (gStim_t.tControl[i].electrodeAdhesionDetect == 1) // Electrod adhesion detection is available ?
						 		{
						 			if (bElecTest[i] == TRUE)
						 			{
						 				nElectrodDetachmentCpt[i]++;
						 				gflag[i] = FALSE;
						 			}
						 		}
						 	}
						 	else
						 		nElectrodDetachmentCpt[i] = 0;
						 }
					}

					if (nElectrodDetachmentCpt[i] > 4) // Electrode detached ?
					{
						fsmTaskNotif_t[2].bReturn = TRUE;
						fsmTaskNotif_t[2].data[0] = gStimGenErrAdhElectrod_c; // Electrodes detachment error.
						if (nElectrodDetachmentCpt[(i + 1) % 2] > 5)
							fsmTaskNotif_t[2].data[1] = 2;
						else
							fsmTaskNotif_t[2].data[1] = gStim_t.tConfig.tPattern[i].outId; // Channel Id.
						hasNotifElectToSend = TRUE;

						bElecTest[i] = 0;
						bElecTest[(i + 1) % 2] = 0;

						nElectrodDetachmentCpt[i] = 0;
						nElectrodDetachmentCpt[(i + 1) % 2] = 0;
					}
				}

				if (stimControl_t[i].nCur >= stimControl_t[i].modulation_t[j].period)
				{
					stimControl_t[i].nCur = 0;
					stimControl_t[i].nModulation--;

					StepReturn[i].OutId = gStim_t.tConfig.tPattern[i].outId;
					StepEnd[i] = TRUE;

					if (stimControl_t[i].nModulation == 0)
					{
						bEnveloppeEnd = TRUE;
						if (bInstVesic == TRUE)
							stimControl_t[i].nModulation++;
						else
							stimControl_t[i].nModulation = gStim_t.tControl[i].tEnvelope.nModulation;
#ifdef V016
						if (stimControl_t[i].nRepetition != 0xFF)
							stimControl_t[i].nRepetition--;

						if (!stimControl_t[i].nRepetition && (stimControl_t[i].nRepetition != 0xFE))
						{
							gStim_t.pauseEn = TRUE;
							gStim_t.pauseEn = TRUE;

							StimulationStop();
						}
#endif
					}
				}
				if (StepEnd[i] && EFM32_STOP_IS_DIS) // Sends envelope steps via UART
				{
					StepEnd[i] = FALSE;

					fsmTaskNotif_t[i].data[0] = StepReturn[i].OutId;
					fsmTaskNotif_t[i].data[1] = StepReturn[i].Step;
					if (i == 0)
					{
						hasNotifToSendV1 = TRUE;
					}
					else
					{
						hasNotifToSendV2 = TRUE;
					}
				}
			}
		}
	}
	return bEnveloppeEnd;
}

/************************************************************************************
 * Name :	StimulationConfig	*/
/**
 * @brief	Configures the stimulation controle.
 * @param	pConfigData a pointer refers to the configuration datas
 * @return	The error of operation.
 ************************************************************************************/
StimErr_t StimulationConfig(StimConfigData_t *pConfigData)
{
	uint8_t cntModul = 0;
	uint8_t i = 0;
	uint8_t nStim;
	StimGenErr_t stimGenErr_t;

	/** - Asserts that at least a stim output is available */
	if (gStim_t.tConfig.nStim >= gStimOutMax_c)
		gStim_t.tConfig.nStim--;

	nStim = gStim_t.tConfig.nStim;

	/** - Gets the stim output Id */
	if (pConfigData->stimOutId >= gStimOutMax_c)
		return gStimErrConfigStimOutIdMax_c;

	gStim_t.tConfig.tPattern[nStim].outId = (StimOutId_t)pConfigData->stimOutId;

	/** - Asserts Pattern Types are the same */
	if (pConfigData->stimPatternId >= gStimPatternMax_c)
		return gStimErrConfigStimPatternIdMax_c;

	if (nStim == 0)
	{
		gStim_t.tConfig.patternId = (StimPatternId_t)pConfigData->stimPatternId;
	}
	else if (gStim_t.tConfig.patternId != (StimPatternId_t)pConfigData->stimPatternId)
		return gStimErrConfigStimPatternIdDiff_c;

	/** - Tests if electrode adhesion detection is active */
	if (pConfigData->bElectrodeAdhesionDetect)
		gStim_t.tControl[nStim].electrodeAdhesionDetect = STIM_ELECTRODE_ADH_DETECT_ON;
	else
		gStim_t.tControl[nStim].electrodeAdhesionDetect = STIM_ELECTRODE_ADH_DETECT_OFF;

#ifdef V016
	/** - Tests end of phase beahvior */
	if (pConfigData->envelopeRepeat)
		gStim_t.tControl[nStim].tEnvelope.nRepetition = pConfigData->envelopeRepeat;
	else
		gStim_t.tControl[nStim].tEnvelope.nRepetition = 0xFF;
#endif

	/** - Specific to neuroperipheric current */
	if (gStim_t.tConfig.patternId == 0x09)
	{
// A Modif
		// Init Stim Supervisor Timer
		STIM_SUPERVIS_FORCE_COUNT_STOP;
		// trbcr = 0x00;  // Timer RB : count stop, count forcible stop bit
		// trbocr = 0x00; // Timer RB one-shot control : not used
		// trbioc = 0x00; // Timer RB : Timer Mode
		// trbmr = (uint8_t)((gTmrRbCntSrcf8_c << 4) | gTmrRbModeTimer_c);
		// // Timer RB : Timer Mode, f1 count source
		// twrc_trbmr = DEF_ON; // write to reload register only.
		// trbpre = 25 - 1;	 // Timer RB Prescaler counter
		// trbpr = 25 - 1;		 // Timer RB counter

		// trcgra_addr = 16000; // Timer RC : Reset match value
		// trcgrc_addr = 16000; // Timer RC : Reset buffer register

		gStim_t.tControl[nStim].tEnvelope.delay = 0;

		if (pConfigData->patternFrequencyA)
		{
			gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulLinIncrease_c;
			gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->patternFrequencyA;
			cntModul++;
		}

		if (pConfigData->pulseWidthA)
		{
			gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulMaxAmp_c;
			gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->pulseWidthA;
			cntModul++;
		}

		if (pConfigData->patternFrequencyB)
		{
			gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulNullAmp_c;
			gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->patternFrequencyB;
			cntModul++;
		}

		gStim_t.tControl[nStim].tEnvelope.patternFrequencyA = 0;
		gStim_t.tControl[nStim].tEnvelope.patternFrequencyB = 0;
		gStim_t.tControl[nStim].tEnvelope.patternFrequencyC = 0;

		gStim_t.tControl[nStim].tEnvelope.pulseWidthA = 0;
		gStim_t.tControl[nStim].tEnvelope.pulseWidthB = 0;
		gStim_t.tControl[nStim].tEnvelope.pulseWidthC = 0;

		gStim_t.tControl[i].tEnvelope.nModulation = cntModul;

		gStim_t.tControl[i].tEnvelope.wobulationEnabled = FALSE;

		if (pConfigData->curAmplitude == 35)
		{
			gStim_t.tControl[nStim].tEnvelope.curAmplitude = 36;
		}
		else
			gStim_t.tControl[nStim].tEnvelope.curAmplitude = pConfigData->curAmplitude;

		gStim_t.tControl[nStim].tEnvelope.maxAmplitude = pConfigData->maxAmplitude;

		gStim_t.tControl[nStim].tEnvelope.amplitudeIncrement = pConfigData->amplitudeIncrement;

		if (pConfigData->delayPeriod == 1)
		{
			gStim_t.tConfig.FreqDiff = 1;
			gStim_t.tConfig.FreqDiff = 1;
		}
		if (pConfigData->delayPeriod == 2)
		{
			gStim_t.tConfig.FreqDiff = 2;
			gStim_t.tConfig.FreqDiff = 2;
		}
		gStim_t.tConfig.frequency = DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[nStim].tEnvelope.patternFrequencyC;
		gStim_t.tConfig.tPattern[nStim].width = 0;
		gStim_t.tConfig.Ratio = 0;
		gStim_t.tConfig.nStim++;
		stimGenErr_t = StimManagementConfigPulse(&gStim_t.tConfig);

		if (stimGenErr_t != gStimGenErrNoError_c)
		{
			gStim_t.tConfig.nStim--;
			return gStimErrConfigStimInvalidPulse_c;
		}
	}

	/** - Other currents */
	else
	{

		/** - Tests Frequencies	and modulation periods */
		/* Gets Frequencies are the same */

		if (nStim == 0)
		{
			if ((gStim_t.tControl[nStim].tEnvelope.patternFrequencyC == 150) && (gStim_t.tControl[nStim].tEnvelope.pulseWidthC == 50))
			{
				gStim_t.tControl[nStim].tEnvelope.patternFrequencyC = 149;
			}
			if ((gStim_t.tControl[nStim].tEnvelope.patternFrequencyB == 150) && (gStim_t.tControl[nStim].tEnvelope.pulseWidthB == 50))
			{
				gStim_t.tControl[nStim].tEnvelope.patternFrequencyB = 149;
			}
			if ((gStim_t.tControl[nStim].tEnvelope.patternFrequencyA == 150) && (gStim_t.tControl[nStim].tEnvelope.pulseWidthA == 50))
			{
				gStim_t.tControl[nStim].tEnvelope.patternFrequencyA = 149;
			}
			/* Tests if wobulation is enabled */

			if (pConfigData->patternFrequencyA != pConfigData->patternFrequencyB || pConfigData->patternFrequencyA != pConfigData->patternFrequencyC || pConfigData->patternFrequencyB != pConfigData->patternFrequencyC || pConfigData->pulseWidthA != pConfigData->pulseWidthB || pConfigData->pulseWidthA != pConfigData->pulseWidthC || pConfigData->pulseWidthB != pConfigData->pulseWidthC)
			{
				gStim_t.tControl[nStim].tEnvelope.wobulationEnabled = TRUE;
			}
			else
			{
				gStim_t.tControl[nStim].tEnvelope.wobulationEnabled = FALSE;
			}

			gStim_t.tControl[nStim].tEnvelope.patternFrequencyA = pConfigData->patternFrequencyA;
			gStim_t.tControl[nStim].tEnvelope.patternFrequencyB = pConfigData->patternFrequencyB;
			gStim_t.tControl[nStim].tEnvelope.patternFrequencyC = pConfigData->patternFrequencyC;
			if (pConfigData->pulseWidthA >= STIM_GEN_TRM_PERIOD_MAX || pConfigData->pulseWidthB >= STIM_GEN_TRM_PERIOD_MAX || pConfigData->pulseWidthC >= STIM_GEN_TRM_PERIOD_MAX)
				return gStimErrConfigStimPulseWidthMax_c;
			gStim_t.tControl[nStim].tEnvelope.pulseWidthA = pConfigData->pulseWidthA;
			gStim_t.tControl[nStim].tEnvelope.pulseWidthB = pConfigData->pulseWidthB;
			gStim_t.tControl[nStim].tEnvelope.pulseWidthC = pConfigData->pulseWidthC;

			cntModul = 0;

			gStim_t.tControl[nStim].tEnvelope.delay = pConfigData->delayPeriod;

			if (pConfigData->increasePeriod > 0)
			{
				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulLinIncrease_c;
				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->increasePeriod;
				cntModul++;
			}

			if (pConfigData->plateauPeriod > 0)
			{
				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulMaxAmp_c;
				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->plateauPeriod;
				cntModul++;
			}

			if (pConfigData->decreasePeriod > 0)
			{
				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulLinDecrease_c;
				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->decreasePeriod;
				cntModul++;
			}

			if (pConfigData->reposePeriod > 0)
			{
				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulNullAmp_c;
				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->reposePeriod;
				cntModul++;
			}

			if (gStim_t.tConfig.patternId == 0x02)
			{
				cntModul--;
				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulLinIncrease_c;
				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = 2;
				cntModul++;

				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulMaxAmp_c;
				gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->plateauPeriod;
				cntModul++;

				bInstVesic = TRUE;
			}
			else
				bInstVesic = FALSE;

			if (cntModul == 0)
				return gStimErrConfigStimNoModulation_c;
			else
				gStim_t.tControl[nStim].tEnvelope.nModulation = cntModul;
		}
		else
		{
			/** If wobulation enabled asserts frequencies are the same */
			if (gStim_t.tControl[0].tEnvelope.wobulationEnabled == TRUE)
			{
				gStim_t.tControl[nStim].tEnvelope.patternFrequencyA = pConfigData->patternFrequencyA;
				gStim_t.tControl[nStim].tEnvelope.patternFrequencyB = pConfigData->patternFrequencyB;
				gStim_t.tControl[nStim].tEnvelope.patternFrequencyC = pConfigData->patternFrequencyC;
				if (pConfigData->pulseWidthA >= STIM_GEN_TRM_PERIOD_MAX || pConfigData->pulseWidthB >= STIM_GEN_TRM_PERIOD_MAX || pConfigData->pulseWidthC >= STIM_GEN_TRM_PERIOD_MAX)
					return gStimErrConfigStimPulseWidthMax_c;
				gStim_t.tControl[nStim].tEnvelope.pulseWidthA = pConfigData->pulseWidthA;
				gStim_t.tControl[nStim].tEnvelope.pulseWidthB = pConfigData->pulseWidthB;
				gStim_t.tControl[nStim].tEnvelope.pulseWidthC = pConfigData->pulseWidthC;

				if ((gStim_t.tControl[nStim].tEnvelope.patternFrequencyC == 150) && (gStim_t.tControl[nStim].tEnvelope.pulseWidthC == 50))
				{
					gStim_t.tControl[nStim].tEnvelope.patternFrequencyC = 149;
				}
				if ((gStim_t.tControl[nStim].tEnvelope.patternFrequencyB == 150) && (gStim_t.tControl[nStim].tEnvelope.pulseWidthB == 50))
				{
					gStim_t.tControl[nStim].tEnvelope.patternFrequencyB = 149;
				}
				if ((gStim_t.tControl[nStim].tEnvelope.patternFrequencyA == 150) && (gStim_t.tControl[nStim].tEnvelope.pulseWidthA == 50))
				{
					gStim_t.tControl[nStim].tEnvelope.patternFrequencyA = 149;
				}
				if (gStim_t.tControl[0].tEnvelope.patternFrequencyA != gStim_t.tControl[nStim].tEnvelope.patternFrequencyA	  // pConfigData->patternFrequencyA
					|| gStim_t.tControl[0].tEnvelope.patternFrequencyB != gStim_t.tControl[nStim].tEnvelope.patternFrequencyB // pConfigData->patternFrequencyB
					|| gStim_t.tControl[0].tEnvelope.patternFrequencyC != gStim_t.tControl[nStim].tEnvelope.patternFrequencyC // pConfigData->patternFrequencyC
				)
				{
					// Determine highest frequency.
					if (gStim_t.tControl[0].tEnvelope.patternFrequencyB > pConfigData->patternFrequencyB)
						gStim_t.tConfig.FreqDiff = 1; // F0 > F1.
					else
						gStim_t.tConfig.FreqDiff = -1; // F1 > F0.

					// Determine the ratio.
					if (gStim_t.tConfig.FreqDiff == 1)
					{
						if ((gStim_t.tControl[0].tEnvelope.patternFrequencyB % pConfigData->patternFrequencyB) == 0)
							gStim_t.tConfig.Ratio = (gStim_t.tControl[0].tEnvelope.patternFrequencyB / pConfigData->patternFrequencyB);
						else
							return gStimErrConfigStimFrequencyDiff_c;
					}
					else
					{
						if ((gStim_t.tControl[nStim].tEnvelope.patternFrequencyB % pConfigData->patternFrequencyB) == 0)
							gStim_t.tConfig.Ratio = (pConfigData->patternFrequencyB / gStim_t.tControl[0].tEnvelope.patternFrequencyB);
						else
							return gStimErrConfigStimFrequencyDiff_c;
					}
				}
				else
				{
					gStim_t.tControl[nStim].tEnvelope.wobulationEnabled = TRUE;
					gStim_t.tConfig.FreqDiff = 0; // Frequencies are the same.
				}

				gStim_t.tControl[nStim].tEnvelope.patternFrequencyA = gStim_t.tControl[0].tEnvelope.patternFrequencyA;
				gStim_t.tControl[nStim].tEnvelope.patternFrequencyB = gStim_t.tControl[0].tEnvelope.patternFrequencyB;
				gStim_t.tControl[nStim].tEnvelope.patternFrequencyC = gStim_t.tControl[0].tEnvelope.patternFrequencyC;

				gStim_t.tControl[nStim].tEnvelope.pulseWidthA = pConfigData->pulseWidthA;
				gStim_t.tControl[nStim].tEnvelope.pulseWidthB = pConfigData->pulseWidthB;
				gStim_t.tControl[nStim].tEnvelope.pulseWidthC = pConfigData->pulseWidthC;
				gStim_t.tControl[nStim].tEnvelope.delay = pConfigData->delayPeriod;

				cntModul = 0;

				if (pConfigData->increasePeriod)
				{
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulLinIncrease_c;
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->increasePeriod;
					cntModul++;
				}

				if (pConfigData->plateauPeriod)
				{
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulMaxAmp_c;
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->plateauPeriod;
					cntModul++;
				}

				if (pConfigData->decreasePeriod)
				{
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulLinDecrease_c;
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->decreasePeriod;
					cntModul++;
				}

				if (pConfigData->reposePeriod)
				{
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulNullAmp_c;
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->reposePeriod;
					cntModul++;
				}
				gStim_t.tControl[nStim].tEnvelope.nModulation = cntModul;
			}
			else
			{ // Wobulation is disabled.
				if (pConfigData->patternFrequencyA != pConfigData->patternFrequencyB || pConfigData->patternFrequencyA != pConfigData->patternFrequencyC || pConfigData->patternFrequencyB != pConfigData->patternFrequencyC || pConfigData->pulseWidthA != pConfigData->pulseWidthB || pConfigData->pulseWidthA != pConfigData->pulseWidthC || pConfigData->pulseWidthB != pConfigData->pulseWidthC)
				{
					return gStimErrConfigStimNoWobulation_c;
				}

				gStim_t.tControl[nStim].tEnvelope.patternFrequencyA = pConfigData->patternFrequencyA;
				gStim_t.tControl[nStim].tEnvelope.patternFrequencyB = pConfigData->patternFrequencyB;
				gStim_t.tControl[nStim].tEnvelope.patternFrequencyC = pConfigData->patternFrequencyC;
				if (pConfigData->pulseWidthA >= STIM_GEN_TRM_PERIOD_MAX || pConfigData->pulseWidthB >= STIM_GEN_TRM_PERIOD_MAX || pConfigData->pulseWidthC >= STIM_GEN_TRM_PERIOD_MAX)
					return gStimErrConfigStimPulseWidthMax_c;
				gStim_t.tControl[nStim].tEnvelope.pulseWidthA = pConfigData->pulseWidthA;
				gStim_t.tControl[nStim].tEnvelope.pulseWidthB = pConfigData->pulseWidthB;
				gStim_t.tControl[nStim].tEnvelope.pulseWidthC = pConfigData->pulseWidthC;

				if ((gStim_t.tControl[nStim].tEnvelope.patternFrequencyC == 150) && (gStim_t.tControl[nStim].tEnvelope.pulseWidthC == 50))
				{
					gStim_t.tControl[nStim].tEnvelope.patternFrequencyC = 149;
				}
				if ((gStim_t.tControl[nStim].tEnvelope.patternFrequencyB == 150) && (gStim_t.tControl[nStim].tEnvelope.pulseWidthB == 50))
				{
					gStim_t.tControl[nStim].tEnvelope.patternFrequencyB = 149;
				}
				if ((gStim_t.tControl[nStim].tEnvelope.patternFrequencyA == 150) && (gStim_t.tControl[nStim].tEnvelope.pulseWidthA == 50))
				{
					gStim_t.tControl[nStim].tEnvelope.patternFrequencyA = 149;
				}
				// Frequencies are different ?
				if (gStim_t.tControl[0].tEnvelope.patternFrequencyA != gStim_t.tControl[nStim].tEnvelope.patternFrequencyA || gStim_t.tControl[0].tEnvelope.patternFrequencyB != gStim_t.tControl[nStim].tEnvelope.patternFrequencyB || gStim_t.tControl[0].tEnvelope.patternFrequencyC != gStim_t.tControl[nStim].tEnvelope.patternFrequencyC)
				{
					// Determine highest frequency.
					if (gStim_t.tControl[0].tEnvelope.patternFrequencyB > gStim_t.tControl[nStim].tEnvelope.patternFrequencyB)
						gStim_t.tConfig.FreqDiff = 1; // F0 > F1.
					else
						gStim_t.tConfig.FreqDiff = -1; // F1 > F0.

					// Determine the ratio.
					if (gStim_t.tConfig.FreqDiff == 1)
					{
						if ((gStim_t.tControl[0].tEnvelope.patternFrequencyB % gStim_t.tControl[nStim].tEnvelope.patternFrequencyB) == 0)
							gStim_t.tConfig.Ratio = (gStim_t.tControl[0].tEnvelope.patternFrequencyB / gStim_t.tControl[nStim].tEnvelope.patternFrequencyB);
						else
							return gStimErrConfigStimFrequencyDiff_c;
					}
					else
					{
						if ((gStim_t.tControl[nStim].tEnvelope.patternFrequencyB % gStim_t.tControl[0].tEnvelope.patternFrequencyB) == 0)
							gStim_t.tConfig.Ratio = (gStim_t.tControl[nStim].tEnvelope.patternFrequencyB / gStim_t.tControl[0].tEnvelope.patternFrequencyB);
						else
							return gStimErrConfigStimFrequencyDiff_c;
					}
				}
				else
					gStim_t.tConfig.FreqDiff = 0; // Frequencies are the same.
				cntModul = 0;

				gStim_t.tControl[nStim].tEnvelope.delay = pConfigData->delayPeriod;

				if (pConfigData->increasePeriod > 0)
				{
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulLinIncrease_c;
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->increasePeriod;
					cntModul++;
				}

				if (pConfigData->plateauPeriod > 0)
				{
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulMaxAmp_c;
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->plateauPeriod;
					cntModul++;
				}

				if (pConfigData->decreasePeriod > 0)
				{
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulLinDecrease_c;
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->decreasePeriod;
					cntModul++;
				}

				if (pConfigData->reposePeriod > 0)
				{
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].id = gModulNullAmp_c;
					gStim_t.tControl[nStim].tEnvelope.tModulation[cntModul].period = pConfigData->reposePeriod;
					cntModul++;
				}

				if (cntModul == 0)
					return gStimErrConfigStimNoModulation_c;
				else
					gStim_t.tControl[nStim].tEnvelope.nModulation = cntModul;
			}
		}

		gStim_t.tControl[nStim].tEnvelope.curAmplitude = pConfigData->curAmplitude;

		gStim_t.tControl[nStim].tEnvelope.maxAmplitude = pConfigData->maxAmplitude;

		gStim_t.tControl[nStim].tEnvelope.amplitudeIncrement = pConfigData->amplitudeIncrement;

		/* Sets Pulse Generation */

		if (((gStim_t.tControl[0].tEnvelope.patternFrequencyC == 150) && (gStim_t.tControl[0].tEnvelope.pulseWidthC == 50)) ||
		    ((gStim_t.tControl[1].tEnvelope.patternFrequencyC == 150) && (gStim_t.tControl[1].tEnvelope.pulseWidthC == 50)))
		{
			gStim_t.tControl[0].tEnvelope.patternFrequencyC = 149;
		}
		gStim_t.tConfig.frequency = DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[0].tEnvelope.patternFrequencyC;
		gStim_t.tConfig.tPattern[nStim].width = gStim_t.tControl[nStim].tEnvelope.pulseWidthC;
		gStim_t.tConfig.nStim++;
		// A Modif
		stimGenErr_t = StimManagementConfigPulse(&gStim_t.tConfig);

		if (stimGenErr_t != gStimGenErrNoError_c)
		{
			gStim_t.tConfig.nStim--;
			return gStimErrConfigStimInvalidPulse_c;
		}

		if (((gStim_t.tControl[0].tEnvelope.patternFrequencyB == 150) && (gStim_t.tControl[0].tEnvelope.pulseWidthB == 50)) ||
			((gStim_t.tControl[1].tEnvelope.patternFrequencyB == 150) && (gStim_t.tControl[1].tEnvelope.pulseWidthB == 50)))
		{
			gStim_t.tControl[0].tEnvelope.patternFrequencyB = 149;
		}
		gStim_t.tConfig.frequency = DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[0].tEnvelope.patternFrequencyB;
		gStim_t.tConfig.tPattern[nStim].width = gStim_t.tControl[nStim].tEnvelope.pulseWidthB;
		// A Modif
	 stimGenErr_t = StimManagementConfigPulse(&gStim_t.tConfig);

		if (stimGenErr_t != gStimGenErrNoError_c)
		{
			gStim_t.tConfig.nStim--;
			return gStimErrConfigStimInvalidPulse_c;
		}

		if (((gStim_t.tControl[0].tEnvelope.patternFrequencyA == 150) && (gStim_t.tControl[0].tEnvelope.pulseWidthA == 50)) ||
		    ((gStim_t.tControl[1].tEnvelope.patternFrequencyB == 150) && (gStim_t.tControl[1].tEnvelope.pulseWidthB == 50)))
		{
			gStim_t.tControl[0].tEnvelope.patternFrequencyA = 149;
		}
		gStim_t.tConfig.frequency = DEF_TIME_NBR_100nS_PER_SEC / gStim_t.tControl[0].tEnvelope.patternFrequencyA;
		gStim_t.tConfig.tPattern[nStim].width = gStim_t.tControl[nStim].tEnvelope.pulseWidthA;
		//A modif
		stimGenErr_t = StimManagementConfigPulse(&gStim_t.tConfig);

		if (stimGenErr_t != gStimGenErrNoError_c)
		{
			gStim_t.tConfig.nStim--;
			return gStimErrConfigStimInvalidPulse_c;
		}
	}
	if (gStim_t.tConfig.patternId == 0x02 || gStim_t.tConfig.patternId == 0x09)
	{
		gStim_t.tControl[0].tEnvelope.wobulationEnabled = FALSE;
		gStim_t.tControl[1].tEnvelope.wobulationEnabled = FALSE;
	}
	return gStimErrNoError_c;
}
/**********************************************************************************
End of function
***********************************************************************************/

/**********************************************************************************
Function Name:
Description:
Parameters: 	none
Return value: 	none
***********************************************************************************/
//#pragma INTERRUPT StimSupervisISR
//void StimSupervisISR(void)
//{
//	gStimTick = TRUE;
//}

void TIMER1_IRQHandler(void)
{
// MeSS_GestionCourantBiphasiquePositif();
 //MeSS_GestionCourantMonophasiquePositif();
  //MeSS_GestionCourantBiphasiqueNegatif();
 // MeSS_GestionCourantMonophasiqueNegatif();
// MeSS_GestionCourantBiphasiqueAlterne();
//  VeineuxBiphas();
  gStimTick = TRUE;
  TIMER_IntClear(TIMER_ENV, TIMER_IF_OF);
}
/**********************************************************************************
End of function
***********************************************************************************/
