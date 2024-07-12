/*
 * siue.c
 *
 *  Created on: 13 déc. 2023
 *      Author: ebt
 */
#include "siue.h"
#include "stddef.h"
#include "rskr8c27def.h"
#include "timer.h"
#include "iadc.h"
#include "stimulation.h"
#include "pin_config.h"
#include "em_gpio.h"
#include "sl_udelay.h"
#include "stim_management.h"
#include "sl_spidrv_instances.h"
#include "math.h"

// #define STIM_OUT_SEL_NONE GPIO->P_CLR[CS_VOIE1_PORT].DOUT = ((1 << CS_VOIE1_PIN) | (1 << CS_VOIE2_PIN)) //GPIO_PinOutClear(CS_VOIE1_PORT, CS_VOIE1_PIN);
#define N_MODULATION_MAX 4 /**< Maximum number of modulation by envelope */

/*! \def iSRP_MOMENT_APPLICATION_AOP
\brief Moment en �s d�finit pour positionner l'AOP avant le d�but de 1ere polarit� du SRP */
#define iSRP_MOMENT_APPLICATION_AOP 700U // en us

/*! Correction d'erreur du timer MFT1 */
#define uiERREUR_TIMER_MFT1 10U

/*! Valeur minimale de configuration du timer MFT1 exprim� � �s*/
#define uiMIN_TIMER_MFT1 40U

#define gNbExponentialValue_c 26 /**< Size of the values array defining a exponential impulse */
#define gNbSinusValue_c 51
#define gNbCompensValue 9
#define gNbExponentialPulseRepetition_c 8 /**< Number Size of the values array defining a exponential impulse */

#define STIM_GEN_TR_MIN 50 // in �s

#define STIM_GEN_COUNT_MAX 65535 // TIMER_MaxCount(TIMER0)
#define NORMAL_MODE 0x0000

sl_sleeptimer_timer_handle_t timer;

uint16_t TablePuissanceCalculee[gStimOutMax_c][gNbExponentialValue_c] = {0};
uint16_t TablePuissanceCalculeeCompens[gStimOutMax_c][gNbExponentialValue_c] = {0};
uint16_t TablePuissanceCalculeeHemiSinusMonophasic[gStimOutMax_c][gNbSinusValue_c];
uint16_t TablePuissanceCalculeeHemiSinusDiphasic[gStimOutMax_c][gNbSinusValue_c];
uint16_t TablePuissanceCalculeeSinus[gStimOutMax_c][gNbSinusValue_c];
/* Exponential current coefficient table */
static const uint8_t gExponentialValue_percent[gNbExponentialValue_c]
    //              = {25, 44, 58, 69, 77, 82, 87, 90, 92, 94, 96, 97, 97, 98, 98, 99, 100};        /* 17 step values */
    = {17, 31, 43, 53, 61, 68, 73, 78, 82, 85, 87, 90, 91, 93, 94, 95, 96, 96, 97, 97, 98, 98, 98, 99, 99, 100 /*, 17, 31, 43, 53, 61, 68, 73, 78, 82, 85, 87, 90, 91, 93, 94, 95, 96, 96, 97, 97, 98, 98, 98, 99, 99, 100*/}; /* 26 step values */

static const uint8_t gBiphasCompens_percent[gNbCompensValue] = {100, 80, 65, 53, 43, 35, 28, 22, 15}; //{0.0, 6.27905195293134, 12.53332335643043, 18.738131458572454, 24.86898871648549, 30.90169943749474, 36.812455268467794, 42.57792915650727, 48.17536741017153, 53.58267949789967, 58.778525229247315, 63.74239897486896, 68.45471059286886, 72.89686274214115, 77.05132427757893, 80.90169943749474, 84.43279255020151, 87.63066800438637, 90.48270524660195, 92.97764858882513, 95.10565162951535, 96.85831611286311, 98.22872507286885, 99.21147013144778, 99.80267284282715, 100.0, 99.80267284282715, 99.21147013144778, 98.22872507286885, 96.85831611286312, 95.10565162951536, 92.97764858882515, 90.48270524660195, 87.63066800438635, 84.4327925502015, 80.90169943749474, 77.05132427757893, 72.89686274214114, 68.45471059286888, 63.74239897486898, 58.77852522924732, 53.58267949789967, 48.17536741017157, 42.57792915650729, 36.812455268467815, 30.90169943749474, 24.86898871648549, 18.738131458572454, 12.533323356430444, 6.279051952931354,0.0};
static const int8_t gSinusValue_percent[gNbSinusValue_c] = {0.0, 12.53332335643043, 24.86898871648549, 36.812455268467794, 48.17536741017153, 58.778525229247315, 68.45471059286888, 77.05132427757893, 84.43279255020151, 90.48270524660195, 95.10565162951535, 98.22872507286887, 99.80267284282715, 99.80267284282715, 98.22872507286885, 95.10565162951535, 90.48270524660195, 84.4327925502015, 77.05132427757893, 68.45471059286885, 58.77852522924732, 48.17536741017152, 36.81245526846777, 24.86898871648549, 12.533323356430401, -2.842170943040401e-14, -12.53332335643043, -24.868988716485504, -36.81245526846783, -48.17536741017153, -58.77852522924735, -68.45471059286888, -77.05132427757894, -84.43279255020153, -90.48270524660197, -95.10565162951536, -98.22872507286888, -99.80267284282715, -99.80267284282715, -98.22872507286885, -95.10565162951536, -90.48270524660197, -84.4327925502015, -77.05132427757891, -68.45471059286882, -58.778525229247236, -48.17536741017153, -36.8124552684678, -24.868988716485447, -12.533323356430387, 0};
uint8_t gNPulse_c[gStimPatternMax_c] = {0};
uint16_t gDigAmplMeas[gStimOutMax_c];

uint8_t cpt = 7;
volatile uint8_t cptFreqDiff = 0;
volatile StimulationGeneration_t gStimGen_t;

bool_t gflag[gStimOutMax_c] = {0};
static const uint8_t gStimOutCmd_c[gStimOutMax_c] = {STIM_OUT_0, STIM_OUT_1};

void (*pStimGenCallback[gStimPatternMax_c])(void);
static void StimGenDummyFunct(void);

void Gpio_SetAop()
{
  GPIO_PinOutSet(CMD_AOP_PORT, CMD_AOP_PIN);
}

void Gpio_ClrAop()
{
  GPIO_PinOutClear(CMD_AOP_PORT, CMD_AOP_PIN);
}

/************************************************************************************
 * Name :	StimManagementHacheurInit 	*/
/**
 * @brief	Description
 * @param	.
 * @return	.
 ************************************************************************************/
void StimManagementHacheurInit(void)
{
  uint8_t i;

  // DISABLE_IRQ;

  for (i = 0; i < gStimPatternMax_c; i++)
    pStimGenCallback[i] = StimGenDummyFunct;

  /** - Callback functions declaration. */
  pStimGenCallback[gStimPatternBiphasic_c] = ImpulsBiphas;
  pStimGenCallback[gStimPatternBiphasicSynchro_c] = ImpulsBiphasSynchro;
  pStimGenCallback[gStimPatternMonophasic_c] = ImpulsMonophas;
  pStimGenCallback[gStimPatternGalvanic_c] = Galvanic;
  pStimGenCallback[gStimPatternBiphasicAltern_c] = ImpulsBiphasAltern;
  pStimGenCallback[gStimPatternVeineuxBiphasic_c] = VeineuxBiphas;
  // pStimGenCallback[gStimPatternNeuro_c] = NeuroMonophas;
  pStimGenCallback[gStimPatternBiphasicCompense_c] = ImpulsBiphasCompens;
  pStimGenCallback[gStimPatternBiphasicNegative_c] = ImpulsBiphasNeg;
  pStimGenCallback[gStimPatternSinus_c] = Sinus2;
  pStimGenCallback[gStimPatternSemiSinusMonophasic_c] = SinusMono;
  pStimGenCallback[gStimPatternSemiSinusDiphas_c] = SinusDiphase;

  /** - Number of alternance per signal. */
  gNPulse_c[gStimPatternBiphasic_c] = 2;
  gNPulse_c[gStimPatternMonophasic_c] = 1;
  gNPulse_c[gStimPatternBiphasicAltern_c] = 2;
  gNPulse_c[gStimPatternVeineuxBiphasic_c] = 52;
  gNPulse_c[gStimPatternBiphasicNegative_c] = 2;
  gNPulse_c[gStimPatternBiphasicSynchro_c] = 2;
  gNPulse_c[gStimPatternSemiSinusMonophasic_c] = gNbSinusValue_c;
  gNPulse_c[gStimPatternSemiSinusDiphas_c] = gNbSinusValue_c;
  gNPulse_c[gStimPatternSinus_c] = gNbSinusValue_c;

  TIMER_TopSet(TIMER_ENV, 2250 - 1);
  SPI_TRANSMIT_DATA(0);
  SPI_TRANSMIT_DATA(0);

  Gpio_ClrAop();
  CMD_M_DISCONNECT;
  STIM_OUT_SEL_NONE;

  // DETECT_RES_CS_EN;
  // ENABLE_IRQ;
}
/************************************************************************************
 * Name :  StimManagementConfigPulse   */
/**
 * @brief  Sets the pulses frequency and pulses widths
 * @param  .
 * @return .
 ************************************************************************************/
StimGenErr_t StimManagementConfigPulse(StimulationConfiguration_t *pStimConfig_t)
{
  uint8_t i;
  static uint32_t temp;
  uint32_t nStimTimerloop;
  uint32_t nPulseTimerloop;
  uint32_t trStim;
  uint32_t trPulse;
  uint32_t frequency;

  /** Asserts stimulation period is greater than total pulses period */
  if (pStimConfig_t->patternId == gStimPatternGalvanic_c)
  {
    return gStimGenErrNoError_c;
  }

  temp = 0;
  for (i = 0; i < pStimConfig_t->nStim; i++)
  {
    temp += pStimConfig_t->tPattern[i].width * gNPulse_c[pStimConfig_t->patternId];
  }

  /** T = N * STIM_GEN_TRM_PERIOD_MAX + Tr */
  nStimTimerloop = (pStimConfig_t->frequency) / STIM_GEN_TRM_PERIOD_MAX;
  trStim = (pStimConfig_t->frequency) % STIM_GEN_TRM_PERIOD_MAX;

  nPulseTimerloop = (10 * temp) / STIM_GEN_TRM_PERIOD_MAX;
  trPulse = (10 * temp) % STIM_GEN_TRM_PERIOD_MAX;

  if (nPulseTimerloop > nStimTimerloop)
    return gStimGenErrPulseWidth_c;

  if (nPulseTimerloop == nStimTimerloop)
    if (trPulse >= trStim)
      return gStimGenErrPulseWidth_c;

  /** Sets stimulation and pulse periods */

  /* T = TStim - TPulse = (NStimTimerloop - NPulseTimerloop) * BSP_MAX_STIM_TIMER_PERIOD + (TrStim - TrPulse) */
  nStimTimerloop -= nPulseTimerloop;

  if (trStim >= trPulse)
    trStim -= trPulse;
  else
  {
    nStimTimerloop--;
    trStim = STIM_GEN_TRM_PERIOD_MAX + trStim - trPulse;
  }

  /* if TrStim < Critical Tinterrupt, TrStim = Critical Tinterrupt */
  if (trStim < STIM_GEN_TR_MIN)
    trStim = STIM_GEN_TR_MIN;

  /** counters value */

  STIM_MNGMNT_GET_RESOURCE;

  gStimGen_t.nTimerLoop = nStimTimerloop;
  gStimGen_t.cntTr = (trStim * STIM_GEN_TRM_CLK_SOURCE_10MHz);
  gStimGen_t.nPulse = pStimConfig_t->nStim;

  for (i = 0; i < gStimGen_t.nPulse; i++)
  {
    gStimGen_t.tPulse[i].outId = pStimConfig_t->tPattern[i].outId;
    gStimGen_t.tPulse[i].cntWidth = (pStimConfig_t->tPattern[i].width * STIM_GEN_TRM_CLK_SOURCE_MHz);
  }

  gStimGen_t.stimGenPatternId = pStimConfig_t->patternId;

  gStimGen_t.FreqDiff = pStimConfig_t->FreqDiff;
  gStimGen_t.Ratio = pStimConfig_t->Ratio;

  STIM_MNGMNT_RELEASE_RESOURCE;

  if (gStimGen_t.stimGenPatternId == 0x2)
  {
    if (pStimConfig_t->tPattern[0].width == 0)
    {
      gStimGen_t.FreqDiff = 0;
      gStimGen_t.FreqDiff = 0;
    }
    else
    {
      gStimGen_t.FreqDiff = 1;
      gStimGen_t.FreqDiff = 1;
    }
  }
  if ((gStimGen_t.stimGenPatternId == gStimPatternSemiSinusMonophasic_c) ||
      (gStimGen_t.stimGenPatternId == gStimPatternSemiSinusDiphas_c) ||
      (gStimGen_t.stimGenPatternId == gStimPatternSinus_c))
  {
    frequency = DEF_TIME_NBR_100nS_PER_SEC / pStimConfig_t->frequency;
    gStimGen_t.cntTr = 1000000 / (frequency * gNbSinusValue_c);
  }
  return gStimGenErrNoError_c;
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :  StimManagementSetDigitalAmplitude   */
/**
 * @brief  Sets the Digital Amplitude
 * @param  amplitude
 * @param  pulseId
 * @return .
 ************************************************************************************/
StimGenErr_t StimManagementSetDigitalAmplitude(uint16_t amplitude, uint8_t pulseId)
{
  uint32_t digAmpl;
  uint8_t i = 0;

  if ((StimOutId_t)pulseId > gStimOutMax_c)
    return gStimGenErrSetDigAmplOutIdMax_c;

  if (amplitude > STIM_GEN_AMPLITUDE_MAX)
    return gStimGenErrSetDigAmplitudeMax_c;

  digAmpl = ((amplitude * 100) * 4096) / 109400;

  gStimGen_t.tPulse[pulseId].digitalAmplitude = digAmpl;

  if (gStimGen_t.stimGenPatternId == gStimPatternVeineuxBiphasic_c)
  {
    for (i = 0; i < gNbExponentialValue_c; i++)
    {
      TablePuissanceCalculee[pulseId][i] = (uint16_t)((gExponentialValue_percent[i] * digAmpl) / 100);
    }
  }

  if ((gStimGen_t.stimGenPatternId == gStimPatternBiphasicCompense_c))
  {
    for (i = 0; i < gNbCompensValue; i++)
    {
      TablePuissanceCalculeeCompens[pulseId][i] = (uint16_t)((gBiphasCompens_percent[i] * digAmpl) / 100);
    }
  }

  if (gStimGen_t.stimGenPatternId == gStimPatternSemiSinusMonophasic_c)
  {
    for (i = 0; i < gNbSinusValue_c; i++)
    {
      if (gSinusValue_percent[i] < 0)
      {
        TablePuissanceCalculeeHemiSinusMonophasic[pulseId][i] = 0;
      }
      else
      {
        TablePuissanceCalculeeHemiSinusMonophasic[pulseId][i] = (uint16_t)((gSinusValue_percent[i] * digAmpl) / 100);
      }
    }
  }

  if (gStimGen_t.stimGenPatternId == gStimPatternSinus_c)
  {
    for (i = 0; i < gNbSinusValue_c; i++)
    {
      if (gSinusValue_percent[i] < 0)
      {
        TablePuissanceCalculeeSinus[pulseId][i] = (uint16_t)((-gSinusValue_percent[i] * digAmpl) / 100);
      }
      else
      {
        TablePuissanceCalculeeSinus[pulseId][i] = (uint16_t)((gSinusValue_percent[i] * digAmpl) / 100);
      }
    }
  }

  if (gStimGen_t.stimGenPatternId == gStimPatternSemiSinusDiphas_c)
  {
    for (i = 0; i < gNbSinusValue_c; i++)
    {
      if (gSinusValue_percent[i] < 0)
      {
        TablePuissanceCalculeeHemiSinusDiphasic[pulseId][i] = (uint16_t)((-gSinusValue_percent[i] * digAmpl) / 100);
      }
      else
      {
        TablePuissanceCalculeeHemiSinusDiphasic[pulseId][i] = (uint16_t)((gSinusValue_percent[i] * digAmpl) / 100);
      }
    }
  }

  return gStimGenErrNoError_c;
}

/*!
 * \fn bool Adc_TraitementAcquisitionCourantRelecture(void)
 * \brief Traitement de l'acquisitions courant de relecture si fin de convertion
 * \retval false si la derni�re demande d'acquisition n'est pas trait�e sinon false
 */
bool Adc_TraitementAcquisitionCourantRelecture(void)
{
  bool bRet = false;

  if (IADC_Read_Current() != 0)
  {
    bRet = true;
  }
  return (bRet);
}

/*!
 * \fn void Gpio_SetElectrostimulation (eCdeElectrostimulation_Type eCdeElectrostimulation)
 * \brief Positionne une �tape de l��lectrostimulation
 * \param eCdeElectrostimulation = eEOFF ou eEPH ou eEPB ou ETAPE...
 */
void Gpio_SetElectrostimulation(eCdeElectrostimulation_Type eCdeElectrostimulation)
{
  switch (eCdeElectrostimulation)
  {
  case eEOFF: // Electrostimulation Off
    // GPIO_ResetBits(CDE_MOS_L1_PIN|CDE_MOS_L2_PIN|CDE_MOS_H1_PIN|CDE_MOS_H2_PIN);
    // CLR_BRIDGE;
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN) | (1 << CMD_L2_PIN) | (1 << CMD_H1_PIN) | (1 << CMD_H2_PIN);
    break;
  case eEPH: // Electrostimulation Polarit� Haute
    //   GPIO_ResetBits(CMD_L1_PIN|CMD_L2_PIN|CMD_H1_PIN|CMD_H2_PIN);
    //    GPIO_SetBits(CMD_L1_PIN|CMD_H2_PIN);
    // CLR_BRIDGE;
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN) | (1 << CMD_L1_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H2_PIN);
    break;
  case eEPB: // Electrostimulation Polarit� Basse
             //  GPIO_ResetBits(CMD_L1_PIN|CMD_L2_PIN|CMD_H1_PIN|CMD_H2_PIN);
             //  GPIO_SetBits(CMD_L2_PIN|CMD_H1_PIN);
             //  CLR_BRIDGE;
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H1_PIN);
    break;
  case eETAPE1: // Etape #1 issue du document de conception hardware RESET
    // GPIO_ResetBits(CDE_MOS_H1_PIN|CDE_MOS_H2_PIN|CDE_MOS_L1_PIN|CDE_MOS_L2_PIN);
    CLR_BRIDGE;

    break;
  case eETAPE2: // Etape #2 issue du document de conception hardware
    // Commande L1 et L2
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
    break;
  case eETAPE3: // Etape #3 issue du document de conception hardware Etat HAUT

    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H2_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN);

    break;
  case eETAPE4: // Etape #4 issue du document de conception hardware "Reset  H1-L2

    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_H2_PIN);
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN);
    break;
  case eETAPE6: // Etape #6 issue du document de conception hardware
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H1_PIN);
    break;
  case eETAPE7: // Etape #7 issue du document de conception hardware
    CLR_BRIDGE;
    break;
  case eETAPE8: // Etape #8 créer pour carte stimbio V2
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H1_PIN);
    break;
  case eETAPE9:
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_H1_PIN);
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
    break;
  case eETAPE10:
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H2_PIN) | (1 << CMD_L1_PIN);
    // GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H2_PIN);
    break;
  case C2:
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
    break;
  case C6:
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN) | (1 << CMD_H1_PIN);
    break;

  default:
    break;
  }
}

//-----------------------------------------------------------------------------
/*!
 * \fn void Timer_SetMft1Timming(uint16_t ui16Time, bool bOneShotTimer)
 * \brief Fonction permettant d'initialiser le timers pour d�clencher au bout de ui16Time �s avec option de r�p�tition
 * \param ui16Time = temps exprim� en �s
 * \param bOneShotTimer = � true indique que le timer est d�clench� une fois sinon est r�p�t� ui16Time �s
 */
void Timer_SetMft1Timming(uint32_t ui16Time)
{
  /*! Variable indiquant si le timer doit �tre arr�t pour ne faire qu'une interruption */
  uint32_t Gui16CounterMft1Timer = 0;

  if (ui16Time >= uiMIN_TIMER_MFT1)
  {
    Gui16CounterMft1Timer = ui16Time - uiERREUR_TIMER_MFT1;
  }
  else
  {
    Gui16CounterMft1Timer = 2U;
  }
  /* D�marrage du timer */
  set_timer0_time(Gui16CounterMft1Timer);
}

void ImpulsBiphasCompens(void)
{
  static uint16_t i = 0, Nloop = 0, flag = TRUE;
  uint16_t tmp = 0;
  static uint8_t pulseId = 0;

  /** Biphasic pulse states */
  static enum {
    gBiphasStatePos_c = 0, /**< Positive Pulse */
    gBiphasStateNeg_c,     /**< Negative Pulse */
    gBiphasStateInter_c,   /**< Inter Pulse */
    gBiphasStateNeg_c1,
    gBiphasStateNull_c, /**< Null Pulse */
    gBiphasStateNullLoop_c,
    gBiphasStateMax_c
  } tBiphasState = gBiphasStatePos_c;

  //	p0_0 = 1; // pin test

  switch (tBiphasState)
  {
  /** Positive Pulse */
  case gBiphasStatePos_c:

    STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */

    CMD_M_SET_NO_PULSE;

    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));
    Gpio_SetAop();
    tmp = NORMAL_MODE | ((gStimGen_t.tPulse[i].digitalAmplitude & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    CMD_M_SET_POSITIVE_PULSE; /**< Enables Positive pulse CMD */

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);

    tBiphasState = gBiphasStateNeg_c;

    break;

  /** Negative Pulse */
  case gBiphasStateNeg_c:

    //	    /** Current Measurement */
    //	    gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
    //	    gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
    //	    gflag[i] = TRUE;
    if (flag)
    {
      CMD_M_DISCONNECT; /**< Disables pulse CMD */

      /** Sets Commands */

      Gpio_SetElectrostimulation(eETAPE4);
      Gpio_SetElectrostimulation(eETAPE6);

      flag = false;
    }

    tmp = NORMAL_MODE | ((TablePuissanceCalculeeCompens[i][pulseId] & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));
    pulseId++;
    if (pulseId == gNbCompensValue - 1)
    {
      tBiphasState = gBiphasStateNeg_c1;
      flag = false;
      pulseId = 0;
    }
    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth / gNbCompensValue);

    break;

  case gBiphasStateNeg_c1:
  {
    CMD_M_DISCONNECT;   /**< Disables pulse CMD */
    CMD_M_SET_NO_PULSE; /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE;  /**< Disables Pulse Output */

    //  Application AOP -> OFF
    // Gpio_ClrAop();
    /** Sets Next Step Time */
    switch (gStimGen_t.FreqDiff)
    {
    case 1:

      if (cptFreqDiff < gStimGen_t.Ratio)
      {
        cptFreqDiff++;
        if (i == 1)
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
        else
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      else
      {
        cptFreqDiff = 0;
        if (++i < gStimGen_t.nPulse) /**< Next Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
          tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
        }
        else /**< No Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }
      break;

    case 0:

      if (++i < gStimGen_t.nPulse) /**< Next Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else /**< No Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      break;

    case -1:

      if (i == 0)
      {
        i = 1;
        cptFreqDiff++;
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else
      {
        if (cptFreqDiff < gStimGen_t.Ratio)
        {
          cptFreqDiff++;
          i = 1;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          tBiphasState = gBiphasStateNull_c;
        }
        else
        {
          cptFreqDiff = 0;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }

      break;
    }

    break;
  }

  /** Inter Pulse */
  case gBiphasStateInter_c:

    /** Sets Commands */
    CMD_M_DISCONNECT;  /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

    /** Sets Level */
    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    // Gpio_ClrAop();
    /** Sets pause cmd */
    // CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1200);
    tBiphasState = gBiphasStatePos_c;

    break;

  /** Null Pulse */
  case gBiphasStateNull_c:

    /** Sets Commands */
    CMD_M_DISCONNECT;  /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

    /** Sets Level */
    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    /** Sets pause cmd */
    CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    if (gStimGen_t.nTimerLoop > 0) /**< Next Loop */
    {
      Nloop = gStimGen_t.nTimerLoop;
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
      tBiphasState = gBiphasStateNullLoop_c;
    }
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1333);
      tBiphasState = gBiphasStatePos_c;
      if (flag == FALSE)
        flag = TRUE; /**< Enables Positive pulse CMD */
      else
        flag = FALSE;
    }

    break;

  case gBiphasStateNullLoop_c:

    Nloop--;
    if (Nloop > 0) /**< Next Loop */
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1333);
      tBiphasState = gBiphasStatePos_c;
      if (flag == FALSE)
        flag = TRUE; /**< Enables Positive pulse CMD */
      else
        flag = FALSE;
    }

    break;

  default:
    /** TODO !!!!! */
    break;
  }
  //	p0_0 = 0; // pin test
}
/************************************************************************************
 * Name :	ImpulsBiphas 	*/
/**
 * @brief	Biphas Impuls
 * @param	.
 * @return	.
 ************************************************************************************/
void ImpulsBiphasSynchro(void)
{
  static uint16_t i = 0, Nloop = 0;
  uint16_t tmp = 0;

  /** Biphasic pulse states */
  static enum {
    gBiphasStatePos_c = 0, /**< Positive Pulse */
    gBiphasStateNeg_c,     /**< Negative Pulse */
    gBiphasStateInter_c,   /**< Inter Pulse */
    gBiphasStateNeg_c1,
    gBiphasStateNull_c, /**< Null Pulse */
    gBiphasStateNullLoop_c,
    gBiphasStateWait_c,
    gBiphasStateMax_c
  } tBiphasState = gBiphasStatePos_c;

  //	p0_0 = 1; // pin test

  switch (tBiphasState)
  {
  /** Positive Pulse */
  case gBiphasStatePos_c:

    STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */

    CMD_M_SET_NO_PULSE;

    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));
    Gpio_SetAop();
    tmp = NORMAL_MODE | ((gStimGen_t.tPulse[i].digitalAmplitude & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    CMD_M_SET_POSITIVE_PULSE; /**< Enables Positive pulse CMD */

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);

    tBiphasState = gBiphasStateWait_c;

    break;

  case gBiphasStateWait_c:
  {
    Gpio_SetElectrostimulation(eETAPE4);
    CMD_M_SET_NO_PULSE;
    STIM_GEN_RELOAD_NEXT_COUNT(158 * 20);

    tBiphasState = gBiphasStateNeg_c;

    break;
  }
  /** Negative Pulse */
  case gBiphasStateNeg_c:

    /** Current Measurement */
    gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
    gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
    gflag[i] = TRUE;

    CMD_M_DISCONNECT; /**< Disables pulse CMD */

    /** Sets Commands */

    Gpio_SetElectrostimulation(eETAPE4);
    Gpio_SetElectrostimulation(eETAPE6);

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
    tBiphasState = gBiphasStateNeg_c1;
    break;

  case gBiphasStateNeg_c1:
  {
    CMD_M_DISCONNECT;   /**< Disables pulse CMD */
    CMD_M_SET_NO_PULSE; /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE;  /**< Disables Pulse Output */

    //  Application AOP -> OFF
    // Gpio_ClrAop();
    /** Sets Next Step Time */
    switch (gStimGen_t.FreqDiff)
    {
    case 1:

      if (cptFreqDiff < gStimGen_t.Ratio)
      {
        cptFreqDiff++;
        if (i == 1)
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
        else
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      else
      {
        cptFreqDiff = 0;
        if (++i < gStimGen_t.nPulse) /**< Next Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
          tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
        }
        else /**< No Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }
      break;

    case 0:

      if (++i < gStimGen_t.nPulse) /**< Next Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else /**< No Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      break;

    case -1:

      if (i == 0)
      {
        i = 1;
        cptFreqDiff++;
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else
      {
        if (cptFreqDiff < gStimGen_t.Ratio)
        {
          cptFreqDiff++;
          i = 1;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          tBiphasState = gBiphasStateNull_c;
        }
        else
        {
          cptFreqDiff = 0;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }

      break;
    }

    break;
  }

  /** Inter Pulse */
  case gBiphasStateInter_c:

    /** Sets Commands */
    CMD_M_DISCONNECT;  /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

    /** Sets Level */
    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    // Gpio_ClrAop();
    /** Sets pause cmd */
    // CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1200);
    tBiphasState = gBiphasStatePos_c;

    break;

  /** Null Pulse */
  case gBiphasStateNull_c:

    /** Sets Commands */
    CMD_M_DISCONNECT;  /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

    /** Sets Level */
    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    /** Sets pause cmd */
    CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    if (gStimGen_t.nTimerLoop > 0) /**< Next Loop */
    {
      Nloop = gStimGen_t.nTimerLoop;
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
      tBiphasState = gBiphasStateNullLoop_c;
    }
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1333);
      tBiphasState = gBiphasStatePos_c;
    }

    break;

  case gBiphasStateNullLoop_c:

    Nloop--;
    if (Nloop > 0) /**< Next Loop */
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1333);
      tBiphasState = gBiphasStatePos_c;
    }

    break;

  default:
    /** TODO !!!!! */
    break;
  }
  //	p0_0 = 0; // pin test
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :	ImpulsBiphasNeg 	*/
/**
 * @brief	Biphas Negative Impuls
 * @param	.
 * @return	.
 ************************************************************************************/

void ImpulsBiphas(void)
{
  static uint16_t i = 0, Nloop = 0;
  uint16_t tmp = 0;

  /** Biphasic pulse states */
  static enum {
    gBiphasStatePos_c = 0, /**< Positive Pulse */
    gBiphasStateNeg_c,     /**< Negative Pulse */
    gBiphasStateInter_c,   /**< Inter Pulse */
    gBiphasStateNeg_c1,
    gBiphasStateNull_c, /**< Null Pulse */
    gBiphasStateNullLoop_c,
    gBiphasStateMax_c
  } tBiphasState = gBiphasStatePos_c;

  //	p0_0 = 1; // pin test

  switch (tBiphasState)
  {
  /** Positive Pulse */
  case gBiphasStatePos_c:

    STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */

    CMD_M_SET_NO_PULSE;

    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));
    Gpio_SetAop();
    tmp = NORMAL_MODE | ((gStimGen_t.tPulse[i].digitalAmplitude & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    CMD_M_SET_POSITIVE_PULSE; /**< Enables Positive pulse CMD */

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);

    tBiphasState = gBiphasStateNeg_c;

    break;

  /** Negative Pulse */
  case gBiphasStateNeg_c:

    /** Current Measurement */
    gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
    gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
    gflag[i] = TRUE;

    CMD_M_DISCONNECT; /**< Disables pulse CMD */

    /** Sets Commands */

    Gpio_SetElectrostimulation(eETAPE4);
    Gpio_SetElectrostimulation(eETAPE6);

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
    tBiphasState = gBiphasStateNeg_c1;
    break;

  case gBiphasStateNeg_c1:
  {
    CMD_M_DISCONNECT;   /**< Disables pulse CMD */
    CMD_M_SET_NO_PULSE; /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE;  /**< Disables Pulse Output */
                        /** Sets Next Step Time */
    switch (gStimGen_t.FreqDiff)
    {
    case 1:

      if (cptFreqDiff < gStimGen_t.Ratio)
      {
        cptFreqDiff++;
        if (i == 1)
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
        else
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      else
      {
        cptFreqDiff = 0;
        if (++i < gStimGen_t.nPulse) /**< Next Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
          tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
        }
        else /**< No Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }
      break;

    case 0:

      if (++i < gStimGen_t.nPulse) /**< Next Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else /**< No Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - 1200 - 162 * 20);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      break;

    case -1:

      if (i == 0)
      {
        i = 1;
        cptFreqDiff++;
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else
      {
        if (cptFreqDiff < gStimGen_t.Ratio)
        {
          cptFreqDiff++;
          i = 1;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          tBiphasState = gBiphasStateNull_c;
        }
        else
        {
          cptFreqDiff = 0;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }

      break;
    }

    break;
  }

  /** Inter Pulse */
  case gBiphasStateInter_c:
	  /** Sets Commands */
	      CMD_M_DISCONNECT;  /**< Disables pulse CMD */
	      STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

	      /** Sets Level */
	      tmp = NORMAL_MODE;
	      SPI_TRANSMIT_DATA(MSB(tmp));
	      SPI_TRANSMIT_DATA(LSB(tmp));

	      /** Sets pause cmd */
	      CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(1200);
    tBiphasState = gBiphasStatePos_c;

    break;

  /** Null Pulse */
  case gBiphasStateNull_c:

    /** Sets Commands */
    CMD_M_DISCONNECT;  /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

    /** Sets Level */
    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    /** Sets pause cmd */
    CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    if (gStimGen_t.nTimerLoop > 0) /**< Next Loop */
    {
      Nloop = gStimGen_t.nTimerLoop;
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
      tBiphasState = gBiphasStateNullLoop_c;
    }
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(1200);
      tBiphasState = gBiphasStatePos_c;
    }

    break;

  case gBiphasStateNullLoop_c:

    Nloop--;
    if (Nloop > 0) /**< Next Loop */
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1333);
      tBiphasState = gBiphasStatePos_c;
    }

    break;

  default:
    /** TODO !!!!! */
    break;
  }
  //	p0_0 = 0; // pin test
}
////////////////////////////////////////////////////////////////// LIO
void ImpulsBiphasNeg(void)
{
  static uint16_t i = 0, Nloop = 0;
  uint16_t tmp = 0;

  /** Biphasic pulse states */
  static enum {
    gBiphasStatePos_c = 0, /**< Positive Pulse */
    gBiphasStateNeg_c,     /**< Negative Pulse */
    gBiphasStateInter_c,   /**< Inter Pulse */
    gBiphasStateNeg_c1,
    gBiphasStateNull_c, /**< Null Pulse */
    gBiphasStateNullLoop_c,
    gBiphasStateMax_c
  } tBiphasState = gBiphasStatePos_c;

  //	p0_0 = 1; // pin test

  switch (tBiphasState)
  {
  /** Positive Pulse */
  case gBiphasStatePos_c:

    STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */

    CMD_M_SET_NO_PULSE;

    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));
    Gpio_SetAop();
    tmp = NORMAL_MODE | ((gStimGen_t.tPulse[i].digitalAmplitude & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    CMD_M_SET_NEGATIVE_PULSE;

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);

    tBiphasState = gBiphasStateNeg_c;

    break;

  /** Negative Pulse */
  case gBiphasStateNeg_c:

    /** Current Measurement */
    gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
    gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
    gflag[i] = TRUE;

    CMD_M_DISCONNECT; /**< Disables pulse CMD */

    /** Sets Commands */
    Gpio_SetElectrostimulation(eETAPE9);
    Gpio_SetElectrostimulation(eETAPE10);

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
    tBiphasState = gBiphasStateNeg_c1;
    break;

  case gBiphasStateNeg_c1:
  {
    CMD_M_DISCONNECT;   /**< Disables pulse CMD */
    CMD_M_SET_NO_PULSE; /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE;  /**< Disables Pulse Output */

    //  Application AOP -> OFF
    // //Gpio_ClrAop()();
    /** Sets Next Step Time */
    switch (gStimGen_t.FreqDiff)
    {
    case 1:

      if (cptFreqDiff < gStimGen_t.Ratio)
      {
        cptFreqDiff++;
        if (i == 1)
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
        else
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      else
      {
        cptFreqDiff = 0;
        if (++i < gStimGen_t.nPulse) /**< Next Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
          tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
        }
        else /**< No Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }
      break;

    case 0:

      if (++i < gStimGen_t.nPulse) /**< Next Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else /**< No Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      break;

    case -1:

      if (i == 0)
      {
        i = 1;
        cptFreqDiff++;
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else
      {
        if (cptFreqDiff < gStimGen_t.Ratio)
        {
          cptFreqDiff++;
          i = 1;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          tBiphasState = gBiphasStateNull_c;
        }
        else
        {
          cptFreqDiff = 0;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }

      break;
    }

    break;
  }

  /** Inter Pulse */
  case gBiphasStateInter_c:

    /** Sets Commands */
    CMD_M_DISCONNECT;  /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

    /** Sets Level */
    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    /** Sets pause cmd */
    CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1200);
    tBiphasState = gBiphasStatePos_c;

    break;

  /** Null Pulse */
  case gBiphasStateNull_c:

    /** Sets Commands */
    CMD_M_DISCONNECT;  /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

    /** Sets Level */
    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    /** Sets pause cmd */
    CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    if (gStimGen_t.nTimerLoop > 0) /**< Next Loop */
    {
      Nloop = gStimGen_t.nTimerLoop;
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
      tBiphasState = gBiphasStateNullLoop_c;
    }
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1333);
      tBiphasState = gBiphasStatePos_c;
    }

    break;

  case gBiphasStateNullLoop_c:

    Nloop--;
    if (Nloop > 0) /**< Next Loop */
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1333);
      tBiphasState = gBiphasStatePos_c;
    }

    break;

  default:
    /** TODO !!!!! */
    break;
  }

  //	p0_0 = 0; // pin test
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :	ImpulsMonophas 	*/
/**
 * @brief	Monophas Impuls
 * @param	.
 * @return	.
 ************************************************************************************/
void ImpulsMonophas(void)
{
  static uint16_t i = 0, Nloop = 0;
  uint16_t tmp = 0;

  /** Biphasic pulse states */
  static enum {
    gBiphasStatePos_c = 0, /**< Positive Pulse */
    gBiphasStateNeg_c,     /**< Negative Pulse */
    gBiphasStateInter_c,   /**< Inter Pulse */
    gBiphasStateNeg_c1,
    gBiphasStateNull_c, /**< Null Pulse */
    gBiphasStateNullLoop_c,
    gBiphasStateMax_c
  } tBiphasState = gBiphasStatePos_c;

  //	p0_0 = 1; // pin test

  switch (tBiphasState)
  {
  /** Positive Pulse */
  case gBiphasStatePos_c:

    STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */

    CMD_M_SET_NO_PULSE;

    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));
    Gpio_SetAop();
    tmp = NORMAL_MODE | ((gStimGen_t.tPulse[i].digitalAmplitude & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    CMD_M_SET_POSITIVE_PULSE; /**< Enables Positive pulse CMD */

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);

    tBiphasState = gBiphasStateNeg_c1;

    break;

  /** Negative Pulse */
  case gBiphasStateNeg_c:

    /** Current Measurement */
    gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
    gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
    gflag[i] = TRUE;

    CMD_M_DISCONNECT; /**< Disables pulse CMD */

    /** Sets Commands */
    Gpio_SetElectrostimulation(eETAPE4);
    Gpio_SetElectrostimulation(eETAPE6);

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
    tBiphasState = gBiphasStateNeg_c1;
    break;

  case gBiphasStateNeg_c1:
  {
    CMD_M_DISCONNECT;   /**< Disables pulse CMD */
    CMD_M_SET_NO_PULSE; /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE;  /**< Disables Pulse Output */

    //  Application AOP -> OFF
    // //Gpio_ClrAop()();
    /** Sets Next Step Time */
    switch (gStimGen_t.FreqDiff)
    {
    case 1:

      if (cptFreqDiff < gStimGen_t.Ratio)
      {
        cptFreqDiff++;
        if (i == 1)
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
        else
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      else
      {
        cptFreqDiff = 0;
        if (++i < gStimGen_t.nPulse) /**< Next Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
          tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
        }
        else /**< No Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }
      break;

    case 0:

      if (++i < gStimGen_t.nPulse) /**< Next Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else /**< No Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      break;

    case -1:

      if (i == 0)
      {
        i = 1;
        cptFreqDiff++;
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else
      {
        if (cptFreqDiff < gStimGen_t.Ratio)
        {
          cptFreqDiff++;
          i = 1;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          tBiphasState = gBiphasStateNull_c;
        }
        else
        {
          cptFreqDiff = 0;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }

      break;
    }

    break;
  }

  /** Inter Pulse */
  case gBiphasStateInter_c:

    /** Sets Commands */
    CMD_M_DISCONNECT;  /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

    /** Sets Level */
    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    /** Sets pause cmd */
    CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1200);
    tBiphasState = gBiphasStatePos_c;

    break;

  /** Null Pulse */
  case gBiphasStateNull_c:

    /** Sets Commands */
    CMD_M_DISCONNECT;  /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

    /** Sets Level */
    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    /** Sets pause cmd */
    CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    if (gStimGen_t.nTimerLoop > 0) /**< Next Loop */
    {
      Nloop = gStimGen_t.nTimerLoop;
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
      tBiphasState = gBiphasStateNullLoop_c;
    }
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1333);
      tBiphasState = gBiphasStatePos_c;
    }

    break;

  case gBiphasStateNullLoop_c:

    Nloop--;
    if (Nloop > 0) /**< Next Loop */
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1333);
      tBiphasState = gBiphasStatePos_c;
    }

    break;

  default:
    /** TODO !!!!! */
    break;
  }

  //	p0_0 = 0; // pin test
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :	Galvanic 	*/
/**
 * @brief	Direct Current
 * @param	.
 * @return	.
 ************************************************************************************/

void Galvanic(void)
{
  static uint16_t i = 0;
  static uint16_t tmp = 0 /*,tmp2=0*/;
  static bool flag = true;
  /** Monophasic pulse states */
  static enum {
    gWidthStatePos_c = 1,  /**< Positive Current */
    gWidthStateNeg_c = 0,  /**< Negative Current */
    gWidthStateNeg_c1 = 3, /**< Negative Current */
    gWidthStateNull_c = 2, /**< Null Pulse */
    gWidthStateMax_c
  } tWidthState = gWidthStateNeg_c;

  //  if (gStimGen_t.FreqDiff == 0)
  //  {
  //    tWidthState = gWidthStateNeg_c;
  //  }
  //  else
  //  {
  //    tWidthState = gWidthStatePos_c;
  //  }

  switch (tWidthState)
  {

  /** Positive Current */
  case gWidthStatePos_c:
    //
    //    tmp = NORMAL_MODE | (((gStimGen_t.tPulse[i].digitalAmplitude) & 0x0FFF) << 2);
    //    if (gStimGen_t.tPulse[i].digitalAmplitude == 0)
    //    {
    //      CMD_GALV_SEL_NONE;
    //    }
    //    else
    //    {
    //      SPI_TRANSMIT_DATA(MSB(tmp));
    //      SPI_TRANSMIT_DATA(LSB(tmp));
    //
    //      /** Sets Commands */
    //      STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */
    //      CMD_GALV_SEL_POS;                                        /**< Enables positive current. */
    //
    //      //     gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
    //    }

    break;

  /** Negative Current */
  case gWidthStateNeg_c:
  {
    if (flag)
    {
      STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */
      flag = false;
      CMD_M_DISCONNECT;
    }

    Gpio_SetAop();
    tmp = NORMAL_MODE | ((gStimGen_t.tPulse[i].digitalAmplitude & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    CMD_GALV_SEL_NEG;
    // CMD_M_SET_POSITIVE_PULSE;
    tWidthState = gWidthStateNeg_c1;
    // sl_sleeptimer_delay_millisecond(1);
    STIM_GEN_RELOAD_NEXT_COUNT(2000 * 20);
    break;
  }

  case gWidthStateNeg_c1:
  {
    GPIO->P_CLR[CMD_L1_PORT].DOUT = (1 << CMD_L1_PIN);

    tWidthState = gWidthStateNeg_c;
    STIM_GEN_RELOAD_NEXT_COUNT(20);
    break;
  }
  /** Null Current */
  case gWidthStateNull_c:

    CMD_GALV_SEL_NONE;

    break;
  }
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :	ImpulsBiphas 	*/
/**
 * @brief	Biphas Impuls
 * @param	.
 * @return	.
 ************************************************************************************/
void ImpulsBiphasAltern(void)
{
  static uint16_t i = 0, Nloop = 0, flag = FALSE;
  uint16_t tmp = 0;

  /** Biphasic pulse states */
  static enum {
    gBiphasStatePos_c = 0, /**< Positive Pulse */
    gBiphasStateNeg_c,     /**< Negative Pulse */
    gBiphasStateInter_c,   /**< Inter Pulse */
    gBiphasStateNeg_c1,
    gBiphasStateNull_c, /**< Null Pulse */
    gBiphasStateNullLoop_c,
    gBiphasStateMax_c
  } tBiphasState = gBiphasStatePos_c;

  //	p0_0 = 1; // pin test

  switch (tBiphasState)
  {
  /** Positive Pulse */
  case gBiphasStatePos_c:

    STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */

    CMD_M_SET_NO_PULSE;

    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));
    Gpio_SetAop();
    tmp = NORMAL_MODE | ((gStimGen_t.tPulse[i].digitalAmplitude & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    if (flag == FALSE)
      CMD_M_SET_POSITIVE_PULSE; /**< Enables Positive pulse CMD */
    else
      CMD_M_SET_NEGATIVE_PULSE;

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);

    tBiphasState = gBiphasStateNeg_c;

    break;

  /** Negative Pulse */
  case gBiphasStateNeg_c:

    /** Current Measurement */
    gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
    gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
    gflag[i] = TRUE;

    CMD_M_DISCONNECT; /**< Disables pulse CMD */

    /** Sets Commands */
    if (flag == FALSE)
    {
      Gpio_SetElectrostimulation(eETAPE4);
      Gpio_SetElectrostimulation(eETAPE6);
    }
    else
    {
      Gpio_SetElectrostimulation(eETAPE9);
      Gpio_SetElectrostimulation(eETAPE10);
    }
    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
    tBiphasState = gBiphasStateNeg_c1;
    break;

  case gBiphasStateNeg_c1:
  {
    CMD_M_DISCONNECT;   /**< Disables pulse CMD */
    CMD_M_SET_NO_PULSE; /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE;  /**< Disables Pulse Output */

    //  Application AOP -> OFF
    // //Gpio_ClrAop()();
    /** Sets Next Step Time */
    switch (gStimGen_t.FreqDiff)
    {
    case 1:

      if (cptFreqDiff < gStimGen_t.Ratio)
      {
        cptFreqDiff++;
        if (i == 1)
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
        else
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      else
      {
        cptFreqDiff = 0;
        if (++i < gStimGen_t.nPulse) /**< Next Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
          tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
        }
        else /**< No Pulse */
        {
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }
      break;

    case 0:

      if (++i < gStimGen_t.nPulse) /**< Next Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else /**< No Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        i = 0;
        tBiphasState = gBiphasStateNull_c;
      }
      break;

    case -1:

      if (i == 0)
      {
        i = 1;
        cptFreqDiff++;
        STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/ 11172); //!!!!!
        tBiphasState = /*gBiphasStatePos_c*/ gBiphasStateInter_c;
      }
      else
      {
        if (cptFreqDiff < gStimGen_t.Ratio)
        {
          cptFreqDiff++;
          i = 1;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
          tBiphasState = gBiphasStateNull_c;
        }
        else
        {
          cptFreqDiff = 0;
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr - (11172 + 2 * gStimGen_t.tPulse[i].cntWidth));
          i = 0;
          tBiphasState = gBiphasStateNull_c;
        }
      }

      break;
    }

    break;
  }

  /** Inter Pulse */
  case gBiphasStateInter_c:

    /** Sets Commands */
    CMD_M_DISCONNECT;  /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

    /** Sets Level */
    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    /** Sets pause cmd */
    CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1200);
    tBiphasState = gBiphasStatePos_c;

    break;

  /** Null Pulse */
  case gBiphasStateNull_c:

    /** Sets Commands */
    CMD_M_DISCONNECT;  /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */

    /** Sets Level */
    tmp = NORMAL_MODE;
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    /** Sets pause cmd */
    CMD_M_SET_NO_PULSE;

    /** Sets Next Step Time */
    if (gStimGen_t.nTimerLoop > 0) /**< Next Loop */
    {
      Nloop = gStimGen_t.nTimerLoop;
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
      tBiphasState = gBiphasStateNullLoop_c;
    }
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1333);
      tBiphasState = gBiphasStatePos_c;
      if (flag == FALSE)
        flag = TRUE; /**< Enables Positive pulse CMD */
      else
        flag = FALSE;
    }

    break;

  case gBiphasStateNullLoop_c:

    Nloop--;
    if (Nloop > 0) /**< Next Loop */
      STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
    else /**< First Pulse */
    {
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth + 1333);
      tBiphasState = gBiphasStatePos_c;
      if (flag == FALSE)
        flag = TRUE; /**< Enables Positive pulse CMD */
      else
        flag = FALSE;
    }

    break;

  default:
    /** TODO !!!!! */
    break;
  }

  //	p0_0 = 0; // pin test
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
 * Name :	VeineuxBiphas 	*/
/**
 * @brief	Veineux biphasic Impuls
 * @param	.
 * @return	.
 ************************************************************************************/
void VeineuxBiphas(void)
{
  static uint8_t u8PulseStepIdx = 0;
  static uint16_t i = 0;
  static uint16_t DigAmp = 0;
  static uint16_t tmp = 0;
  /** Veineux Biphasic pulse states */
  static enum {
    gVeineuxBiphaseInit_c = 0,
    gVeineuxBiphasePosIncr_c,
    gVeineuxBiphasePosDecr_c,
    gVeineuxBiphaseNegIncr_c,
    gVeineuxBiphaseNegDecr_c,
    gVeineuxBiphaseWait_c,
    gVeineuxBiphaseWaitNeg_c,
    gVeineuxBiphasStateMax_c
  } tVeineuxBiphasState = gVeineuxBiphaseInit_c;

  switch (tVeineuxBiphasState)
  {
  case gVeineuxBiphaseInit_c:
  {
    CMD_M_DISCONNECT;

    gflag[0] = TRUE;

    STIM_GEN_RELOAD_NEXT_COUNT(iSS_TIMMING_COURT * 20);
    tVeineuxBiphasState = gVeineuxBiphasePosIncr_c;
    break;
  }

  case gVeineuxBiphasePosIncr_c:
  {
    if (gflag[0] == TRUE)
    {
      STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */

      CMD_M_SET_NO_PULSE;

      tmp = NORMAL_MODE;
      SPI_TRANSMIT_DATA(MSB(tmp));
      SPI_TRANSMIT_DATA(LSB(tmp));
      Gpio_SetAop();

      gflag[0] = FALSE;
    }

    tmp = NORMAL_MODE | ((TablePuissanceCalculee[i][u8PulseStepIdx] & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    CMD_M_SET_POSITIVE_PULSE; /**< Enables Positive pulse CMD */

    u8PulseStepIdx++;
    if (u8PulseStepIdx >= gNbExponentialValue_c - 1)
    {
      gflag[0] = TRUE;
      // STIM_OUT_SEL_NONE;
      u8PulseStepIdx = 0;
      tVeineuxBiphasState = gVeineuxBiphasePosDecr_c;
    }
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
    break;
  }

  case gVeineuxBiphasePosDecr_c:
  {
    tmp = gStimGen_t.tPulse[i].digitalAmplitude - TablePuissanceCalculee[i][u8PulseStepIdx];
    DigAmp = NORMAL_MODE | ((tmp & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(DigAmp));
    SPI_TRANSMIT_DATA(LSB(DigAmp));

    u8PulseStepIdx++;
    if (u8PulseStepIdx == gNbExponentialValue_c - 1)
    {
      gflag[0] = TRUE;
      // STIM_OUT_SEL_NONE;
      u8PulseStepIdx = 0;
      tVeineuxBiphasState = gVeineuxBiphaseWait_c;
    }
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
    break;
  }

  case gVeineuxBiphaseWait_c:
  {
    CMD_M_DISCONNECT;   /**< Disables pulse CMD */
    CMD_M_SET_NO_PULSE; /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE;  /**< Disables Pulse Output */
    if (cpt > 0)
    {
      tVeineuxBiphasState = gVeineuxBiphasePosIncr_c;
      cpt--;
    }
    else
    {
      tVeineuxBiphasState = gVeineuxBiphaseNegIncr_c;
      cpt = 7;
    }

    gflag[0] = TRUE;
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
    break;
  }

  case gVeineuxBiphaseNegIncr_c:
  {
    if (gflag[0] == TRUE)
    {
      STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */

      CMD_M_SET_NO_PULSE;

      tmp = NORMAL_MODE;
      SPI_TRANSMIT_DATA(MSB(tmp));
      SPI_TRANSMIT_DATA(LSB(tmp));
      Gpio_SetAop();

      gflag[0] = FALSE;
    }

    tmp = NORMAL_MODE | ((TablePuissanceCalculee[i][u8PulseStepIdx] & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));

    CMD_M_SET_NEGATIVE_PULSE; /**< Enables Positive pulse CMD */

    u8PulseStepIdx++;
    if (u8PulseStepIdx >= gNbExponentialValue_c - 1)
    {
      gflag[0] = TRUE;
      // STIM_OUT_SEL_NONE;
      u8PulseStepIdx = 0;
      tVeineuxBiphasState = gVeineuxBiphaseNegDecr_c;
    }
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
    break;
  }

  case gVeineuxBiphaseNegDecr_c:
  {
    tmp = gStimGen_t.tPulse[i].digitalAmplitude - TablePuissanceCalculee[i][u8PulseStepIdx];
    DigAmp = NORMAL_MODE | ((tmp & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(DigAmp));
    SPI_TRANSMIT_DATA(LSB(DigAmp));

    u8PulseStepIdx++;
    if (u8PulseStepIdx == gNbExponentialValue_c - 1)
    {
      gflag[0] = TRUE;
      // STIM_OUT_SEL_NONE;
      u8PulseStepIdx = 0;
      tVeineuxBiphasState = gVeineuxBiphaseWaitNeg_c;
    }
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
    break;
  }

  case gVeineuxBiphaseWaitNeg_c:
  {
    CMD_M_DISCONNECT;   /**< Disables pulse CMD */
    CMD_M_SET_NO_PULSE; /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE;  /**< Disables Pulse Output */
    if (cpt > 0)
    {
      tVeineuxBiphasState = gVeineuxBiphaseNegIncr_c;
      cpt--;
    }
    else
    {
      tVeineuxBiphasState = gVeineuxBiphasePosIncr_c;
      cpt = 7;
    }

    gflag[0] = TRUE;
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
    break;
  }

  default:
  {
    break;
  }
  }
}

void Sinus(void)
{
  static uint8_t u8PulseStepIdx = 0;
  static uint16_t i = 0;
  static uint16_t DigAmp = 0;
  uint16_t tmp = 0;

  /** Veineux Biphasic pulse states */
  static enum {
    gVeineuxBiphaseInit_c = 0,
    gVeineuxBiphaseNegInit,
    gVeineuxBiphaseNoPulse,
    gVeineuxBiphasePos_c,
    gVeineuxBiphasePosIncr_c,
    gVeineuxBiphasePosDecr_c,
    gVeineuxBiphaseNeg_c,
    gVeineuxBiphaseNegNoPulse_c,
    gVeineuxBiphaseNegIncr_c,
    gVeineuxBiphaseNegDecr_c,
    gVeineuxBiphaseWait_c,
    gVeineuxBiphaseWaitNeg_c,
    gVeineuxBiphasStateMax_c
  } tVeineuxBiphasState = gVeineuxBiphaseInit_c;

  switch (tVeineuxBiphasState)
  {
  case gVeineuxBiphaseInit_c:
  {
    // STIM_OUT_SEL_NONE;
    Gpio_SetElectrostimulation(eETAPE1);

    gflag[0] = TRUE;

    STIM_GEN_RELOAD_NEXT_COUNT(iSS_TIMMING_COURT * 20);
    tVeineuxBiphasState = gVeineuxBiphaseNoPulse;
    break;
  }

  case gVeineuxBiphaseNoPulse:
  {
    CMD_M_SET_NO_PULSE;
    Gpio_SetAop();
    PETIT_DELAI_NOP;
    tmp = 400;
    DigAmp = NORMAL_MODE | ((tmp & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(DigAmp));
    SPI_TRANSMIT_DATA(LSB(DigAmp));
    STIM_GEN_RELOAD_NEXT_COUNT(100 * 20);
    tVeineuxBiphasState = gVeineuxBiphasePos_c;
    break;
  }

  case gVeineuxBiphasePos_c:
  {
    Gpio_SetElectrostimulation(eETAPE3); // Haut L1-H2 / CLR L1
    STIM_GEN_RELOAD_NEXT_COUNT(iSS_MOMENT_RELECTURE_COURANT * 20);
    tVeineuxBiphasState = gVeineuxBiphasePosIncr_c;
    break;
  }

  case gVeineuxBiphasePosIncr_c:
  {
    if (gflag[0] == TRUE)
    {
      STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */
      gflag[0] = FALSE;
    }

    tmp = (uint16_t)TablePuissanceCalculeeSinus[i][u8PulseStepIdx];
    DigAmp = NORMAL_MODE | ((tmp & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(DigAmp));
    SPI_TRANSMIT_DATA(LSB(DigAmp));

    u8PulseStepIdx++;
    if (u8PulseStepIdx >= gNbSinusValue_c / 2 - 1)
    {
      gflag[0] = TRUE;
      // STIM_OUT_SEL_NONE;
      // u8PulseStepIdx = 0;
      tVeineuxBiphasState = gVeineuxBiphaseWait_c;
    }
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr * 20);
    break;
  }

  case gVeineuxBiphaseWait_c:
  {
    STIM_OUT_SEL_NONE;
    Gpio_SetElectrostimulation(eETAPE4);
    tVeineuxBiphasState = gVeineuxBiphaseNegNoPulse_c;

    STIM_GEN_RELOAD_NEXT_COUNT(80 * 20);
    // Gpio_ClrAop()();
    gflag[0] = TRUE;
    break;
  }

  case gVeineuxBiphaseNegInit:
  {
    Gpio_SetElectrostimulation(eETAPE1);

    gflag[0] = TRUE;

    STIM_GEN_RELOAD_NEXT_COUNT(iSS_TIMMING_COURT * 20);
    tVeineuxBiphasState = gVeineuxBiphaseNegNoPulse_c;
    break;
  }

  case gVeineuxBiphaseNegNoPulse_c:
  {
    CMD_M_SET_NO_PULSE;
    Gpio_SetAop();
    PETIT_DELAI_NOP;
    tmp = 400;
    DigAmp = NORMAL_MODE | ((tmp & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(DigAmp));
    SPI_TRANSMIT_DATA(LSB(DigAmp));
    STIM_GEN_RELOAD_NEXT_COUNT(100 * 20);
    tVeineuxBiphasState = gVeineuxBiphaseNeg_c;
    break;
  }

  case gVeineuxBiphaseNeg_c:
  {
    CMD_M_SET_NEGATIVE_PULSE;
    STIM_GEN_RELOAD_NEXT_COUNT(iSS_MOMENT_RELECTURE_COURANT * 20);
    tVeineuxBiphasState = gVeineuxBiphaseNegIncr_c;
    break;
  }

  case gVeineuxBiphaseNegIncr_c:
  {
    if (gflag[0] == TRUE)
    {
      STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */
      gflag[0] = FALSE;
    }

    tmp = TablePuissanceCalculeeSinus[i][u8PulseStepIdx]; //(uint16_t) TablePuissanceCalculee[i][u8PulseStepIdx];
    DigAmp = NORMAL_MODE | ((tmp & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(DigAmp));
    SPI_TRANSMIT_DATA(LSB(DigAmp));

    u8PulseStepIdx++;
    if (u8PulseStepIdx >= gNbSinusValue_c - 1)
    {
      gflag[0] = TRUE;
      // STIM_OUT_SEL_NONE;
      u8PulseStepIdx = 0;
      tVeineuxBiphasState = gVeineuxBiphaseWaitNeg_c;
    }
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr * 20);
    break;
  }

  case gVeineuxBiphaseWaitNeg_c:
  {
    STIM_OUT_SEL_NONE;
    Gpio_SetElectrostimulation(eETAPE9);

    STIM_GEN_RELOAD_NEXT_COUNT(80 * 20);
    tVeineuxBiphasState = gVeineuxBiphaseNoPulse;
    // Gpio_ClrAop()();
    gflag[0] = TRUE;
    break;
  }
  default:
  {
    break;
  }
  }
}

void Sinus2(void)
{
  static uint8_t u8PulseStepIdx = 0;
  static uint16_t i = 0;
  static bool flag = true;
  uint16_t tmp = 0;

  /** Veineux Biphasic pulse states */
  static enum {
    gSinusStatePos,
    gSinusStateNeg,
    gSinusStateWait,
    gSinusStateMax
  } tSinusState = gSinusStatePos;

  switch (tSinusState)
  {
  case gSinusStatePos:
  {
    if (flag == TRUE)
    {
      STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */

      CMD_M_SET_NO_PULSE;

      tmp = NORMAL_MODE;
      SPI_TRANSMIT_DATA(MSB(tmp));
      SPI_TRANSMIT_DATA(LSB(tmp));
      Gpio_SetAop();
      flag = false;
    }
    tmp = NORMAL_MODE | ((TablePuissanceCalculeeSinus[i][u8PulseStepIdx] & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));
    CMD_M_SET_POSITIVE_PULSE; /**< Enables Positive pulse CMD */

    u8PulseStepIdx++;
    if (u8PulseStepIdx >= gNbSinusValue_c / 2 - 1)
    {
      gflag[0] = TRUE;
      // STIM_OUT_SEL_NONE;
      // u8PulseStepIdx = 0;
      tSinusState = gSinusStateNeg;
      flag = true;
    }
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr * 20);
    break;
  }

  case gSinusStateNeg:
  {
    if (flag == TRUE)
    {
      Gpio_SetElectrostimulation(eETAPE4);
      Gpio_SetElectrostimulation(eETAPE6);
      flag = false;
    }

    tmp = NORMAL_MODE | ((TablePuissanceCalculeeSinus[i][u8PulseStepIdx] & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(tmp));
    SPI_TRANSMIT_DATA(LSB(tmp));
    u8PulseStepIdx++;
    if (u8PulseStepIdx >= gNbSinusValue_c)
    {
      gflag[0] = TRUE;
      // STIM_OUT_SEL_NONE;
      u8PulseStepIdx = 0;
      tSinusState = gSinusStateWait;
      flag = true;
    }
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr * 20);
    break;
  }
  case gSinusStateWait:
  {
    CMD_M_DISCONNECT;   /**< Disables pulse CMD */
    CMD_M_SET_NO_PULSE; /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE;  /**< Disables Pulse Output */
    STIM_GEN_RELOAD_NEXT_COUNT(50 * 20);
    tSinusState = gSinusStatePos;
    break;
  }

  default:
  {
    break;
  }
  }
}

void SinusMono(void)
{
  static uint8_t u8PulseStepIdx = 0;
  static uint16_t i = 0;
  static uint16_t DigAmp = 0;
  uint16_t tmp = 0;

  /** Veineux Biphasic pulse states */
  static enum {
    gVeineuxBiphaseInit_c = 0,
    gVeineuxBiphaseNegInit,
    gVeineuxBiphaseNoPulse,
    gVeineuxBiphasePos_c,
    gVeineuxBiphasePosIncr_c,
    gVeineuxBiphasePosDecr_c,
    gVeineuxBiphaseNeg_c,
    gVeineuxBiphaseNegNoPulse_c,
    gVeineuxBiphaseNegIncr_c,
    gVeineuxBiphaseNegDecr_c,
    gVeineuxBiphaseWait_c,
    gVeineuxBiphaseWaitNeg_c,
    gVeineuxBiphasStateMax_c
  } tVeineuxBiphasState = gVeineuxBiphaseInit_c;

  switch (tVeineuxBiphasState)
  {
  case gVeineuxBiphaseInit_c:
  {
    // STIM_OUT_SEL_NONE;
    Gpio_SetElectrostimulation(eETAPE1);

    gflag[0] = TRUE;

    STIM_GEN_RELOAD_NEXT_COUNT(iSS_TIMMING_COURT * 20);
    tVeineuxBiphasState = gVeineuxBiphaseNoPulse;
    break;
  }

  case gVeineuxBiphaseNoPulse:
  {
    CMD_M_SET_NO_PULSE;
    Gpio_SetAop();
    PETIT_DELAI_NOP;
    tmp = 400;
    DigAmp = NORMAL_MODE | ((tmp & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(DigAmp));
    SPI_TRANSMIT_DATA(LSB(DigAmp));
    STIM_GEN_RELOAD_NEXT_COUNT(750 * 20);
    tVeineuxBiphasState = gVeineuxBiphasePos_c;
    break;
  }

  case gVeineuxBiphasePos_c:
  {
    Gpio_SetElectrostimulation(eETAPE3); // Haut L1-H2 / CLR L1
    STIM_GEN_RELOAD_NEXT_COUNT(iSS_MOMENT_RELECTURE_COURANT * 20);
    tVeineuxBiphasState = gVeineuxBiphasePosIncr_c;
    break;
  }

  case gVeineuxBiphasePosIncr_c:
  {
    if (gflag[0] == TRUE)
    {
      STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */
      gflag[0] = FALSE;
    }

    tmp = (uint16_t)TablePuissanceCalculeeHemiSinusMonophasic[i][u8PulseStepIdx];
    DigAmp = NORMAL_MODE | ((tmp & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(DigAmp));
    SPI_TRANSMIT_DATA(LSB(DigAmp));

    u8PulseStepIdx++;
    if (u8PulseStepIdx >= gNbSinusValue_c - 1)
    {
      gflag[0] = TRUE;
      // STIM_OUT_SEL_NONE;
      u8PulseStepIdx = 0;
      tVeineuxBiphasState = gVeineuxBiphaseWait_c;
    }
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr * 20);
    break;
  }

  case gVeineuxBiphasePosDecr_c:
  {
    tmp = gStimGen_t.tPulse[i].digitalAmplitude; //(uint16_t) TablePuissanceCalculee[i][u8PulseStepIdx];
    DigAmp = NORMAL_MODE | ((tmp & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(DigAmp));
    SPI_TRANSMIT_DATA(LSB(DigAmp));

    u8PulseStepIdx++;
    if (u8PulseStepIdx == gNbSinusValue_c - 1)
    {
      gflag[0] = TRUE;
      // STIM_OUT_SEL_NONE;
      u8PulseStepIdx = 0;
      tVeineuxBiphasState = gVeineuxBiphaseWait_c;
    }
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr * 20);
    break;
  }

  case gVeineuxBiphaseWait_c:
  {
    STIM_OUT_SEL_NONE;
    Gpio_SetElectrostimulation(eETAPE4);
    tVeineuxBiphasState = gVeineuxBiphaseNoPulse;

    STIM_GEN_RELOAD_NEXT_COUNT(80 * 20);
    // Gpio_ClrAop()();
    gflag[0] = TRUE;
    break;
  }

  default:
  {
    break;
  }
  }
}

void SinusDiphase(void)
{
  static uint8_t u8PulseStepIdx = 0;
  static uint16_t i = 0;
  uint16_t DigAmp = 0;
  uint16_t tmp = 0;

  static enum {
    gSinusDiPos_c = 0,
    gSinusDiNeg_c,
    gSinusDiWait_c,
    gSinusDiInit_c
  } tSinusMonoState = gSinusDiInit_c;

  switch (tSinusMonoState)
  {
  case gSinusDiInit_c:
  {
    CMD_M_DISCONNECT;
    CMD_M_SET_NO_PULSE;
    gflag[0] = TRUE;
    tSinusMonoState = gSinusDiPos_c;
    STIM_GEN_RELOAD_NEXT_COUNT(1 * 20);
    break;
  }
  case gSinusDiPos_c:
  {
    if (gflag[0] == TRUE)
    {
      GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);
      // STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */
      Gpio_SetAop();
      gflag[0] = FALSE;
    }

    tmp = (uint16_t)(TablePuissanceCalculeeHemiSinusDiphasic[i][u8PulseStepIdx]);
    DigAmp = NORMAL_MODE | ((tmp & 0x0FFF) << 2);
    SPI_TRANSMIT_DATA(MSB(DigAmp));
    SPI_TRANSMIT_DATA(LSB(DigAmp));
    CMD_M_SET_POSITIVE_PULSE; /**< Enables Positive pulse CMD */

    if (u8PulseStepIdx >= gNbSinusValue_c - 1)
    {
      tSinusMonoState = gSinusDiWait_c;
      u8PulseStepIdx = 0;
    }
    u8PulseStepIdx++;
    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr * 20);
    break;
  }
  case gSinusDiWait_c:
  {
    STIM_OUT_SEL_NONE; /**< Disables Pulse Output */
    Gpio_SetElectrostimulation(eETAPE4);
    // CMD_M_SET_NO_PULSE; /**< Disables pulse CMD */
    gflag[0] = TRUE;
    // //Gpio_ClrAop()();
    u8PulseStepIdx = 0;
    tSinusMonoState = gSinusDiPos_c;
    STIM_GEN_RELOAD_NEXT_COUNT(1 * 20);
    break;
  }
  default:
  {
    break;
  }
  }
}

/************************************************************************************
 * Name :	NeuroMonophas 	*/
/**
 * @brief	.
 * @param	.
 * @return	.
 ************************************************************************************/
void NeuroMonophas(void)
{
  static uint16_t i = 0;
  uint16_t tmp = 0;

  /** Monophasic pulse states */
  static enum {
    gWidthStatePos_c = 1,  /**< Positive Current */
    gWidthStateNeg_c = 0,  /**< Negative Current */
    gWidthStateNull_c = 2, /**< Null Pulse */
    gWidthStateMax_c
  } tWidthState;

  //	p0_0 = 0;
  if (gStimGen_t.FreqDiff == 1)
  {
    tWidthState = gWidthStateNeg_c;
    tWidthState = gWidthStateNeg_c;
  }
  if (gStimGen_t.FreqDiff == 2)
  {
    tWidthState = gWidthStatePos_c;
    tWidthState = gWidthStatePos_c;
  }

  switch (tWidthState)
  {

  /** Positive Current */
  case gWidthStatePos_c:

    /** Sets levels */
    tmp = NORMAL_MODE | ((1000 & 0x0FFF) << 2);
    if (gStimGen_t.tPulse[i].digitalAmplitude == 0)
    {
      CMD_GALV_SEL_NONE;
      SPI_TRANSMIT_DATA(0x00);
      SPI_TRANSMIT_DATA(0x00);
    }
    else
    {
      SPI_TRANSMIT_DATA(MSB(tmp));
      SPI_TRANSMIT_DATA(LSB(tmp));

      /** Sets Commands */

      STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */
      CMD_GALV_SEL_POS;                                        /**< Enables positive current. */

      STIM_GEN_RELOAD_NEXT_COUNT(5000 /*STIM_GEN_COUNT_MAX*/);
    }

    break;

  /** Negative Current */
  case gWidthStateNeg_c:

    /** Sets levels */

    tmp = NORMAL_MODE | ((1000 & 0x0FFF) << 2);
    if (gStimGen_t.tPulse[i].digitalAmplitude == 0)
    {
      CMD_GALV_SEL_NONE;
      SPI_TRANSMIT_DATA(0x00);
      SPI_TRANSMIT_DATA(0x00);
    }
    else
    {
      SPI_TRANSMIT_DATA(MSB(tmp));
      SPI_TRANSMIT_DATA(LSB(tmp));

      /** Sets Commands */
      STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]); /**< Active Pulse Output */
      CMD_GALV_SEL_NEG;                                        /**< Enables positive current. */

      STIM_GEN_RELOAD_NEXT_COUNT(/*STIM_GEN_COUNT_MAX*/ 5000);
    }

    break;

  /** Null Current */
  case gWidthStateNull_c:

    CMD_GALV_SEL_NONE;

    break;

  default:
    break;
  }
}

/************************************************************************************
 * Name :  ElectrodeAdhesionDetection  */
/**
 * @brief  .
 * @param  .
 * @return .
 ************************************************************************************/
bool_t ElectrodeAdhesionDetection(StimOutId_t OutId)
{
  uint16_t tmp = 0;
  uint16_t meanAmp = 0;
  uint16_t amp = 0;
  uint16_t courant = 0;
  bool pulseDone = false;
  bool_t flag = FALSE;

//  GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN) | (1 << CMD_L2_PIN) | (1 << CMD_H1_PIN) | (1 << CMD_H2_PIN);
//  GPIO->P_SET[CMD_110V_ON_OFF_PORT].DOUT = (1 << CMD_110V_ON_OFF_PIN);
//  GPIO->P_CLR[ON_OFF_BOOSTER_PORT].DOUT = (1 << ON_OFF_BOOSTER_PIN);

  /** Biphasic pulse states */
  static enum {
    gBiPhasInit1_c = 0,   // LIO
    gBiPhasInit2_c,       // LIO
    gBiphasStatePos_c,    /**< Positive Pulse */
    gBiphasStatePos_c1,   // LIO
    gBiphasStatePalier_c, // LIO
    gBiphasStateNeg_c,    /**< Negative Pulse */
    gBiphasStateInter_c,  /**< Inter Pulse */
    gBiphasStateInter1_c,
    gBiphasStateInter2_c,
    gBiphasStateNull_c, /**< Null Pulse */
    gBiphasStateNullLoop_c,
    gBiphasStateMax_c
  } tBiphasState = gBiphasStatePos_c;

  STIM_OUT_SEL_NONE;
  // DETECT_RES_CS_DIS;
  //   p0_0 = 1; // pin test
  while (1)
  {
    amp = 0;
    for (uint8_t i = 0; i < 10; i++)
    {
      while (pulseDone != true)
      {
        switch (tBiphasState)
        {
          /** Positive Pulse */
        case gBiphasStatePos_c:
        {
          if (OutId == 0)
          {
            GPIO_PinOutClear(CS_VOIE2_PORT, CS_VOIE2_PIN); /**< Active Pulse Output on Output 2*/
            GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);   /**< Active Pulse Output on Output 1*/
          }
          else
          {
            GPIO_PinOutClear(CS_VOIE1_PORT, CS_VOIE1_PIN); /**< Active Pulse Output on Output 1*/
            GPIO_PinOutSet(CS_VOIE2_PORT, CS_VOIE2_PIN);   /**< Active Pulse Output on Output 2*/
          }
          // Application EPH
          // Gpio_SetElectrostimulation(eETAPE3); // Haut L1-H2 / CLR L1
          CMD_M_SET_NO_PULSE;

          tmp = NORMAL_MODE;
          SPI_TRANSMIT_DATA(MSB(tmp));
          SPI_TRANSMIT_DATA(LSB(tmp));
          Gpio_SetAop();

          /** Sets Level */
          tmp = NORMAL_MODE | (((courant * 100 * 4096 / 10900) & 0x0FFF) << 2);
          SPI_TRANSMIT_DATA(MSB(tmp));
          SPI_TRANSMIT_DATA(LSB(tmp));

          CMD_M_SET_POSITIVE_PULSE; /**< Enables Positive pulse CMD */
          // Configuration du timming prochaine étape
          sl_udelay_wait(50);
          // amp += IADC_Read_Current();
          tBiphasState = gBiphasStatePalier_c;
          break;
        }

        case gBiphasStatePalier_c:
        {
          amp += IADC_Read_Current();
          //  Application EPB
          Gpio_SetElectrostimulation(eETAPE4); // Reset H2-L1
          Gpio_SetElectrostimulation(eETAPE6); // BAS H1-L2
          // Configuration du timming prochaine étape
          sl_udelay_wait(50);

          tBiphasState = gBiphasStateInter1_c;
          break;
        }

        case gBiphasStateInter1_c:
        {
          STIM_OUT_SEL_NONE; /**< Disables Pulse Output */
          Gpio_SetElectrostimulation(eETAPE9);
          // CMD_M_DISCONNECT;   /**< Disables pulse CMD */
          CMD_M_SET_NO_PULSE; /**< Disables pulse CMD */

          //  Application AOP -> OFF
          // //Gpio_ClrAop()();
          sl_udelay_wait(100);
          pulseDone = true;
          tBiphasState = gBiphasStatePos_c;
          break;
        }
        default:
          /** TODO !!!!! */
          break;
        }
      }
      pulseDone = false;
    }

    meanAmp = amp / 10;
    if (meanAmp > 100)
    {
      flag = true;
      break;
    }

    if (courant < 20)
      courant += 20;
    else
    {
      flag = false;
      break;
    }
  }
  // DETECT_RES_CS_EN;
  GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN) | (1 << CMD_L2_PIN) | (1 << CMD_H1_PIN) | (1 << CMD_H2_PIN);

  Gpio_ClrAop();
//  GPIO->P_CLR[CMD_110V_ON_OFF_PORT].DOUT = (1 << CMD_110V_ON_OFF_PIN);
//  GPIO->P_SET[ON_OFF_BOOSTER_PORT].DOUT = (1 << ON_OFF_BOOSTER_PIN);
  return flag;
}

void StimGenDummyFunct(void)
{
  sl_sleeptimer_delay_millisecond(200);
}
/**********************************************************************************
End of function
***********************************************************************************/
