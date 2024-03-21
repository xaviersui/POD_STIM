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

//#define STIM_OUT_SEL_NONE GPIO->P_CLR[CS_VOIE1_PORT].DOUT = ((1 << CS_VOIE1_PIN) | (1 << CS_VOIE2_PIN)) //GPIO_PinOutClear(CS_VOIE1_PORT, CS_VOIE1_PIN);

#define N_MODULATION_MAX          4     /**< Maximum number of modulation by envelope */

/*! \def iSRP_MOMENT_APPLICATION_AOP
\brief Moment en �s d�finit pour positionner l'AOP avant le d�but de 1ere polarit� du SRP */
#define iSRP_MOMENT_APPLICATION_AOP 700U // en us

/*! Correction d'erreur du timer MFT1 */
#define uiERREUR_TIMER_MFT1                                                     10U

/*! Valeur minimale de configuration du timer MFT1 exprim� � �s*/
#define uiMIN_TIMER_MFT1                                                        20U

#define gNbExponentialValue_c     26    /**< Size of the values array defining a exponential impulse */
#define gNbExponentialPulseRepetition_c 8   /**< Number Size of the values array defining a exponential impulse */

#define STIM_GEN_TR_MIN         50          // in �s

#define STIM_GEN_COUNT_MAX        TIMER_MaxCount(TIMER0)
#define NORMAL_MODE           0x0000

sl_sleeptimer_timer_handle_t timer;

uint16_t        TablePuissanceCalculee[gStimOutMax_c][gNbExponentialValue_c];
/* Exponential current coefficient table */
static const uint8_t  gExponentialValue_percent[gNbExponentialValue_c]
//              = {25, 44, 58, 69, 77, 82, 87, 90, 92, 94, 96, 97, 97, 98, 98, 99, 100};        /* 17 step values */
              = {17,31,43,53,61,68,73,78,82,85,87,90,91,93,94,95,96,96,97,97,98,98,98,99,99,100};   /* 26 step values */
//const           uint8_t TablePuissanceVeine [26]=
//            {3,12,23,32,40,47,52,61,67,72,74,78,82,85,87,90,91,92,93,94,95,96,97,98,99,100};
uint8_t  gNPulse_c[gStimPatternMax_c]= {0};
uint16_t        gDigAmplMeas[gStimOutMax_c];

volatile        uint8_t cpt=7;
volatile        uint8_t cptFreqDiff = 0;
volatile        StimulationGeneration_t gStimGen_t;



bool_t      gflag[gStimOutMax_c]= {0};
static const uint8_t  gStimOutCmd_c[gStimOutMax_c] = { STIM_OUT_0, STIM_OUT_1 };
//static     uint8_t  gNPulse_c[gStimPatternMax_c];
//static Stimulation_t  gStim_t;    /**< */
void (*pStimGenCallback[gStimPatternMax_c])(void);

void Gpio_SetAop()
{
  GPIO_PinOutSet(CMD_AOP_PORT, CMD_AOP_PIN);
}

void Gpio_ClrAop()
{
  GPIO_PinOutClear(CMD_AOP_PORT, CMD_AOP_PIN);
}

/************************************************************************************
* Name :  StimManagementConfigPulse   *//**
* @brief  Sets the pulses frequency and pulses widths
* @param  .
* @return .
************************************************************************************/
StimGenErr_t StimManagementConfigPulse(StimulationConfiguration_t* pStimConfig_t)
{
  uint8_t   i;
  static uint32_t temp;
  uint16_t  nStimTimerloop, nPulseTimerloop, trStim, trPulse;

  /** Asserts stimulation period is greater than total pulses period */

  temp = 0;
  for(i=0;i<pStimConfig_t->nStim;i++)
  {
    temp += pStimConfig_t->tPattern[i].width * gNPulse_c[pStimConfig_t->patternId];
  }
  /** T = N * STIM_GEN_TRM_PERIOD_MAX + Tr */
  nStimTimerloop  = pStimConfig_t->frequency / STIM_GEN_TRM_PERIOD_MAX;
  trStim      = (pStimConfig_t->frequency) % STIM_GEN_TRM_PERIOD_MAX;

  nPulseTimerloop = (10 * temp) / STIM_GEN_TRM_PERIOD_MAX;
  trPulse     = (10 * temp) % STIM_GEN_TRM_PERIOD_MAX;

  if(nPulseTimerloop > nStimTimerloop)
    return gStimGenErrPulseWidth_c;

  if(nPulseTimerloop == nStimTimerloop)
    if(trPulse >= trStim)
     return gStimGenErrPulseWidth_c;

  /** Sets stimulation and pulse periods */

  /* T = TStim - TPulse = (NStimTimerloop - NPulseTimerloop) * BSP_MAX_STIM_TIMER_PERIOD + (TrStim - TrPulse) */
  nStimTimerloop -= nPulseTimerloop;

  if(trStim >= trPulse)
    trStim -= trPulse;
  else
  {
    nStimTimerloop--;
    trStim = STIM_GEN_TRM_PERIOD_MAX + trStim - trPulse;
  }

  /* if TrStim < Critical Tinterrupt, TrStim = Critical Tinterrupt */
  if(trStim < STIM_GEN_TR_MIN)
    trStim = STIM_GEN_TR_MIN;


  /** counters value */

  //STIM_MNGMNT_GET_RESOURCE; disable IRQ

  gStimGen_t.nTimerLoop = nStimTimerloop;
  gStimGen_t.cntTr = (trStim * 2);
  gStimGen_t.nPulse = pStimConfig_t->nStim;

  for(i=0;i<gStimGen_t.nPulse;i++)
  {
    gStimGen_t.tPulse[i].outId = pStimConfig_t->tPattern[i].outId;
    gStimGen_t.tPulse[i].cntWidth = (pStimConfig_t->tPattern[i].width);
  }



  gStimGen_t.stimGenPatternId = pStimConfig_t->patternId;

  gStimGen_t.FreqDiff = pStimConfig_t->FreqDiff;
  gStimGen_t.Ratio = pStimConfig_t->Ratio;

  //STIM_MNGMNT_RELEASE_RESOURCE; enable IRQ

  if(gStimGen_t.stimGenPatternId == 0x2)
  {
    if(pStimConfig_t->tPattern[0].width == 0)
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

  return gStimGenErrNoError_c;
}
/**********************************************************************************
End of function
***********************************************************************************/
int32_t Ad5691r_SetIntensiteStimulation(uint32_t v /*uint32_t ui32Intensite*/)
{
  volatile int32_t i32Ret = 0;
  volatile float fTensionToDac;
  volatile Data16bit_t DataBuffer;
  volatile uint8_t ui8Data;
  volatile uint16_t ui16Val;
  static uint16_t last = 0;

  int i;

  // On programme l'intensit� si elle est diff�rente de la pr�c�dente programm�e
  /* if (Gui32LastIntensiteProgrammed != ui32Intensite)
  {
   Gui32LastIntensiteProgrammed = ui32Intensite; */
#ifdef SIUE1316_OPTION_TRACE_INTENSITE_PROGRAMME
  if (GuiIndexIntensite < iNB_INTENSITE_PRG)
  {
    GtuiIntensiteProgrammed[GuiIndexIntensite] = ui32Intensite;
    GuiIndexIntensite++;
  }
  else
    GuiIndexIntensite = 0;
#endif
  // Conversion de l'intensit� vers une tension DAC
  if (v >= 100)
  {
      fTensionToDac = (float)v;
      //fTensionToDac = fTensionToDac /2;

      //fTensionToDac = (fTensionToDac * fGAIN_DAC) + fOFFSET_DAC; // voir avec MICROTEC FONCTION
      ui16Val = (uint16_t)fTensionToDac;
  }
  else
  {
      ui16Val = 0U;
  }
  if (ui16Val > 4095U)
  {
      ui16Val = 4095U;
  }
//  if (last != v)
//  {
    last = v;
    DataBuffer.ui16bit =(uint16_t)ui16Val;
    DataBuffer.ui16bit=DataBuffer.ui16bit<<4;
    // Inversion Msb et Lsb car les Msb doivent �tre envoy�s en premier
       ui8Data=DataBuffer.u8bit[0];
       DataBuffer.u8bit[0]=DataBuffer.u8bit[1];
       DataBuffer.u8bit[1]=ui8Data;
       i=0;
       do {
         // Ecriture du DAC register
       i32Ret = I2C_LeaderWrite(I2C_DAC_ADDR << 1, ui8WRITE_DAC_AND_INPUT_REGISTER_COMMAND_BYTE, DataBuffer.u8bit, 2);
         i++;
       } while ((i32Ret!=0)&&(i<iNB_REESSAI_COMMUNICATION));


 // }

  /*if (i32Ret != 0)
  {
      _Error_Handler(__FILE__, __LINE__);
  }*/
  //}
  return (i32Ret);
}
/**********************************************************************************
End of function
***********************************************************************************/
/************************************************************************************
* Name :  StimManagementSetDigitalAmplitude   *//**
* @brief  Sets the Digital Amplitude
* @param  amplitude
* @param  pulseId
* @return .
************************************************************************************/
StimGenErr_t StimManagementSetDigitalAmplitude(uint16_t amplitude, uint8_t pulseId)
{
  uint32_t digAmpl;
  uint8_t i=0;
  //gStimGen_t.stimGenPatternId = gStimPatternVeineuxBiphasic_c; // Force le mode veineux


  if((StimOutId_t)pulseId > gStimOutMax_c)
    return gStimGenErrSetDigAmplOutIdMax_c;

  if(amplitude > STIM_GEN_AMPLITUDE_MAX)
    return gStimGenErrSetDigAmplitudeMax_c;

  digAmpl = (amplitude*100);//DAC_VALUE_MAX) / STIM_GEN_AMPLITUDE_MAX

  //STIM_MNGMNT_GET_RESOURCE;
  gStimGen_t.tPulse[pulseId].digitalAmplitude = (uint16_t) (digAmpl* fGAIN_DAC) + fOFFSET_DAC; /** fGAIN_DAC) + fOFFSET_DAC;**/
  //STIM_MNGMNT_RELEASE_RESOURCE;

  if(gStimGen_t.stimGenPatternId == gStimPatternVeineuxBiphasic_c)
  {
//    STIM_MNGMNT_GET_RESOURCE;
    for(i=0; i<gNbExponentialValue_c; i++)
    {
      TablePuissanceCalculee[pulseId][i] = (uint16_t) ((uint32_t)gExponentialValue_percent[i] * ((amplitude*100  * fGAIN_DAC) + fOFFSET_DAC) / 2); //(fTensionToDac * fGAIN_DAC) + fOFFSET_DAC;
    }
//    STIM_MNGMNT_RELEASE_RESOURCE;
  }

  return gStimGenErrNoError_c;
}

/**********************************************************************************
End of function
***********************************************************************************/

/*!
* \fn bool Adc_TraitementAcquisitionCourantRelecture(void)
* \brief Traitement de l'acquisitions courant de relecture si fin de convertion
* \retval false si la derni�re demande d'acquisition n'est pas trait�e sinon false
*/
bool Adc_TraitementAcquisitionCourantRelecture(void)
{
bool bRet=false;

 if (IADC_Read_Current()!= 0)
 {
     bRet=true;

 }
   return(bRet);
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
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN)|(1 << CMD_L2_PIN)|(1 << CMD_H1_PIN)|(1 << CMD_H2_PIN);
    break;
  case eEPH: // Electrostimulation Polarit� Haute
  //   GPIO_ResetBits(CMD_L1_PIN|CMD_L2_PIN|CMD_H1_PIN|CMD_H2_PIN);
 //    GPIO_SetBits(CMD_L1_PIN|CMD_H2_PIN);
  //CLR_BRIDGE;
  GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN) | (1 << CMD_L1_PIN);
//  sl_udelay_wait(5);
  GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
//  sl_udelay_wait(5);
  //CLR_BRIDGE;
//  sl_udelay_wait(7);
  GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H2_PIN);
    break;
  case eEPB: // Electrostimulation Polarit� Basse
   //  GPIO_ResetBits(CMD_L1_PIN|CMD_L2_PIN|CMD_H1_PIN|CMD_H2_PIN);
   //  GPIO_SetBits(CMD_L2_PIN|CMD_H1_PIN);
  //  CLR_BRIDGE;
    //sl_udelay_wait(5);
  //  CLR_BRIDGE;
  //  GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN) | (1 << CMD_L1_PIN);
 //   sl_udelay_wait(5);
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN);
  //  sl_udelay_wait(5);
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
   /* GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN);
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H2_PIN); */
     GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN);
     GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
     GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H2_PIN);
    break;
  case eETAPE4: // Etape #4 issue du document de conception hardware "Reset  H1-L2
    // GPIO_ResetBits(CDE_MOS_H2_PIN);
    // GPIO_ResetBits(CDE_MOS_L1_PIN);
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_H2_PIN);
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN);
    break;
  case eETAPE6: // Etape #6 issue du document de conception hardware
 /*   GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H1_PIN); */
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H1_PIN);
    break;
  case eETAPE7: // Etape #7 issue du document de conception hardware
    CLR_BRIDGE;
    break;
  case  eETAPE8: // Etape #8 créer pour carte stimbio V2
     GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
     GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN);
     GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H1_PIN);
    break;
  case eETAPE9:
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_H1_PIN);
    GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L2_PIN);
    break;
  case eETAPE10:
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN);
    GPIO->P_SET[CMD_H1_PORT].DOUT = (1 << CMD_H2_PIN);
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
void Timer_SetMft1Timming(uint16_t ui16Time)
{
  /*! Variable indiquant si le timer doit �tre arr�t pour ne faire qu'une interruption */
  static volatile uint16_t Gui16CounterMft1Timer;

  if (ui16Time >= uiMIN_TIMER_MFT1)
    {
    Gui16CounterMft1Timer=ui16Time-uiERREUR_TIMER_MFT1;
    }
  else {
    Gui16CounterMft1Timer=2U;
  }
  /* D�marrage du timer */
  set_timer0_time(Gui16CounterMft1Timer)  ;

}

//-----------------------------------------------------------------------------
/*!
 * \fn void MeSS_GestionCourantBiphasiquePositif(void)
 * \brief Fonction de gestion des courant Biphasique
 */
void MeSS_GestionCourantBiphasiquePositif(void)
{
  static uint8_t state = 0;
 // static uint16_t v = 0; //(400 * 4095) / 3300;
  static uint32_t IntensiteProgramme = 15000;
  static uint8_t first = 0;
  static uint16_t ui16TimeApplicationAop = 500;

  GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);

  if (first < 20)
  {
       switch (state)
    {
    case 0:
      // Configuration du timming prochaine étape
      Timer_SetMft1Timming(iSS_TIMMING_COURT);
      //Application "RAZ des commandes du pont" ordre1
      Gpio_SetElectrostimulation(eETAPE1); //RAZ
       break;
    case 1:
     // Configuration du timming prochaine étape
      Timer_SetMft1Timming(ui16TimeApplicationAop);
      // Application Mise à la masse du pont
      Gpio_SetElectrostimulation(eETAPE2); //L1-L2
      //  Application AOP -> ON
       Gpio_SetAop();
     // Petit d�lai par des cycles horloge
     PETIT_DELAI_NOP;
     //Mise en forme Intensité programmé
     (void)Ad5691r_SetIntensiteStimulation(IntensiteProgramme); //exprimer en uV
      break;
      //--------------------------------------------------------------------------
      // *********                   Polarit� -> HAUTE                 ***********
      //--------------------------------------------------------------------------
    case 2: // Etape de rebouclage de g�n�ration du stimuli par la polarit� -> HAUTE
      // Configuration du timming prochaine étape
      Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
      // Application EPH
      Gpio_SetElectrostimulation(eETAPE3); // Haut L1-H2 / CLR L1
          break;
    case 3: // Bas

      // Configuration du timming prochaine étape
      Timer_SetMft1Timming(iSS_DUREE_IMPULSION);/// Variable qui donne le temps de la pulsation Delta "t"
      // R�cup�ration de la mesure de courant de relecture �ventuelle
      (void)Adc_TraitementAcquisitionCourantRelecture();
              break;
   //--------------------------------------------------------------------------
   // *********                   Polarit� -> BASSE                 ***********
   //--------------------------------------------------------------------------
       case 4: //Etape de changement de polarit� -> BASSE
         // Application EPB
            Gpio_SetElectrostimulation(eETAPE4); // Reset H1-L2
            Gpio_SetElectrostimulation(eETAPE6); //BAS H2-L1
            // Configuration du timming prochaine étape
                Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiMIN_TIMER_MFT1);
                break;
    case 5: // Bas
        // Configuration du timming prochaine étape
          Timer_SetMft1Timming(iSS_DUREE_IMPULSION-uiMIN_TIMER_MFT1);/// Variable qui donne le temps de la pulsation Delta "t"
        // R�cup�ration de la mesure de courant de relecture �ventuelle
              (void)Adc_TraitementAcquisitionCourantRelecture();
              break;
    //--------------------------------------------------------------------------
    // *********                    BASE = PALIER                    ***********
    //--------------------------------------------------------------------------
    case 6: // Bas
      // Configuration du timming prochaine étape
      Timer_SetMft1Timming((iSS_DUREE_TPS_BASE-1733)/2);/// Variable  rapport périodique entre chaque impulsion
         //Application "RAZ des commandes du pont" ordre7
          Gpio_SetElectrostimulation(eETAPE7); //Reset
          (void)Ad5691r_SetIntensiteStimulation(600); // à enlever
          // Application AOP -> OFF
          Gpio_ClrAop();
              break;
    case 7: // OFF
      // Configuration du timming prochaine étape
        Timer_SetMft1Timming(iSS_DUREE_TPS_BASE/2);/// Variable rapport périodique entre chaque impulsion
      state = 0;
      first++;
      break;
    default:
      break;
    }
  }

  // Configuration prochaine �tape
  state++;
}

/************************************************************************************
* Name :  ImpulsBiphas  *//**
* @brief  Biphas Impuls
* @param  .
* @return .
************************************************************************************/
void ImpulsBiphas(void)
{
  static uint16_t i = 0, Nloop = 0;
  uint16_t tmp = 0;
  //uint8_t j,k;
//GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);

  /** Biphasic pulse states */
  static enum
  {
    gBiPhasInit1_c = 0,       //LIO
    gBiPhasInit2_c,           //LIO
    gBiphasStatePos_c ,     /**< Positive Pulse */
    gBiphasStatePos_c1,       //LIO
    gBiphasStatePalier_c,     //LIO
    gBiphasStateNeg_c,        /**< Negative Pulse */
    gBiphasStateInter_c,      /**< Inter Pulse */
    gBiphasStateInter1_c,
    gBiphasStateInter2_c,
    gBiphasStateNull_c,       /**< Null Pulse */
    gBiphasStateNullLoop_c,
    gBiphasStateMax_c
  } tBiphasState = gBiphasStatePos_c;

//  p0_0 = 1; // pin test

  switch(tBiphasState)
  {

     case gBiPhasInit1_c:
        // Configuration du timming prochaine étape
        STIM_GEN_RELOAD_NEXT_COUNT(iSS_TIMMING_COURT);
        /**< Desactive CMD */
        CMD_M_DISCONNECT;
        tBiphasState = gBiPhasInit2_c;
        break;

    case gBiPhasInit2_c:


        // Configuration du timming prochaine étape
        STIM_GEN_RELOAD_NEXT_COUNT(500);
        (void)Ad5691r_SetIntensiteStimulation(0*fGAIN_DAC+fOFFSET_DAC);
       /** Sets Next Step Time */
        CMD_M_SET_NO_PULSE;    //L1-L2
        //  Application AOP -> ON
        Gpio_SetAop();
        // Petit d�lai par des cycles horloge
         PETIT_DELAI_NOP;
         /** Sets Level */
        (void)Ad5691r_SetIntensiteStimulation(gStimGen_t.tPulse[i].digitalAmplitude); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
        tBiphasState = gBiphasStatePos_c;
        break;

        /** Positive Pulse */
    case gBiphasStatePos_c  :

         Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
        /** Sets Commands */
        STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);//   a modif    /**< Active Pulse Output */
        /**< Enables Positive pulse CMD */
        CMD_M_SET_POSITIVE_PULSE;
        //  SWITCH_START;             /**< Enables switching */
        tBiphasState = gBiphasStatePos_c1;
        break;

     case gBiphasStatePos_c1  :

         //CMD_M_DISCONNECT;       /**< Desactive CMD */
         /** Sets Next Step Time */
         STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth); /// DUREE DE L'IMPULSION
         /** Current Measurement */
         gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
         gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
         tBiphasState = gBiphasStatePalier_c;
         break;


      case gBiphasStatePalier_c  :

         //CMD_M_DISCONNECT;       /**< Desactive CMD */
         /**< Disable  pulse CMD */
         Gpio_SetElectrostimulation(eETAPE4);
         /**Negatif Pulse*/
         Gpio_SetElectrostimulation(eETAPE6);
         Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiMIN_TIMER_MFT1);
         tBiphasState = gBiphasStateNeg_c;
         break;


          /** Negative Pulse */
      case gBiphasStateNeg_c  :
         /** Sets Next Step Time */
         STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth); /// DUREE DE L'IMPULSION
         /** Current Measurement */
         gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
         gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
         gflag[i]=TRUE;
         //  CMD_M_SET_NO_PULSE;             /**< Disables pulse CMD */
         /** Sets Commands */
         //  CMD_M_SET_NEGATIVE_PULSE;       /**< Enables Negative pulse CMD */

         tBiphasState = gBiphasStateInter1_c;
         break;

      case gBiphasStateInter1_c:
        CMD_M_DISCONNECT;
     //   STIM_OUT_CLR(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);//   a modif    /**< Active Pulse Output */
        Gpio_ClrAop();

        /** Sets Next Step Time */
        switch(gStimGen_t.FreqDiff)
                           {
        //                   case 1 :
        //
        //                       if(cptFreqDiff<gStimGen_t.Ratio)
        //                           {
        //                             cptFreqDiff++;
        //                               if(i==1)
        //                                   STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr-(11172+2*gStimGen_t.tPulse[i].cntWidth));
        //                                 else
        //                                     STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        //                                 i = 0;
        //                                   tBiphasState = gBiphasStateNull_c;
        //                             }
        //                         else
        //                             {
        //                               cptFreqDiff=0;
        //                                 if(++i < gStimGen_t.nPulse)       /**< Next Pulse */
        //                                     {
        //                                       STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/11172);   //!!!!!
        //                                         tBiphasState = /*gBiphasStatePos_c*/gBiphasStateInter_c;
        //                                       }
        //                                   else                  /**< No Pulse */
        //                                       {
        //                                         STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        //                                           i = 0;
        //                                             tBiphasState = gBiphasStateNull_c;
        //                                         }
        //                               }
        //                      break;
        //
          case 0 :

            if(++i < gStimGen_t.nPulse)       /**< Next Pulse */
              {
                STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/2172);   //!!!!!
               STIM_OUT_CLR(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);//   a modif    /**< desActive Pulse Output */

                tBiphasState = /*gBiphasStatePos_c*/gBiPhasInit1_c;

              }
            else                  /**< No Pulse */
              {
                STIM_GEN_RELOAD_NEXT_COUNT(20000);
                i = 0;
                STIM_OUT_CLR(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);//   a modif    /**< desa Pulse Output */

                tBiphasState = gBiPhasInit1_c;
              }
            break;

        //                     case -1 :
        //
        //                         if(i==0)
        //                             {
        //                               i=1;
        //                                 cptFreqDiff++;
        //                                   STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/11172);   //!!!!!
        //                                     tBiphasState = /*gBiphasStatePos_c*/gBiphasStateInter_c;
        //                               }
        //                           else
        //                               {
        //                                 if(cptFreqDiff < gStimGen_t.Ratio)
        //                                     {
        //                                       cptFreqDiff++;
        //                                         i=1;
        //                                           STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
        //                                             tBiphasState = gBiphasStateNull_c;
        //                                       }
        //                                   else
        //                                       {
        //                                         cptFreqDiff = 0;
        //                                           STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr-(11172+2*gStimGen_t.tPulse[i].cntWidth));
        //                                           i = 0;
        //                                             tBiphasState = gBiphasStateNull_c;
        //                                         }
        //                                 }
        //
        //                      break;
            };
        break;

    /** Inter Pulse */
    case gBiphasStateInter_c  :

        /** Sets Commands */
        SWITCH_STOP;              /**< Disables switching */
       //  CMD_DEMAG_DIS;              /**< Disables Pulse */
       CMD_M_DISCONNECT;             /**< Disables pulse CMD */
       STIM_OUT_SEL_NONE;            /**< Disables Pulse Output */
       /** Sets Level */
       tmp = NORMAL_MODE;
       (void)Ad5691r_SetIntensiteStimulation(tmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
       //  Application AOP -> OFF
       Gpio_ClrAop();
      /** Sets pause cmd */
      // CMD_M_SET_NO_PULSE;
      /** Sets Next Step Time */
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth+229);
      tBiphasState = gBiphasStatePos_c;
      break;

    /** Null Pulse */
    case gBiphasStateNull_c :

      /** Sets Commands */
      SWITCH_STOP;              /**< Disables switching */
      CMD_M_DISCONNECT;             /**< Disables pulse CMD */
      STIM_OUT_SEL_NONE;            /**< Disables Pulse Output */
      /** Sets Level */
      tmp = NORMAL_MODE;
      (void)Ad5691r_SetIntensiteStimulation(tmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
      //  Application AOP -> OFF
      Gpio_ClrAop();
      /** Sets pause cmd */
      //    CMD_M_SET_NO_PULSE;
      /** Sets Next Step Time */
      if (gStimGen_t.nTimerLoop > 0)      /**< Next Loop */
      {
        Nloop = gStimGen_t.nTimerLoop;
        STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
        tBiphasState = gBiphasStateNullLoop_c;
      }
      else                  /**< First Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth+229);
        tBiphasState = gBiPhasInit1_c;
      }
      break;

    case gBiphasStateNullLoop_c :

      Nloop--;
      if(Nloop > 0)             /**< Next Loop */
        STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
      else                  /**< First Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth+229);
        tBiphasState = gBiPhasInit1_c;
      }
      break;

    default :
    /** TODO !!!!! */
      break;

  }


}
/**********************************************************************************
End of function
***********************************************************************************/



//-----------------------------------------------------------------------------
/*!
 * \fn void MeSS_GestionCourantBiphasiqueNegatif(void)
 * \brief Fonction de gestion des courant Biphasique - LIO
 */
void MeSS_GestionCourantBiphasiqueNegatif(void)
{
  static uint8_t state = 0;
 // static uint16_t v = 0; //(400 * 4095) / 3300;
  static uint32_t IntensiteProgramme = 15000;
  static uint8_t first = 0;
  static uint16_t ui16TimeApplicationAop = 500;

  GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);
  if (first < 20)
   {
        switch (state)
     {
     case 0:
       // Configuration du timming prochaine étape
       Timer_SetMft1Timming(iSS_TIMMING_COURT);
       //Application "RAZ des commandes du pont" ordre1
       Gpio_SetElectrostimulation(eETAPE1); //RAZ
        break;
     case 1:
      // Configuration du timming prochaine étape
       Timer_SetMft1Timming(ui16TimeApplicationAop);
       // Application Mise à la masse du pont
       Gpio_SetElectrostimulation(eETAPE2); //L1-L2
       //  Application AOP -> ON
        Gpio_SetAop();
      // Petit d�lai par des cycles horloge
      PETIT_DELAI_NOP;
      //Mise en forme Intensité programmé
      (void)Ad5691r_SetIntensiteStimulation(IntensiteProgramme); //exprimer en uV
       break;
       //--------------------------------------------------------------------------
       // *********                   Polarit� -> BASSE                 ***********
       //--------------------------------------------------------------------------
     case 2: // Etape de rebouclage de g�n�ration du stimuli par la polarit� -> BASSE
       // Configuration du timming prochaine étape
       Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
       // Application EPH
       Gpio_SetElectrostimulation(eETAPE8); // BAS L1-H2
           break;
     case 3: // Bas

       // Configuration du timming prochaine étape
       Timer_SetMft1Timming(iSS_DUREE_IMPULSION);/// Variable qui donne le temps de la pulsation Delta "t"
       // R�cup�ration de la mesure de courant de relecture �ventuelle
       (void)Adc_TraitementAcquisitionCourantRelecture();
               break;
    //--------------------------------------------------------------------------
    // *********                   Polarit� -> HAUTE                 ***********
    //--------------------------------------------------------------------------
        case 4: //Etape de changement de polarit� -> HAUTE
          // Application EPB

             Gpio_SetElectrostimulation(eETAPE9); // Reset H2-L1
             Gpio_SetElectrostimulation(eETAPE10); //HAUT H1-L2
             // Configuration du timming prochaine étape
                 Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiMIN_TIMER_MFT1);
                 break;
     case 5: // HAUT
         // Configuration du timming prochaine étape
           Timer_SetMft1Timming(iSS_DUREE_IMPULSION-uiMIN_TIMER_MFT1);/// Variable qui donne le temps de la pulsation Delta "t"
         // R�cup�ration de la mesure de courant de relecture �ventuelle
               (void)Adc_TraitementAcquisitionCourantRelecture();
               break;
     //--------------------------------------------------------------------------
     // *********                    BASE = PALIER                    ***********
     //--------------------------------------------------------------------------
     case 6: // Bas
       // Configuration du timming prochaine étape
       Timer_SetMft1Timming((iSS_DUREE_TPS_BASE-1733)/2);/// Variable  rapport périodique entre chaque impulsion
          //Application "RAZ des commandes du pont" ordre7
           Gpio_SetElectrostimulation(eETAPE7); //Reset
           (void)Ad5691r_SetIntensiteStimulation(600); // à enlever
           // Application AOP -> OFF
           Gpio_ClrAop();
               break;
     case 7: // OFF
       // Configuration du timming prochaine étape
         Timer_SetMft1Timming(iSS_DUREE_TPS_BASE/2);/// Variable rapport périodique entre chaque impulsion
       state = 0;
       first++;
       break;
     default:
       break;
     }
   }

   // Configuration prochaine �tape
   state++;

}

//-----------------------------------------------------------------------------

/************************************************************************************
* Name :  ImpulsMonophas  *//**
* @brief  Monophas Impuls
* @param  .
* @return .
************************************************************************************/
void ImpulsMonophas(void)
{
  static uint16_t i = 0, Nloop = 0;
  uint16_t tmp = 0;
  //uint8_t lsb,msb;
//GPIO_PinOutSet(CMD_110V_ON_OFF_PORT, CMD_110V_ON_OFF_PIN);
  GPIO_PinOutSet(CS_VOIE2_PORT, CS_VOIE2_PIN);
  /** Monophasic pulse states */
  static enum
  {
    gMonoPhasInit1_c = 0,
    gMonoPhasInit2_c,
    gMonophasStatePos_c,      /**< Positive Pulse */
    gMonophasStatePos_c1,
    gMonophasStatePalier_c,
    gMonophasStatePalier1_c,
    gMonophasStateNeg_c,
    gMonophasStateInter_c,      /**< Inter Pulse */
    gMonophasStateInter1_c,      /**< Inter Pulse */
    gMonophasStateNull_c,       /**< Null Pulse */
    gMonophasStateNullLoop_c,
    gMonophasStateMax_c
  } tMonophasState = gMonoPhasInit1_c;

  switch(tMonophasState)
  {
    case gMonoPhasInit1_c:

        // Configuration du timming prochaine étape
        STIM_GEN_RELOAD_NEXT_COUNT(iSS_TIMMING_COURT);
        /**< Desactive CMD */
        CMD_M_DISCONNECT;
        tMonophasState = gMonoPhasInit2_c;
        break;

    case gMonoPhasInit2_c:
        // Configuration du timming prochaine étape
        STIM_GEN_RELOAD_NEXT_COUNT(500);
       /** Sets Next Step Time */
        CMD_M_SET_NO_PULSE;    //L1-L2
        //  Application AOP -> ON
        Gpio_SetAop();
        // Petit d�lai par des cycles horloge
        PETIT_DELAI_NOP;
        /** Sets Level */
        (void)Ad5691r_SetIntensiteStimulation(gStimGen_t.tPulse[i].digitalAmplitude);
        tMonophasState = gMonophasStatePos_c;
        break;

    case gMonophasStatePos_c  :
        Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
        /** Sets Commands */
        //STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);

        CMD_M_SET_POSITIVE_PULSE;       /**< Enables Positive pulse CMD */
       // SWITCH_START;             /**< Enables switching */
        tMonophasState = gMonophasStatePos_c1;
        break;

    case gMonophasStatePos_c1  :
         /** Sets Next Step Time */
         STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth); /// DUREE DE L'IMPULSION
         gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
         gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
         gflag[i]=TRUE;
         tMonophasState = gMonophasStatePalier_c;
         break;

    case gMonophasStatePalier_c  :
        Gpio_SetElectrostimulation(eETAPE4);
        if(++i < gStimGen_t.nPulse)      /**< Next Pulse */
          {
            /** Sets Next Step Time */
            Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
            tMonophasState =gMonophasStateInter_c;
          }
        else                  /**< No Pulse */
          {
            Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
            i = 0;
            tMonophasState = gMonophasStatePalier1_c;
          }
         break;

    case gMonophasStateInter_c:
      STIM_GEN_RELOAD_NEXT_COUNT(2172);    // Temps mort inter pulse
      CMD_M_DISCONNECT;             /**< Disables pulse CMD */
      Gpio_ClrAop();
      tMonophasState = gMonoPhasInit1_c;
      break;

    case gMonophasStatePalier1_c:
      CMD_M_DISCONNECT;             /**< Disables pulse CMD */
      Gpio_ClrAop();
      Timer_SetMft1Timming(gStimGen_t.cntTr);
      tMonophasState = gMonoPhasInit1_c;
      break;

    case gMonophasStateNullLoop_c :

      Nloop--;
      if(Nloop > 0)             /**< Next Loop */
        STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
      else                  /**< First Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
        tMonophasState = gMonoPhasInit2_c;//gMonophasStatePos_c;
      }

      break;

    default :
    /** TODO !!!!! */
      break;

  }
}
/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
* Name :  ImpulsBiphasAlterné  *//**
* @brief  Biphas Impuls
* @param  .
* @return .
************************************************************************************/

void ImpulsBiphasAltern(void)
{
  static uint16_t i = 0, Nloop = 0, flag=TRUE;
  uint16_t tmp = 0;
  //uint8_t lsb,msb,k;


  /** Biphasic pulse states */
  static enum
  {
    gBiPhasInit1_c = 0,       //LIO
    gBiPhasInit2_c,           //LIO
    gBiphasStatePos_c ,     /**< Positive Pulse */
    gBiphasStatePos_c1,       //LIO
    gBiphasStatePalier_c,     //LIO
    gBiphasStateNeg_c,        /**< Negative Pulse */
    gBiphasStateNeg_c1,        /**< Negative Pulse */
    gBiphasStateInter_c,      /**< Inter Pulse */
    gBiphasStateNull_c,       /**< Null Pulse */
    gBiphasStateNullLoop_c,
    gBiphasStateMax_c
  } tBiphasState = gBiphasStatePos_c;

//  p0_0 = 1; // pin test

  switch(tBiphasState)
  {
    case gBiPhasInit1_c:
        // Configuration du timming prochaine étape
        STIM_GEN_RELOAD_NEXT_COUNT(iSS_TIMMING_COURT);
        CMD_M_DISCONNECT;       /**< Desactive CMD */
        tBiphasState = gBiPhasInit2_c;
        break;

    case gBiPhasInit2_c:
        // Configuration du timming prochaine étape
        STIM_GEN_RELOAD_NEXT_COUNT(500);
        /** Sets Next Step Time */
        CMD_M_SET_NO_PULSE;    //L1-L2
        //  Application AOP -> ON
        Gpio_SetAop();
        // Petit d�lai par des cycles horloge
        PETIT_DELAI_NOP;
        /** Sets Level */
        (void)Ad5691r_SetIntensiteStimulation(gStimGen_t.tPulse[i].digitalAmplitude); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
        tBiphasState = gBiphasStatePos_c1;
        break;

    case gBiphasStatePos_c1:
      Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiMIN_TIMER_MFT1);

      if(flag)
        CMD_M_SET_POSITIVE_PULSE;
      else
        CMD_M_SET_NEGATIVE_PULSE;

      /** Sets Commands */
      STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);
      tBiphasState = gBiphasStatePos_c;
      break;

     case gBiphasStatePos_c  :
       /** Sets Next Step Time */
       STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth); /// DUREE DE L'IMPULSION

       /** Current Measurement */
       gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
       gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
       tBiphasState = gBiphasStateNeg_c;
       break;

        /** Negative Pulse */
       case gBiphasStateNeg_c  :
         if(flag)
           {
             Gpio_SetElectrostimulation(eETAPE4);
             Gpio_SetElectrostimulation(eETAPE6);
           }
         else
           {
             Gpio_SetElectrostimulation(eETAPE9);
             Gpio_SetElectrostimulation(eETAPE10);
           }

         Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiMIN_TIMER_MFT1);
         tBiphasState = gBiphasStateNeg_c1;
         break;

       case gBiphasStateNeg_c1  :
         /** Sets Next Step Time */
         STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth); // DUREE DE L'IMPULSION
         /** Current Measurement */
         gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
         gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
         gflag[i]=TRUE;
         tBiphasState = gBiphasStatePalier_c;
         break;

     case gBiphasStatePalier_c  :
       CMD_M_DISCONNECT;
       Gpio_ClrAop();
       /** Sets Next Step Time */
       switch(gStimGen_t.FreqDiff)
       {
//                       case 1 :
//
//                         if(cptFreqDiff<gStimGen_t.Ratio)
//                         {
//                           cptFreqDiff++;
//                           if(i==1)
//                             STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr-(11172+2*gStimGen_t.tPulse[i].cntWidth));
//                           else
//                             STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
//                           i = 0;
//                           tBiphasState = gBiphasStateNull_c;
//                         }
//                         else
//                         {
//                           cptFreqDiff=0;
//                           if(++i < gStimGen_t.nPulse)       /**< Next Pulse */
//                           {
//                             STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/11172);   //!!!!!
//                             tBiphasState = /*gBiphasStatePos_c*/gBiphasStateInter_c;
//                           }
//                           else                  /**< No Pulse */
//                           {
//                             STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
//                             i = 0;
//                             tBiphasState = gBiphasStateNull_c;
//                           }
//                         }
//                         break;

                        case 0 :

                         if(++i < gStimGen_t.nPulse)       /**< Next Pulse */
                         {
                           STIM_GEN_RELOAD_NEXT_COUNT(11172);   //!!!!!
                           tBiphasState = gBiPhasInit1_c;
                         }
                         else                  /**< No Pulse */
                         {
                           STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
                           i = 0;
                           flag = !flag;
                           tBiphasState = gBiPhasInit1_c;
                         }
                         break;

//                       case -1 :
//
//                         if(i==0)
//                         {
//                           i=1;
//                           cptFreqDiff++;
//                           STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/11172);   //!!!!!
//                           tBiphasState = /*gBiphasStatePos_c*/gBiphasStateInter_c;
//                         }
//                         else
//                         {
//                           if(cptFreqDiff < gStimGen_t.Ratio)
//                           {
//                             cptFreqDiff++;
//                             i=1;
//                             STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
//                             tBiphasState = gBiphasStateNull_c;
//                           }
//                           else
//                           {
//                             cptFreqDiff = 0;
//                             STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr-(11172+2*gStimGen_t.tPulse[i].cntWidth));
//                             i = 0;
//                             tBiphasState = gBiphasStateNull_c;
//                           }
//                         }
//
//                         break;
                     }
                     break;

    /** Inter Pulse */
    case gBiphasStateInter_c  :
        /** Sets Commands */
        CMD_M_DISCONNECT;             /**< Disables pulse CMD */
        STIM_OUT_SEL_NONE;            /**< Disables Pulse Output */
        /** Sets Level */
        tmp = NORMAL_MODE;
        // Application AOP -> OFF
        Gpio_ClrAop();
       (void)Ad5691r_SetIntensiteStimulation(tmp); //exprimer en uV
       /** Sets Next Step Time */
       STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth+1200);
       tBiphasState = gBiPhasInit1_c;
       break;

    default :
    /** TODO !!!!! */
      break;
  }


}
/**********************************************************************************
End of function
***********************************************************************************/


/************************************************************************************
* Name :  ImpulsBiphasNeg   *//**
* @brief  Biphas Negative Impuls
* @param  .
* @return .
************************************************************************************/

void ImpulsBiphasNeg(void)
{
  static uint16_t i = 0, Nloop = 0;
  uint16_t tmp = 0;



  /** Biphasic pulse states */
  static enum
  {
    gBiPhasInit1_c = 0,       //LIO
    gBiPhasInit2_c,           //LIO
    gBiphasStatePos_c,      /**< Positive Pulse */
    gBiphasStatePos_c1,       //LIO
    gBiphasStatePalier_c,     //LIO
    gBiphasStateNeg_c,        /**< Negative Pulse */
    gBiphasStateInter_c,      /**< Inter Pulse */
    gBiphasStateNull_c,       /**< Null Pulse */
    gBiphasStateNullLoop_c,
    gBiphasStateMax_c
  } tBiphasState = gBiphasStatePos_c;

//  p0_0 = 1; // pin test

  switch(tBiphasState)
  {

        case gBiPhasInit1_c:
        // Configuration du timming prochaine étape
        STIM_GEN_RELOAD_NEXT_COUNT(iSS_TIMMING_COURT);
        CMD_M_DISCONNECT;       /**< Desactive CMD */
        tBiphasState = gBiPhasInit2_c;
        break;

        case gBiPhasInit2_c:
              // Configuration du timming prochaine étape
        STIM_GEN_RELOAD_NEXT_COUNT(500);
       /** Sets Next Step Time */
        CMD_M_SET_NO_PULSE;    //L1-L2
        //  Application AOP -> ON
        Gpio_SetAop();
        // Petit d�lai par des cycles horloge
         PETIT_DELAI_NOP;
         /** Sets Level */
        (void)Ad5691r_SetIntensiteStimulation(gStimGen_t.tPulse[i].digitalAmplitude); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
        tBiphasState = gBiphasStatePos_c;
        break;



    /** Positive Pulse */
        case gBiphasStatePos_c  :

            Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
           /** Sets Commands */
            STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);//   a modif    /**< Active Pulse Output */

            CMD_M_SET_NEGATIVE_PULSE;       /**< Enables Positive pulse CMD */

            /** Sets Next Step Time */
            tBiphasState = gBiphasStatePos_c1;
         break;

        case gBiphasStatePos_c1  :
             /** Sets Next Step Time */
            STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth); /// DUREE DE L'IMPULSION
            gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
            gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
            tBiphasState = gBiphasStatePalier_c;
         break;


         case gBiphasStatePalier_c  :
           /**< Disable Negatif pulse CMD */
           Gpio_SetElectrostimulation(eETAPE9);
           /**Enable Pulse positif*/
           Gpio_SetElectrostimulation(eETAPE10);

           Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiMIN_TIMER_MFT1);
           tBiphasState = gBiphasStateNeg_c;

         break;

    /** Negative Pulse */
    case gBiphasStateNeg_c  :
      /** Sets Next Step Time */
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth); /// DUREE DE L'IMPULSIO
      gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current();
      gDigAmplMeas[i] = gStimGen_t.tPulse[i].digitalMeasAmplitude;
      tBiphasState = gBiphasStateInter_c;
      break;

    /** Inter Pulse */
    case gBiphasStateInter_c  :
      CMD_M_DISCONNECT;             /**< Disables pulse CMD */
      Gpio_ClrAop();
            /** Sets Next Step Time */
            switch(gStimGen_t.FreqDiff)
            {
//              case 1 :
//
//                if(cptFreqDiff<gStimGen_t.Ratio)
//                {
//                  cptFreqDiff++;
//                  if(i==1)
//                    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr-(11172+2*gStimGen_t.tPulse[i].cntWidth));
//                  else
//                    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
//                  i = 0;
//                  tBiphasState = gBiphasStateNull_c;
//                }
//                else
//                {
//                  cptFreqDiff=0;
//                  if(++i < gStimGen_t.nPulse)       /**< Next Pulse */
//                  {
//                    STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/11172);   //!!!!!
//                    tBiphasState = /*gBiphasStatePos_c*/gBiphasStateInter_c;
//                  }
//                  else                  /**< No Pulse */
//                  {
//                    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
//                    i = 0;
//                    tBiphasState = gBiphasStateNull_c;
//                  }
//                }
//                break;

              case 0 :

                if(++i < gStimGen_t.nPulse)       /**< Next Pulse */
                {
                  STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/11172);   //!!!!!
                  tBiphasState = /*gBiphasStatePos_c*/gBiPhasInit1_c;
                }
                else                  /**< No Pulse */
                {
                  STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
                  i = 0;
                  tBiphasState = gBiPhasInit1_c;
                }
                break;

//              case -1 :
//
//                if(i==0)
//                {
//                  i=1;
//                  cptFreqDiff++;
//                  STIM_GEN_RELOAD_NEXT_COUNT(/*gStimGen_t.tPulse[i].cntWidth*/11172);   //!!!!!
//                  tBiphasState = /*gBiphasStatePos_c*/gBiphasStateInter_c;
//                }
//                else
//                {
//                  if(cptFreqDiff < gStimGen_t.Ratio)
//                  {
//                    cptFreqDiff++;
//                    i=1;
//                    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
//                    tBiphasState = gBiphasStateNull_c;
//                  }
//                  else
//                  {
//                    cptFreqDiff = 0;
//                    STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr-(11172+2*gStimGen_t.tPulse[i].cntWidth));
//                    i = 0;
//                    tBiphasState = gBiphasStateNull_c;
//                  }
//                }
//
//                break;
            }
      break;


    /** Null Pulse */
    case gBiphasStateNull_c :

      /** Sets Commands */
      //CMD_DEMAG_DIS;              /**< Disables Pulse */
      //SWITCH_STOP;              /**< Disables switching */
      CMD_M_DISCONNECT;             /**< Disables pulse CMD */
      STIM_OUT_SEL_NONE;            /**< Disables Pulse Output */

      /** Sets Level */
      tmp = NORMAL_MODE;
             // Application AOP -> OFF
            Gpio_ClrAop();
            (void)Ad5691r_SetIntensiteStimulation(tmp); //exprimer en uV
      /** Sets pause cmd */
   //   CMD_M_SET_NO_PULSE;

      /** Sets Next Step Time */
      if (gStimGen_t.nTimerLoop > 0)      /**< Next Loop */
      {
        Nloop = gStimGen_t.nTimerLoop;
        STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
        tBiphasState = gBiphasStateNullLoop_c;
      }
      else                  /**< First Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
        tBiphasState = gBiPhasInit1_c;
      }

      break;

    case gBiphasStateNullLoop_c :

      Nloop--;
      if(Nloop > 0)             /**< Next Loop */
        STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
      else                  /**< First Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
        tBiphasState = gBiPhasInit1_c;
      }

      break;

    default :
    /** TODO !!!!! */
      break;

  }

//  p0_0 = 0; // pin test

}
/**********************************************************************************
End of function
***********************************************************************************/


///*!
// * \fn void MeSS_GestionCourantBiphasique(void)
// * \brief Fonction de gestion des courant Monophasique + LIO
// */
//void MeSS_GestionCourantMonophasiquePositif()
//{
//  static uint8_t state = 0;
//  static uint32_t IntensiteProgramme =  0;
//  static uint8_t first = 0;
//  static uint16_t ui16TimeApplicationAop = 500;
//
//
//  IntensiteProgramme  = (gStimGen_t.tPulse[0].digitalAmplitude);
//  GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);
//
//  if (first < 20)
//  {
//       switch (state)
//    {
//    case 0:
//      // Configuration du timming prochaine étape
//      Timer_SetMft1Timming(iSS_TIMMING_COURT);
//      //Application "RAZ des commandes du pont" ordre1
//      Gpio_SetElectrostimulation(eETAPE1); //RAZ
//       break;
//    case 1:
//     // Configuration du timming prochaine étape
//      Timer_SetMft1Timming(ui16TimeApplicationAop);
//      // Application Mise à la masse du pont
//      Gpio_SetElectrostimulation(eETAPE2); //L1-L2
//      //  Application AOP -> ON
//       Gpio_SetAop();
//     // Petit d�lai par des cycles horloge
//     PETIT_DELAI_NOP;
//     //Mise en forme Intensité programmé
//     (void)Ad5691r_SetIntensiteStimulation(IntensiteProgramme); //exprimer en uV
//      break;
//      //--------------------------------------------------------------------------
//      // *********                   Polarit� -> HAUTE                 ***********
//      //--------------------------------------------------------------------------
//    case 2: // Etape de rebouclage de g�n�ration du stimuli par la polarit� -> HAUTE
//      // Configuration du timming prochaine étape
//      Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
//      // Application EPH
//      Gpio_SetElectrostimulation(eETAPE3); // Haut L1-H2
//          break;
//    case 3: // Bas
//
//      // Configuration du timming prochaine étape
//      Timer_SetMft1Timming(iSS_DUREE_IMPULSION);/// Variable qui donne le temps de la pulsation Delta "t" gStimGen_t.tPulse[pulseId].cntWidth
//      // R�cup�ration de la mesure de courant de relecture �ventuelle
//      (void)Adc_TraitementAcquisitionCourantRelecture();
//              break;
//   //--------------------------------------------------------------------------
//       case 4: //Etape reset
//         // Application EPB
//            Gpio_SetElectrostimulation(eETAPE4); // Reset H2-L1
//     //       Gpio_SetElectrostimulation(eETAPE6); //BAS H1-L2
//            // Configuration du timming prochaine étape
//                Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiMIN_TIMER_MFT1);
//                break;
//
//    //--------------------------------------------------------------------------
//    // *********                    BASE = PALIER                    ***********
//    //--------------------------------------------------------------------------
//    case 5: // Bas
//      // Configuration du timming prochaine étape
//      Timer_SetMft1Timming((iSS_DUREE_TPS_BASE-1733)/2);/// Variable  rapport périodique entre chaque impulsion
//         //Application "RAZ des commandes du pont" ordre7
//          Gpio_SetElectrostimulation(eETAPE7); //Reset
//          (void)Ad5691r_SetIntensiteStimulation(600); // à enlever
//          // Application AOP -> OFF
//          Gpio_ClrAop();
//              break;
//    case 6: // OFF
//      // Configuration du timming prochaine étape
//        Timer_SetMft1Timming(iSS_DUREE_TPS_BASE/2);/// Variable rapport périodique entre chaque impulsion
//      state = 0;
//      first++;
//      break;
//    default:
//      break;
//    }
//  }
//
//  // Configuration prochaine �tape
//  state++;
//}
//-------------------- POD3607_LIO_V2----------------------
/*!
 * \fn void MeSS_GestionCourantBiphasique(void)
 * \brief Fonction de gestion des courant Monophasique +
 */
void MeSS_GestionCourantMonophasiquePositif(void)
{
  static uint8_t state = 0;
  static uint32_t IntensiteProgramme = 20000;
  static uint8_t first = 0;
  static uint16_t ui16TimeApplicationAop = 500;

  GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);

  if (first < 20)
  {
       switch (state)
    {
    case 0:
      // Configuration du timming prochaine étape
      Timer_SetMft1Timming(iSS_TIMMING_COURT);
      //Application "RAZ des commandes du pont" ordre1
      Gpio_SetElectrostimulation(eETAPE1); //RAZ
       break;
    case 1:
     // Configuration du timming prochaine étape
      Timer_SetMft1Timming(ui16TimeApplicationAop);
      // Application Mise à la masse du pont
      Gpio_SetElectrostimulation(eETAPE2); //L1-L2
      //  Application AOP -> ON
       Gpio_SetAop();
     // Petit d�lai par des cycles horloge
     PETIT_DELAI_NOP;
     //Mise en forme Intensité programmé
     (void)Ad5691r_SetIntensiteStimulation(IntensiteProgramme); //exprimer en uV
      break;
      //--------------------------------------------------------------------------
      // *********                   Polarit� -> HAUTE                 ***********
      //--------------------------------------------------------------------------
    case 2: // Etape de rebouclage de g�n�ration du stimuli par la polarit� -> HAUTE
      // Configuration du timming prochaine étape
      Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
      // Application EPH
      Gpio_SetElectrostimulation(eETAPE3); // Haut L1-H2
          break;
    case 3: // Bas

      // Configuration du timming prochaine étape
      Timer_SetMft1Timming(iSS_DUREE_IMPULSION);/// Variable qui donne le temps de la pulsation Delta "t"
      // R�cup�ration de la mesure de courant de relecture �ventuelle
      (void)Adc_TraitementAcquisitionCourantRelecture();
              break;
   //--------------------------------------------------------------------------
       case 4: //Etape reset
         // Application EPB
            Gpio_SetElectrostimulation(eETAPE4); // Reset H2-L1
     //       Gpio_SetElectrostimulation(eETAPE6); //BAS H1-L2
            // Configuration du timming prochaine étape
                Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiMIN_TIMER_MFT1);
                break;

    //--------------------------------------------------------------------------
    // *********                    BASE = PALIER                    ***********
    //--------------------------------------------------------------------------
    case 5: // Bas
      // Configuration du timming prochaine étape
      Timer_SetMft1Timming((iSS_DUREE_TPS_BASE-1733)/2);/// Variable  rapport périodique entre chaque impulsion
         //Application "RAZ des commandes du pont" ordre7
          Gpio_SetElectrostimulation(eETAPE7); //Reset
          //(void)Ad5691r_SetIntensiteStimulation(600); // à enlever
          // Application AOP -> OFF
          Gpio_ClrAop();
              break;
    case 6: // OFF
      // Configuration du timming prochaine étape
        Timer_SetMft1Timming(iSS_DUREE_TPS_BASE/2);/// Variable rapport périodique entre chaque impulsion
      state = 0;
      first++;
      break;
    default:
      break;
    }
  }

  // Configuration prochaine �tape
  state++;
}

//-----------------------------------------------------------------------------
/*!
 * \fn void MeSS_GestionCourantMonophasiqueNegatif(void)
 * \brief Fonction de gestion des courants Monophasique Négatif
 */
void MeSS_GestionCourantMonophasiqueNegatif(void)
{
  static uint8_t state = 0;
  static uint32_t IntensiteProgramme = 15000;
  static uint8_t first = 0;
  static uint16_t ui16TimeApplicationAop = 500;

  GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);
  if (first < 20)
   {
        switch (state)
     {
     case 0:
       // Configuration du timming prochaine étape
       Timer_SetMft1Timming(iSS_TIMMING_COURT);
       //Application "RAZ des commandes du pont" ordre1
       Gpio_SetElectrostimulation(eETAPE1); //RAZ
        break;
     case 1:
      // Configuration du timming prochaine étape
       Timer_SetMft1Timming(ui16TimeApplicationAop);
       // Application Mise à la masse du pont
       Gpio_SetElectrostimulation(eETAPE2); //L1-L2
       //  Application AOP -> ON
        Gpio_SetAop();
      // Petit d�lai par des cycles horloge
      PETIT_DELAI_NOP;
      //Mise en forme Intensité programmé
      (void)Ad5691r_SetIntensiteStimulation(IntensiteProgramme); //exprimer en uV
       break;
       //--------------------------------------------------------------------------
       // *********                   Polarit� -> BASSE                 ***********
       //--------------------------------------------------------------------------
     case 2: // Etape de rebouclage de g�n�ration du stimuli par la polarit� -> BASSE
       // Configuration du timming prochaine étape
       Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
       // Application EPH
       Gpio_SetElectrostimulation(eETAPE8); // Haut L1-H2
           break;
     case 3: // Bas

       // Configuration du timming prochaine étape
       Timer_SetMft1Timming(iSS_DUREE_IMPULSION);/// Variable qui donne le temps de la pulsation Delta "t"
       // R�cup�ration de la mesure de courant de relecture �ventuelle
       (void)Adc_TraitementAcquisitionCourantRelecture();
               break;
    //--------------------------------------------------------------------------
        case 4: //Etape reset
          // Application EPB
             Gpio_SetElectrostimulation(eETAPE9); // Reset H2-L1
             // Configuration du timming prochaine étape
                 Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiMIN_TIMER_MFT1);
                 break;

     //--------------------------------------------------------------------------
     // *********                    BASE = PALIER                    ***********
     //--------------------------------------------------------------------------
     case 5: // Bas
       // Configuration du timming prochaine étape
       Timer_SetMft1Timming((iSS_DUREE_TPS_BASE-1733)/2);/// Variable  rapport périodique entre chaque impulsion
          //Application "RAZ des commandes du pont" ordre7
           Gpio_SetElectrostimulation(eETAPE7); //Reset
           (void)Ad5691r_SetIntensiteStimulation(600); // à enlever
           // Application AOP -> OFF
           Gpio_ClrAop();
               break;
     case 6: // OFF
       // Configuration du timming prochaine étape
         Timer_SetMft1Timming(iSS_DUREE_TPS_BASE/2);/// Variable rapport périodique entre chaque impulsion
       state = 0;
       first++;
       break;
     default:
       break;
     }
   }

   // Configuration prochaine �tape
   state++;

}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/*!
 * \fn void MeSS_GestionCourantBiphasiquePositif(void)
 * \brief Fonction de gestion des courant BiphasiqueAlterne
 */
void MeSS_GestionCourantBiphasiqueAlterne(void)
{
  static uint8_t state = 0;
 // static uint16_t v = 0; //(400 * 4095) / 3300;
  static uint32_t IntensiteProgramme = 15000;
  static uint8_t first = 0;
  static uint16_t ui16TimeApplicationAop = 500;

  GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);

  if (first < 20)
  {
       switch (state)
    {
    case 0:
      // Configuration du timming prochaine étape
      Timer_SetMft1Timming(iSS_TIMMING_COURT);
      //Application "RAZ des commandes du pont" ordre1
      Gpio_SetElectrostimulation(eETAPE1); //RAZ
       break;
    case 1:
     // Configuration du timming prochaine étape
      Timer_SetMft1Timming(ui16TimeApplicationAop);
      // Application Mise à la masse du pont
      Gpio_SetElectrostimulation(eETAPE2); //L1-L2
      //  Application AOP -> ON
       Gpio_SetAop();
     // Petit d�lai par des cycles horloge
     PETIT_DELAI_NOP;
     //Mise en forme Intensité programmé
     (void)Ad5691r_SetIntensiteStimulation(IntensiteProgramme); //exprimer en uV
      break;
      //--------------------------------------------------------------------------
      // *********                   Polarit� -> HAUTE                 ***********
      //--------------------------------------------------------------------------
    case 2: // Etape de rebouclage de g�n�ration du stimuli par la polarit� -> HAUTE
      // Configuration du timming prochaine étape
      Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
      // Application EPH
      Gpio_SetElectrostimulation(eETAPE3); // Haut L2-H1
          break;
    case 3: // Bas

      // Configuration du timming prochaine étape
      Timer_SetMft1Timming(iSS_DUREE_IMPULSION);/// Variable qui donne le temps de la pulsation Delta "t"
      // R�cup�ration de la mesure de courant de relecture �ventuelle
      (void)Adc_TraitementAcquisitionCourantRelecture();
              break;
   //--------------------------------------------------------------------------
   // *********                   Polarit� -> BASSE                 ***********
   //--------------------------------------------------------------------------
       case 4: //Etape de changement de polarit� -> BASSE
         // Application EPB
            Gpio_SetElectrostimulation(eETAPE4); // Reset H1-L2
            Gpio_SetElectrostimulation(eETAPE6); //BAS H2-L1
            // Configuration du timming prochaine étape
                Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiMIN_TIMER_MFT1);
                break;
    case 5: // Bas
        // Configuration du timming prochaine étape
          Timer_SetMft1Timming(iSS_DUREE_IMPULSION-uiMIN_TIMER_MFT1);/// Variable qui donne le temps de la pulsation Delta "t"
        // R�cup�ration de la mesure de courant de relecture �ventuelle
              (void)Adc_TraitementAcquisitionCourantRelecture();
              break;
    //--------------------------------------------------------------------------
    // *********                    BASE = PALIER                    ***********
    //--------------------------------------------------------------------------
    case 6: // Bas
      // Configuration du timming prochaine étape
      Timer_SetMft1Timming((iSS_DUREE_TPS_BASE-1733));/// Variable  rapport périodique entre chaque impulsion
         //Application "RAZ des commandes du pont" ordre7
          Gpio_SetElectrostimulation(eETAPE7); //Reset
          (void)Ad5691r_SetIntensiteStimulation(600); // à enlever
          // Application AOP -> OFF
          Gpio_ClrAop();
              break;

      //--------------------------------------------------------------------------
         // *********          ALTERNE COURANT BIPHASIQUE NEGATIF     ***********
         //--------------------------------------------------------------------------
      case 7:
          // Configuration du timming prochaine étape
          Timer_SetMft1Timming(iSS_TIMMING_COURT);
          //Application "RAZ des commandes du pont" ordre1
          Gpio_SetElectrostimulation(eETAPE1); //RAZ
           break;
       case 8:
         // Configuration du timming prochaine étape
          Timer_SetMft1Timming(ui16TimeApplicationAop);
          // Application Mise à la masse du pont
          Gpio_SetElectrostimulation(eETAPE2); //L1-L2
          //  Application AOP -> ON
           Gpio_SetAop();
         // Petit d�lai par des cycles horloge
         PETIT_DELAI_NOP;
         //Mise en forme Intensité programmé
         (void)Ad5691r_SetIntensiteStimulation(IntensiteProgramme); //exprimer en uV
          break;
          //--------------------------------------------------------------------------
          // *********                   Polarit� -> BASSE                 ***********
          //--------------------------------------------------------------------------
        case 9: // Etape de rebouclage de g�n�ration du stimuli par la polarit� -> BASSE
          // Configuration du timming prochaine étape
          Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiERREUR_TIMER_MFT1);
          // Application EPH
          Gpio_SetElectrostimulation(eETAPE8); // BAS L1-H2
              break;
        case 10: // Bas

          // Configuration du timming prochaine étape
          Timer_SetMft1Timming(iSS_DUREE_IMPULSION);/// Variable qui donne le temps de la pulsation Delta "t"
          // R�cup�ration de la mesure de courant de relecture �ventuelle
          (void)Adc_TraitementAcquisitionCourantRelecture();
                  break;
       //--------------------------------------------------------------------------
       // *********                   Polarit� -> HAUTE                 ***********
       //--------------------------------------------------------------------------
         case 11: //Etape de changement de polarit� -> HAUTE
             // Application EPB
                Gpio_SetElectrostimulation(eETAPE9); // Reset H2-L1
                Gpio_SetElectrostimulation(eETAPE10); //HAUT H1-L2
                // Configuration du timming prochaine étape
                    Timer_SetMft1Timming(iSS_MOMENT_RELECTURE_COURANT-uiMIN_TIMER_MFT1);
                    break;
         case 12: // Bas
            // Configuration du timming prochaine étape
              Timer_SetMft1Timming(iSS_DUREE_IMPULSION-uiMIN_TIMER_MFT1);/// Variable qui donne le temps de la pulsation Delta "t"
            // R�cup�ration de la mesure de courant de relecture �ventuelle
                  (void)Adc_TraitementAcquisitionCourantRelecture();
                  break;
        //--------------------------------------------------------------------------
        // *********                    BASE = PALIER                    ***********
        //--------------------------------------------------------------------------
        case 13: // Bas
          // Configuration du timming prochaine étape
          Timer_SetMft1Timming((iSS_DUREE_TPS_BASE-1733)/2);/// Variable  rapport périodique entre chaque impulsion
             //Application "RAZ des commandes du pont" ordre7
              Gpio_SetElectrostimulation(eETAPE7); //Reset
              (void)Ad5691r_SetIntensiteStimulation(600); // à enlever
              // Application AOP -> OFF
              Gpio_ClrAop();
                  break;
        case 14: // OFF
          // Configuration du timming prochaine étape
            Timer_SetMft1Timming(iSS_DUREE_TPS_BASE/2);/// Variable rapport périodique entre chaque impulsion
          state = 0;
          first++;
          break;
    default:
      break;
    }
  }

  // Configuration prochaine �tape
  state++;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


/************************************************************************************
* Name :  VeineuxBiphas   *//**
* @brief  Veineux biphasic Impuls
* @param  .
* @return .
************************************************************************************/
void VeineuxBiphas(void)
{
  static uint8_t u8PulseStepIdx = 0;
  static uint16_t i = 0, Nloop = 0, DigAmp=0;
  uint16_t tmp = 0;

  /** Veineux Biphasic pulse states */
  static enum
  {
    gVeineuxBiphasStateInit_c = 0,
    gVeineuxBiphasStatePosRise_c,     /**< Positive Pulse, rising edge */
    gVeineuxBiphasStatePosFall_c,       /**< Positive Pulse, falling edge */
    gVeineuxBiphasStatePosFall_1_c,
    gVeineuxBiphasStateNegFall_c,       /**< Negative Pulse, falling edge */
    gVeineuxBiphasStateNegRise_c,       /**< Negative Pulse, rising edge */
    gVeineuxBiphasStateNegRise_1_c,       /**< Negative Pulse, rising edge */
    gVeineuxBiphasStateInter_c,         /**< Inter Pulse */
    gVeineuxBiphasStateNull_c,          /**< Null Pulse */
    gVeineuxBiphasStateNullLoop_c,
    gVeineuxBiphasStateInterNeg_c,        /**< Inter Pulse */
    gVeineuxBiphasStateNullNeg_c,       /**< Null Pulse */
    gVeineuxBiphasStateNullLoopNeg_c,

    gVeineuxBiphasStateMax_c
  } tVeineuxBiphasState = gVeineuxBiphasStateInit_c;

  switch(tVeineuxBiphasState)
  {

   case gVeineuxBiphasStateInit_c:

        // Configuration du timming prochaine étape
         STIM_GEN_RELOAD_NEXT_COUNT(iSS_TIMMING_COURT);
         CMD_M_DISCONNECT;       /**< Desactive CMD */
         CMD_M_SET_NO_PULSE;    //L1-L2
         tVeineuxBiphasState = gVeineuxBiphasStatePosRise_c;
   break;

    /** Positive Pulse, rising edge */
   case gVeineuxBiphasStatePosRise_c :

         /**  Application AOP -> ON */
         Gpio_SetAop();
         // Petit d�lai par des cycles horloge
         PETIT_DELAI_NOP;
         /** Sets Level */
         tmp = TablePuissanceCalculee[i][u8PulseStepIdx];
         DigAmp = NORMAL_MODE | ( (tmp & 0x0FFF) << 2 );
         /** Sets Level */
         (void)Ad5691r_SetIntensiteStimulation(DigAmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
         /** Sets Commands */
         if(gflag[0] == TRUE)
         {
             GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);//STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);      /**< Active Pulse Output */
             CMD_M_SET_POSITIVE_PULSE;       /**< Enables Positive pulse CMD */
             SWITCH_START;             /**< Enables switching */
             gflag[0] = FALSE;
          }
          u8PulseStepIdx++;
           /** Sets Next Step Time */
          STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);

          if( u8PulseStepIdx >= (gNbExponentialValue_c-1) )
          {
              tVeineuxBiphasState = gVeineuxBiphasStatePosFall_c;
              u8PulseStepIdx = 0;
           }
     break;

     /** Positive Pulse, falling edge */
    case gVeineuxBiphasStatePosFall_c :

         /** Sets Level */
         PETIT_DELAI_NOP;
         tmp = gStimGen_t.tPulse[i].digitalAmplitude - TablePuissanceCalculee[i][u8PulseStepIdx];
         if( gStimGen_t.tPulse[i].digitalAmplitude < TablePuissanceCalculee[i][u8PulseStepIdx] )
           tmp = 0;
         DigAmp = NORMAL_MODE | ( (tmp & 0x0FFF) << 2 );
         (void)Ad5691r_SetIntensiteStimulation(DigAmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
         /** Sets Commands */
         u8PulseStepIdx++;
         /** Sets Next Step Time */
         STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
         tVeineuxBiphasState = gVeineuxBiphasStatePosFall_1_c;
    break;

    case gVeineuxBiphasStatePosFall_1_c:

         if( u8PulseStepIdx == (gNbExponentialValue_c-1) )
         {
            u8PulseStepIdx = 0;
            /** Sets Next Step Time */
            if(++i < gStimGen_t.nPulse)       /**< Next Pulse */
            {
               STIM_GEN_RELOAD_NEXT_COUNT(2000);
               tVeineuxBiphasState = gVeineuxBiphasStateInter_c;
            }
            else                  /**< No Pulse */
            {
               STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
               i = 0;
               tVeineuxBiphasState = gVeineuxBiphasStateNull_c;
            }
          }
    break;

      /** Inter Pulse */
    case gVeineuxBiphasStateInter_c :

        /** Sets Commands */
        CMD_M_DISCONNECT;             /**< Disables pulse CMD */
        STIM_OUT_SEL_NONE;            /**< Disables Pulse Output */
        gflag[0] = TRUE;
        /** Sets Level */
        /**  Application AOP -> OFF */
        Gpio_ClrAop();
        tmp = NORMAL_MODE;
        // (void)Ad5691r_SetIntensiteStimulation(DigAmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
        /** Sets pause cmd */
        CMD_M_SET_NO_PULSE;
        /** Sets Next Step Time */
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
        tVeineuxBiphasState = gVeineuxBiphasStatePosRise_c;
     break;

    /** Null Pulse */
    case gVeineuxBiphasStateNull_c  :

        /** Sets Commands */
        //CMD_DEMAG_DIS;              /**< Disables Pulse */
        SWITCH_STOP;              /**< Disables switching */
        CMD_M_DISCONNECT;             /**< Disables pulse CMD */
        STIM_OUT_SEL_NONE;            /**< Disables Pulse Output */
        gflag[0] = TRUE;
        /**  Application AOP -> OFF */
        Gpio_ClrAop();
        /** Sets Level */
        tmp = NORMAL_MODE;
        // (void)Ad5691r_SetIntensiteStimulation(DigAmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
        /** Sets pause cmd */
        CMD_M_SET_NO_PULSE;
        /** Sets Next Step Time */
        if (gStimGen_t.nTimerLoop > 0)      /**< Next Loop */
          {
            Nloop = gStimGen_t.nTimerLoop;
            STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
            tVeineuxBiphasState = gVeineuxBiphasStateNullLoop_c;
          }
        else                  /**< First Pulse */
          {
            STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
            if(cpt>0)
              {
                tVeineuxBiphasState = gVeineuxBiphasStatePosRise_c;
                cpt--;
              }
            else
              {
                tVeineuxBiphasState = gVeineuxBiphasStateNegFall_c;
                cpt = 7;
              }
          }
    break;

    case gVeineuxBiphasStateNullLoop_c  :

        Nloop--;
        if(Nloop > 0)             /**< Next Loop */
          STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
        else                  /**< First Pulse */
          {
            STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
            if(cpt>0)
              {
                tVeineuxBiphasState = gVeineuxBiphasStatePosRise_c;
                cpt--;
              }
            else
              {
                tVeineuxBiphasState = gVeineuxBiphasStateNegFall_c;
                cpt = 7;
              }
          }
     break;


    /** Negative Pulse, falling edge */
    case gVeineuxBiphasStateNegFall_c :

      /** Sets Level */
      /**  Application AOP -> ON */
      Gpio_SetAop();
      // Petit d�lai par des cycles horloge
      PETIT_DELAI_NOP;
      tmp = TablePuissanceCalculee[i][u8PulseStepIdx];
      DigAmp = NORMAL_MODE | ( (tmp & 0x0FFF) << 2 );
      (void)Ad5691r_SetIntensiteStimulation(DigAmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
      /** Sets Commands */
      if(gflag[0] == TRUE)
      {
        GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);// STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);  /**< Active Pulse Output */
        CMD_M_SET_NEGATIVE_PULSE;       /**< Enables Negative pulse CMD */
        SWITCH_START;             /**< Enables switching */
        gflag[0] = FALSE;
      }
      u8PulseStepIdx++;
      /** Sets Next Step Time */
      STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
      if( u8PulseStepIdx == (gNbExponentialValue_c-1) )
      {
        tVeineuxBiphasState = gVeineuxBiphasStateNegRise_c;
        u8PulseStepIdx = 0;
      }
    break;

    /** Negative Pulse, Rising edge */
    case gVeineuxBiphasStateNegRise_c :

      /** Sets Level */
       tmp = gStimGen_t.tPulse[i].digitalAmplitude - TablePuissanceCalculee[i][u8PulseStepIdx];
       if(gStimGen_t.tPulse[i].digitalAmplitude<TablePuissanceCalculee[i][u8PulseStepIdx])
        tmp = 0;
      DigAmp = NORMAL_MODE | ( (tmp & 0x0FFF) << 2 );
       PETIT_DELAI_NOP;
       (void)Ad5691r_SetIntensiteStimulation(DigAmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude

       /** Sets Commands */
       //  STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);      /**< Active Pulse Output */
       //  CMD_M_SET_NEGATIVE_PULSE;       /**< Enables Negative pulse CMD */
       //  SWITCH_START;             /**< Enables switching */
       /** Sets Next Step Time */
       STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
       u8PulseStepIdx++;
       tVeineuxBiphasState = gVeineuxBiphasStateNegRise_1_c;
    break;

    case  gVeineuxBiphasStateNegRise_1_c:


      if( u8PulseStepIdx == (gNbExponentialValue_c-1) )
            {
              u8PulseStepIdx = 0;
              /** Sets Next Step Time */
              if(++i < gStimGen_t.nPulse)       /**< Next Pulse */
              {
                STIM_GEN_RELOAD_NEXT_COUNT(2000);
                tVeineuxBiphasState = gVeineuxBiphasStateInterNeg_c;

              }
              else                  /**< No Pulse */
              {
                STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.cntTr);
                i = 0;
                tVeineuxBiphasState = gVeineuxBiphasStateNullNeg_c;
              }
            }



    break;

    /** Inter Pulse */
    case gVeineuxBiphasStateInterNeg_c  :

        /** Sets Commands */
        SWITCH_STOP;              /**< Disables switching */
        CMD_M_DISCONNECT;             /**< Disables pulse CMD */
        STIM_OUT_SEL_NONE;            /**< Disables Pulse Output */
        gflag[0] = TRUE;
        /**  Application AOP -> OFF */
        Gpio_ClrAop();
        /** Sets Level */
        PETIT_DELAI_NOP;
        tmp = NORMAL_MODE;
        //  (void)Ad5691r_SetIntensiteStimulation(DigAmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude

        /** Sets pause cmd */
        CMD_M_SET_NO_PULSE;

        /** Sets Next Step Time */
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
        tVeineuxBiphasState = gVeineuxBiphasStateNegFall_c;

     break;

    /** Null Pulse */
    case gVeineuxBiphasStateNullNeg_c :

      /** Sets Commands */
      //  CMD_DEMAG_DIS;              /**< Disables Pulse */
      SWITCH_STOP;              /**< Disables switching */
      CMD_M_DISCONNECT;             /**< Disables pulse CMD */
      STIM_OUT_SEL_NONE;            /**< Disables Pulse Output */
      gflag[0] = TRUE;

      /**  Application AOP -> OFF */
      Gpio_ClrAop();
      PETIT_DELAI_NOP;
      /** Sets Level */
      tmp = NORMAL_MODE;
      //(void)Ad5691r_SetIntensiteStimulation(DigAmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
      /** Sets pause cmd */
      CMD_M_SET_NO_PULSE;

      /** Sets Next Step Time */
      if (gStimGen_t.nTimerLoop > 0)      /**< Next Loop */
      {
        Nloop = gStimGen_t.nTimerLoop;
        STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
        tVeineuxBiphasState = gVeineuxBiphasStateNullLoopNeg_c;
      }
      else                  /**< First Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
        if(cpt>0)
        {
          tVeineuxBiphasState = gVeineuxBiphasStateNegFall_c;
          cpt--;
        }
        else
        {
          tVeineuxBiphasState = gVeineuxBiphasStatePosRise_c;
          cpt = 7;
        }
      }

     break;

    case gVeineuxBiphasStateNullLoopNeg_c :

      Nloop--;
      if(Nloop > 0)             /**< Next Loop */
        STIM_GEN_RELOAD_NEXT_COUNT(STIM_GEN_COUNT_MAX);
      else                  /**< First Pulse */
      {
        STIM_GEN_RELOAD_NEXT_COUNT(gStimGen_t.tPulse[i].cntWidth);
        if(cpt>0)
        {
          tVeineuxBiphasState = gVeineuxBiphasStateNegFall_c;
          cpt--;
        }
        else
        {
          tVeineuxBiphasState = gVeineuxBiphasStatePosRise_c;
          cpt = 7;
        }
      }

      break;

    default :
    /** TODO !!!!! */
      break;

  }

}
/**********************************************************************************
End of function
***********************************************************************************/


/************************************************************************************
* Name :  NeuroMonophas   *//**
* @brief  .
* @param  .
* @return .
************************************************************************************/

void NeuroMonophas(void)
{
  static uint16_t i = 0 ;
  uint16_t tmp = 0;
  CMD_GALV_SEL_NONE;
   /** Monophasic pulse states */
  static enum
  {
    gWidthStatePos_c = 1,     /**< Positive Current */
    gWidthStateNeg_c = 0,     /**< Negative Current */
    gWidthStateNull_c = 2,      /**< Null Pulse */
    gWidthStateMax_c
  } tWidthState;

  if( gStimGen_t.FreqDiff == 1)
  {
    tWidthState = gWidthStateNeg_c;
    tWidthState = gWidthStateNeg_c;

  }
  if( gStimGen_t.FreqDiff == 2)
  {
    tWidthState = gWidthStatePos_c;
    tWidthState = gWidthStatePos_c;
  }

  switch(tWidthState)
  {

    /** Positive Current */
    case gWidthStatePos_c :

          //      CMD_GALV_SEL_NONE;  /**<  Desactive CMD and Opto. */

      tmp = NORMAL_MODE | ( (gStimGen_t.tPulse[i].digitalAmplitude & 0x0FFF) << 2 );
      if(gStimGen_t.tPulse[i].digitalAmplitude == 0)
      {
      //  SWITCH_STOP;
        CMD_GALV_SEL_NONE;
        (void)Ad5691r_SetIntensiteStimulation(0); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
      }
      else
      {
         (void)Ad5691r_SetIntensiteStimulation(tmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude
        /** Sets Commands */
         GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);//  STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);  /**< Active Pulse Output */
        CMD_GALV_SEL_POS;     /**< Enables positive current. */
        SWITCH_START;             /**< Enables switching */
          //  tWidthState = gWidthStateNull_c;
        STIM_GEN_RELOAD_NEXT_COUNT(5000/*STIM_GEN_COUNT_MAX*/);

      }
        break;

    /** Negative Current */
    case gWidthStateNeg_c :

      //CMD_GALV_SEL_NONE;    /**<  Desactive CMD and Opto. */
      /** Sets levels */
      tmp = NORMAL_MODE | ( (gStimGen_t.tPulse[i].digitalAmplitude & 0x0FFF) << 2 );
      if(gStimGen_t.tPulse[i].digitalAmplitude == 0)
      {
        CMD_GALV_SEL_NONE;
        (void)Ad5691r_SetIntensiteStimulation(0); //exprimer en uV
      }
      else
      {
        (void)Ad5691r_SetIntensiteStimulation(tmp); //exprimer en uV
                /** Sets Commands */
        GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);// STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);  /**< Active Pulse Output */
        CMD_GALV_SEL_NEG;     /**< Enables positive current. */
        SWITCH_START;             /**< Enables switching */
      //  tWidthState = gWidthStateNull_c;
        STIM_GEN_RELOAD_NEXT_COUNT(/*STIM_GEN_COUNT_MAX*/5000);
      }

      break;

    /** Null Current */
    case gWidthStateNull_c :

      CMD_GALV_SEL_NONE;

      break;

    default:
      break;
  }

}


/**********************************************************************************
End of function
***********************************************************************************/


/************************************************************************************
* Name :  Galvanic  *//**
* @brief  Direct Current
* @param  .
* @return .
************************************************************************************/

void Galvanic(void)
{
  static uint16_t i = 0 ;
  uint16_t tmp = 0/*,tmp2=0*/;



  /** Monophasic pulse states */
  static enum
  {
    gWidthStatePos_c = 1,     /**< Positive Current */
    gWidthStateNeg_c = 0,     /**< Negative Current */
    gWidthStateNull_c = 2,      /**< Null Pulse */
    gWidthStateMax_c
  } tWidthState = gWidthStatePos_c;


//  tmp2 = NORMAL_MODE | ( (gStimGen_t.tPulse[i].cntWidth+1 & 0x0FFF) << 2 );
  gStimGen_t.FreqDiff;
  if(gStimGen_t.FreqDiff == 0)
  {
    tWidthState = gWidthStateNeg_c;
    tWidthState = gWidthStateNeg_c;
  }
  else
  {
    tWidthState = gWidthStatePos_c;
    tWidthState = gWidthStatePos_c;
  }
  switch(tWidthState)
  {

    /** Positive Current */
    case gWidthStatePos_c :

//      CMD_GALV_SEL_NONE;  /**<  Desactive CMD and Opto. */

      /** Sets levels */
          tmp = NORMAL_MODE | ( ((gStimGen_t.tPulse[i].digitalAmplitude) & 0x0FFF) << 2 );
      if(gStimGen_t.tPulse[i].digitalAmplitude == 0)
      {
        CMD_GALV_SEL_NONE;
      }
      else
      {
          (void)Ad5691r_SetIntensiteStimulation(tmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude

        /** Sets Commands */
        GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);// STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);  /**< Active Pulse Output */
        CMD_GALV_SEL_POS;     /**< Enables positive current. */
        SWITCH_START;             /**< Enables switching */
        /** Current Measurement */
        gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current() ;
      }

        break;

    /** Negative Current */
    case gWidthStateNeg_c :

//      CMD_GALV_SEL_NONE;    /**<  Desactive CMD and Opto. */
      /** Sets levels */
      tmp = NORMAL_MODE | ( ((gStimGen_t.tPulse[i].digitalAmplitude) & 0x0FFF) << 2 );
      if(gStimGen_t.tPulse[i].digitalAmplitude == 0)
      {
        CMD_GALV_SEL_NONE;
      }
      else
      {
          (void)Ad5691r_SetIntensiteStimulation(tmp); //exprimer en uV gStimGen_t.tPulse[i].digitalAmplitude

        /** Sets Commands */
        GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);// STIM_OUT_SEL(gStimOutCmd_c[gStimGen_t.tPulse[i].outId]);  /**< Active Pulse Output */
        CMD_GALV_SEL_NEG;   /**< Enables poitive current. */
        SWITCH_START;             /**< Enables switching */

      //  tWidthState = gWidthStateNull_c;

        /** Current Measurement */

        gStimGen_t.tPulse[i].digitalMeasAmplitude = IADC_Read_Current() ;
      }

      break;

    /** Null Current */
    case gWidthStateNull_c :

      CMD_GALV_SEL_NONE;

    break;

    default :

  break;

  }

}
/**********************************************************************************
End of function
***********************************************************************************/


/************************************************************************************
* Name :  ElectrodeAdhesionDetection  *//**
* @brief  .
* @param  .
* @return .
************************************************************************************/

bool_t ElectrodeAdhesionDetection(StimOutId_t OutId)
{


  // D�claration des variables
  //-----------------------------
  uint16_t RestAmplitud=0, tmp=0, Amplitud=0, Value=0, digAmpl=0, MeanAmplitud=0, k=1750;
  bool_t flag=FALSE;
  uint8_t  cpt=0, i=0, l;

//-----------------------------------------------------------------------------

  // Activation de l'optocoupleur de detection.
  DETECT_RES_CS_EN;
  while(k>0)
    k--;
  // Mesure de la valeur au repos.

//  for(i=0; i<10; i++)
//  {
  //------------------------------
    do
    {
    // Envoi de tmp=0 sur le DAC

    CMD_M_DISCONNECT;       /**< Desactive CMD */
    //CMD_DEMAG_EN;             /**< Enables Pulse */

    digAmpl = NORMAL_MODE;

  /*  STIM_GEN_RELOAD_NEXT_COUNT(500);
    CMD_M_SET_NO_PULSE; */ //Ajout Lio
    /**Application AOP -> ON */
    Gpio_SetAop();
    /** Sets Level */
    (void)Ad5691r_SetIntensiteStimulation(digAmpl);
  for(l=0; l<10; l++)
    {}
//----------------------------------------------------------------------------
    // Detection de la voie
    /** Sets Commands */
    if(OutId==0)
      GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);  /**< Active Pulse Output on Output 1*/
    else
      GPIO_PinOutSet(CS_VOIE2_PORT, CS_VOIE2_PIN);    /**< Active Pulse Output on Output 2*/

    /*  CMD_M_SET_POSITIVE_PULSE;       */
    Gpio_SetElectrostimulation(eETAPE10);/**< Enables Positive pulse CMD */
    SWITCH_START;             /**< Enables switching */

    k=63;
    sl_udelay_wait(50);
//    while(k>0)    // On attend 50us que le niveau s'établisse.
//      k--;
//-----------------------------------------------------------------------------
    /** Current Measurement */

    RestAmplitud =(IADC_Read_Current() /*& 0x0FFC*/ )/* >> 2*/ ;
    SWITCH_STOP;
    CMD_M_DISCONNECT;             /**< Disables pulse CMD */
    STIM_OUT_SEL_NONE;
    /**Application AOP -> OFF*/
       Gpio_ClrAop();
     }while(RestAmplitud==0x3FF);
//    MaxRest = MaxRest + RestAmplitud;
//  }
//  MaxRest /= 10;

  // Fin de la mesure de la valeur de repos.
//--------------------------------------------------------------------------------

  do
  {
    Amplitud = 0;
    for(i=0;i<10;i++)
    {
      k=80;
      // Envoi d'une impulsion et mesure.
//      CMD_M_DISCONNECT;       /**< Desactive CMD */
      //CMD_DEMAG_EN;             /**< Enables Pulse */
   //   STIM_GEN_RELOAD_NEXT_COUNT(500); //Ajout lio
     // CMD_M_SET_NO_PULSE; //Ajout lio
      /**Application AOP -> ON*/
         Gpio_SetAop();
     /** Sets Level */
      digAmpl =(tmp * fGAIN_DAC) + fOFFSET_DAC;

       (void)Ad5691r_SetIntensiteStimulation(digAmpl);

      for(l=0; l<10; l++)
      {}
    //----------------------------------------------------------------------------
      // detection de la voie.
      /** Sets Commands */
      if(OutId==0)
      GPIO_PinOutSet(CS_VOIE1_PORT, CS_VOIE1_PIN);  /**< Active Pulse Output on Output 1*/
      else
      GPIO_PinOutSet(CS_VOIE2_PORT, CS_VOIE2_PIN);    /**< Active Pulse Output on Output 2*/

      CMD_M_SET_POSITIVE_PULSE;       /**< Enables Positive pulse CMD */
     // Gpio_SetElectrostimulation(eETAPE10);/**< Enables Positive pulse CMD */
     // SWITCH_START;             /**< Enables switching */
    //----------------------------------------------------------------------------
      sl_udelay_wait(50);
    //-----------------------------------------------------------------------------

      /** Current Measurement */
      Amplitud = Amplitud + (( IADC_Read_Current()/* & 0x0FFC*/ )/* >> 2*/) ;
      CMD_M_SET_NO_PULSE;
      k=80;
      CMD_M_SET_NEGATIVE_PULSE;
    //  Gpio_SetElectrostimulation(eETAPE6);
      sl_udelay_wait(100);
      CMD_M_SET_NO_PULSE;
     // SWITCH_STOP;
      CMD_M_DISCONNECT;             /**< Disables pulse CMD */
      STIM_OUT_SEL_NONE;
      /**Application AOP -> OFF */
       Gpio_ClrAop();
      /** Sets Level */
      digAmpl = NORMAL_MODE;
      (void)Ad5691r_SetIntensiteStimulation(digAmpl);
      CMD_M_SET_NO_PULSE;
      k=200;
      sl_udelay_wait(100);
    }

    MeanAmplitud=Amplitud/10;

  //------------------------------------------------------------------------------------------
    if(MeanAmplitud> RestAmplitud)
    {
      Value = MeanAmplitud - RestAmplitud;
    }
    else
    {
      Value = 0;
    }

    if(Value>0x55)
    {
      flag=TRUE;
      cpt++;
    }
    else
    {
      flag=FALSE;
      cpt=0;
      if((tmp)<29)
        tmp=tmp+2;
      else
        break;
    }

  } while(flag!=TRUE || cpt<3);

  DETECT_RES_CS_DIS;
  k=200;
  while(k>0)
    k--;
  return flag;
}

/**********************************************************************************
End of function
***********************************************************************************/


