/**
* \file   main.c
* @ingroup  GrpAppl
* @brief  Main file of "STIM_BIO" project.
*
* Copyright : 2009 Vivaltis.\n
*         All Rights Reserved
*/

/**
* @defgroup GrpAppl Main Application
* This is the high level application group.
*/

/**
* @defgroup GrpBSP Hardware Abstraction Layer (HAL)
* This is the board management drivers group.
*/

/**
* @defgroup GrpPlm Platform Management
* This is the Platform layer management (PLM) group.
*/

/*! \mainpage A simple manual

This reference manual describes how the POD StimBio Software is designed.

This manual is divided in the following sections:
- \subpage intro
  - \subpage advanced "Advanced usage"
- \subpage Module index
- \subpage Data structure index
- \subpage File index
- \subpage Module documentation
- \subpage Data structure documentation
- \subpage File documentation
*/

//-----------------------------------------------------------

/*! \page intro Introduction
POD StimBio has two main tasks : Stimulation and Biofeedback.
Each task is controled by several functions.
All functions and data structures are described in this manual.
This page introduces the user to the topic.
*/

//-----------------------------------------------------------

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
/* rskr8c27def.h defines some common definitions and includes the sfr.h file */
#include "rskr8c27def.h"
#include "main.h"
#include "fsm.h"
#include "includes.h"

#include "bsp.h"
//#include "stim_management.h"
#include "bio_management.h"


//#define SET_CMD_PIN_DIR(mask, portDir)  GPIO_SET_PORT0_DIR(mask, portDir)

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define SRL_COMM_BUFF_SIZE_MAX      50        /**< srl comm.\ buff maximal size. Define the maximal size of srl comm buff */
#define SRL_COMM_GET_RESOURCE     DISABLE_IRQ   /** Macro that. define the maximal size of the srl comm buff */

#define SRL_COMM_RELEASE_RESOURCE   ENABLE_IRQ

#ifdef TEST_MODE_2
#define BUFF_SIZE_MAX         100
#endif


#include "sl_component_catalog.h"
#include "sl_system_init.h"
#include "app.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#include "sl_power_manager.h"
#endif
#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "sl_system_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
#include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
static void ApplicationInit(void);

int main(void)
{
  // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that if the kernel is present, processing task(s) will be created by
  // this call.
  sl_system_init();

  /*StimErr_t stimErr0, stimErr1;
    StimGenErr_t stimGenErr;
    StimConfigData_t stimConfigData_t;
    StimulationConfiguration_t  tStimConfig;*/


#ifdef TEST_MODE_2
    struct
    {
      int16_t   buffSampleA[BUFF_SIZE_MAX];
      uint8_t   readIdx;
      uint8_t   writeIdx;
      uint8_t   nAvailableMeas;

    } bioSampl_t;

    int16_t sample[5];
    uint8_t nSamp = 0;


    //uint8_t ConfigData[20] = {1,2,3,4,5,6,7,8,9,7,8,9,4,5,6,1,2,3};

    tStimConfig.patternId = gStimPatternBiphasic_c;
    tStimConfig.nStim = 2;
    tStimConfig.frequency = 50;                 // in Herz
    tStimConfig.tPattern[gStimOut0_c].outId = gStimOut0_c;
    tStimConfig.tPattern[gStimOut0_c].width = 200;        // in �s
    tStimConfig.tPattern[gStimOut0_c].amplitude = 400;      // in A*(10^-4)

    tStimConfig.tPattern[gStimOut1_c].outId = gStimOut1_c;
    tStimConfig.tPattern[gStimOut1_c].width = 100;        // in �s
    tStimConfig.tPattern[gStimOut1_c].amplitude = 600;      // in A*(10^-4)
#endif

  // Initialize the application. For example, create periodic timer(s) or
  // task(s) if the kernel is present.
  //app_init();

  BspInit();
  ApplicationInit();

#ifdef PIN_TEST_MODE_2

  while(1)
  {
    //SrlCommManagmntWriteData(tabRx, 32);
    //p0_0 = !p0_0 ; // pin test
    UartWriteData (tabRx, 32);
//    p0_0 = !p0_0 ; // pin test

  }
#endif


#ifdef TEST_MODE_2
  bioSampl_t.readIdx = 0;
  bioSampl_t.nAvailableMeas = 100;
  while(1)
  {

    b = TRUE;

    DISABLE_IRQ

    if(b/*gBioSampl_t.nAvailableMeas > 0*/)
    {
      for(i=0;i<2/*gBioSampl_t.nBio*/;i++)
      {
        for(j=0;j<5;j++)
        {
          sample[j] = bioSampl_t.buffSampleA[bioSampl_t.readIdx];
          bioSampl_t.readIdx = (bioSampl_t.readIdx + 1) % BUFF_SIZE_MAX;
        }
      }

      bioSampl_t.nAvailableMeas--;
      bioSampl_t.nAvailableMeas = bioSampl_t.nAvailableMeas - bioSampl_t.readIdx;
    }
//    p0_0 = ! p0_0 ; // pin test


    ENABLE_IRQ
/*
    for(i=0;i<0x000f;i++)
      for(j=0;j<0xffff;j++);

    p0_0 = ! p0_0 ; // pin test
*/

//    SrlCommManagmntWriteData(tabTx, 9);     // !!!!!!
  }
#endif

  //stimGenErr = StimManagementConfigPulse(&tStimConfig);

#ifdef TEST_MODE_2_BIO1
  tBioConfigData.acquisitionFrequency = 100;
  tBioConfigData.inputId = 1;
  tBioConfigData.bAbs = 0;

  BiofeedbackConfig(&tBioConfigData);

  tBioConfigData.acquisitionFrequency = 100;
  tBioConfigData.inputId = 0;

  BiofeedbackConfig(&tBioConfigData);

  tBioSelGainData.inputId = 0;
  tBioSelGainData.gain = 2;

//  BiofeedbackSelectGain(&tBioSelGainData);

  tBioSelGainData.inputId = 1;
  tBioSelGainData.gain = 2;

  BiofeedbackSelectGain(&tBioSelGainData);

  bioSelCutoffFreqData_t.cutoffFrequency = 1;
  bioSelCutoffFreqData_t.inputId = 0;

  BiofeedbackSelectCutOffFrequency(&bioSelCutoffFreqData_t);

  bioSelCutoffFreqData_t.cutoffFrequency = 1;
  bioSelCutoffFreqData_t.inputId = 1;

  BiofeedbackSelectCutOffFrequency(&bioSelCutoffFreqData_t);

  BiofeedbackStart(&nDataRcvd);
#endif

//  SrlCommManagmntWriteData(tabTx, 10);      // !!!!!!

#ifdef TEST_MODE
// Configuration de l'enveloppe sur la voie 0.
  stimConfigData_t.stimOutId = 0;
  stimConfigData_t.stimPatternId = 0x0B;
  stimConfigData_t.bElectrodeAdhesionDetect = 0;
  stimConfigData_t.delayPeriod = 0;
  stimConfigData_t.increasePeriod = 0;
  stimConfigData_t.plateauPeriod = 0x0A;
  stimConfigData_t.decreasePeriod = 0;
  stimConfigData_t.reposePeriod = 0;
  stimConfigData_t.patternFrequencyA = 100;   //Tm pour la neuro.
  stimConfigData_t.pulseWidthA = 300;   //Tp pour la neuro.
  stimConfigData_t.patternFrequencyB = 100;   //Tr pour la neuro.
  stimConfigData_t.pulseWidthB = 300;
  stimConfigData_t.patternFrequencyC = 100;
  stimConfigData_t.pulseWidthC = 300;
  stimConfigData_t.curAmplitude = 200;
  stimConfigData_t.maxAmplitude = 1000;
  stimConfigData_t.amplitudeIncrement = 5 ;
#ifdef V016
  stimConfigData_t.envelopeRepeat = 0;
#endif

  stimErr0 = StimulationConfig(&stimConfigData_t);

// Configuration de l'envelopppe sur la voie 1.
//  stimConfigData_t.delayPeriod = 0;
//  stimConfigData_t.increasePeriod = 0;
//  stimConfigData_t.plateauPeriod = 6;
//  stimConfigData_t.decreasePeriod = 0;
//  stimConfigData_t.reposePeriod = 0;
//  stimConfigData_t.patternFrequencyA = 50;
//  stimConfigData_t.patternFrequencyB = 70;
//  stimConfigData_t.patternFrequencyC = 50;
  stimConfigData_t.stimOutId = 1;
  stimConfigData_t.curAmplitude = 200 ;

  stimErr1 = StimulationConfig(&stimConfigData_t);
//  if(!stimErr0 && !stimErr1)
//  p0_0 = 1;
    StimulationStart();


#endif

// #if defined(SL_CATALOG_KERNEL_PRESENT)
//   // Start the kernel. Task(s) created in app_init() will start running.
//   sl_system_kernel_start();
// #else // SL_CATALOG_KERNEL_PRESENT
  FsmRun();
//    while (1) {
// //     // Do not remove this call: Silicon Labs components process action routine
// //     // must be called from the super loop.
// //     sl_system_process_action();

// //     // Application process.
//      app_process_action();

// // #if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
// //     // Let the CPU go to sleep if the system allows it.
// //     sl_power_manager_sleep();
// // #endif
// }
// #endif // SL_CATALOG_KERNEL_PRESENT
}



/**********************************************************************************
Function Name:  ApplicationInit
Description:  Task Init function
Parameters:   none
Return value:   none
***********************************************************************************/
void ApplicationInit(void)
{
  //StimulationInit();
  BiofeedbackInit();
}

/**********************************************************************************
End of function ApplicationInit
***********************************************************************************/
