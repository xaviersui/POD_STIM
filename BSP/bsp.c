/**
* @file   bsp.c
* @ingroup  GrpBSP
* @brief  Bio Management drivers.
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
#include "bsp.h"

//#include "stim_management.h"
#include "bio_management.h"
#include "srl_comm_management.h"
#include "stim_management.h"
#include "spi.h"
#include "uart.h"
#include "pin_config.h"
#include "iadc.h"
#include "i2c.h"
#include "timer.h"
#include "siue.h"
//#include "Flash.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define R8C27_STOP_PORT_NB        gGpioPort1_c
#define R8C27_STOP_PIN_MASK       BIT3
#define R8C27_STOP_PULL_UP_REG      gGpioPullUpReg0_c

#define R8C27_STOP_HIGH         R8C27_STOP_PIN = gGpioPinStateHigh_c
#define R8C27_STOP_LOW          R8C27_STOP_PIN = gGpioPinStateLow_c

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
static void BoardInit(void);

extern void (*pStimGenCallback[gStimPatternMax_c])(void);
extern uint8_t  gNPulse_c[gStimPatternMax_c];
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


/**********************************************************************************
Function Name:  BoardInit
Description:  Board Init function
Parameters:   none
Return value:   none
***********************************************************************************/
void BoardInit(void)
{
  ///////////////////// Configure en GPIO en sortie //////////////////////
    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinModeSet(SW_DETECT_PORT, SW_DETECT_PIN, gpioModeWiredOrPullDown, 0);
    GPIO_PinModeSet(CMD_AOP_PORT,CMD_AOP_PIN, gpioModeWiredOrPullDown, 0);
    GPIO_PinModeSet(CMD_110V_ON_OFF_PORT, CMD_110V_ON_OFF_PIN, gpioModeWiredOrPullDown, 0);
    GPIO_PinModeSet(CMD_GV_P_PORT, CMD_GV_P_PIN, gpioModeWiredOrPullDown, 0);
    GPIO_PinModeSet(CMD_GV_N_PORT, CMD_GV_N_PIN, gpioModeWiredOrPullDown, 0);
    GPIO_PinModeSet(CMD_L2_PORT, CMD_L2_PIN, gpioModeWiredOrPullDown, 0);
    GPIO_PinModeSet(CMD_L1_PORT, CMD_L1_PIN, gpioModeWiredOrPullDown, 0);
    GPIO_PinModeSet(CMD_H2_PORT, CMD_H2_PIN, gpioModeWiredOrPullDown, 0);
    GPIO_PinModeSet(CMD_H1_PORT, CMD_H1_PIN, gpioModeWiredOrPullDown, 0);

    ////////////// Enable GPIO pins PA5 (SDA) and PA6 (SCL) ///////////////
    CMU_ClockEnable(cmuClock_I2C0, true);
    GPIO_PinModeSet(I2C0_SCL_PORT, I2C0_SCL_PIN, gpioModeWiredAndPullUpFilter, 1);
    GPIO_PinModeSet(I2C0_SDA_PORT, I2C0_SDA_PIN, gpioModeWiredAndPullUpFilter, 1);

    GPIO_PinModeSet(IO_RF_STOP_PORT, IO_RF_STOP_PIN, gpioModeInput, 0);

    initIADC();
      init_I2C();
      initTIMER();

      /** - Callback functions declaration. */
        pStimGenCallback[gStimPatternBiphasic_c] = MeSS_GestionCourantBiphasiquePositif;
        pStimGenCallback[gStimPatternMonophasic_c] = ImpulsMonophas;
        //pStimGenCallback[gStimPatternGalvanic_c] = Galvanic;
        pStimGenCallback[gStimPatternBiphasicAltern_c] = MeSS_GestionCourantBiphasiqueAlterne;
        //pStimGenCallback[gStimPatternVeineuxBiphasic_c] = VeineuxBiphas;
        //pStimGenCallback[gStimPatternNeuro_c] = NeuroMonophas;
        pStimGenCallback[gStimPatternBiphasicNegative_c] = MeSS_GestionCourantBiphasiqueNegatif;

        /** - Number of alternance per signal. */
        gNPulse_c[gStimPatternBiphasic_c] = 2;
        gNPulse_c[gStimPatternMonophasic_c] = 2;
        gNPulse_c[gStimPatternBiphasicAltern_c] = 2;
        gNPulse_c[gStimPatternVeineuxBiphasic_c] = 52;
        gNPulse_c[gStimPatternBiphasicNegative_c] = 2;
  while(EFM32_STOP_IS_EN);
  SrlCommManagmntInit();
  //StimManagementHacheurInit();
  //BioManagementInit();
  //error = flash_Init(BLOCK_DEF);
}

/**********************************************************************************
End of function BoardInit
***********************************************************************************/

/**********************************************************************************
Function Name:  BspInit
Description:  Bsp Init function
Parameters:   none
Return value:   none
***********************************************************************************/
void BspInit(void)
{
  //spi_master_mode_init();
  BoardInit();

}

/**********************************************************************************
End of function BspInit
***********************************************************************************/

