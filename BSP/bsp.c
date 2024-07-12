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

// #include "stim_management.h"
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
#include "em_emu.h"
#include "em_wdog.h"
#include <CarteStimBio_WdgI.h>
/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define R8C27_STOP_PORT_NB gGpioPort1_c
#define R8C27_STOP_PIN_MASK BIT3
#define R8C27_STOP_PULL_UP_REG gGpioPullUpReg0_c

#define R8C27_STOP_HIGH R8C27_STOP_PIN = gGpioPinStateHigh_c
#define R8C27_STOP_LOW R8C27_STOP_PIN = gGpioPinStateLow_c

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
extern uint8_t gNPulse_c[gStimPatternMax_c];
bool On110V = false;
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
  // EMU_EnterEM1();
  ///////////////////// Configure en GPIO en sortie //////////////////////
  /// Pin used for stimulation
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SW_DETECT_PORT, SW_DETECT_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(CMD_AOP_PORT, CMD_AOP_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(CMD_110V_ON_OFF_PORT, CMD_110V_ON_OFF_PIN, gpioModeWiredOrPullDown, 0);
  GPIO_PinModeSet(CMD_GV_P_PORT, CMD_GV_P_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(CMD_GV_N_PORT, CMD_GV_N_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(CMD_L2_PORT, CMD_L2_PIN, gpioModeWiredOrPullDown, 0);
  GPIO_PinModeSet(CMD_L1_PORT, CMD_L1_PIN, gpioModeWiredOrPullDown, 0);
  GPIO_PinModeSet(CMD_H2_PORT, CMD_H2_PIN, gpioModeWiredOrPullDown, 0);
  GPIO_PinModeSet(CMD_H1_PORT, CMD_H1_PIN, gpioModeWiredOrPullDown, 0);
  GPIO_PinModeSet(CS_VOIE1_PORT, CS_VOIE1_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(CS_VOIE2_PORT, CS_VOIE2_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(ON_OFF_BOOSTER_PORT, ON_OFF_BOOSTER_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(MESURE_COURANT_PORT, MESURE_COURANT_PIN, gpioModeInput, 0);

  /// Pin used for biofeedback
  GPIO_PinModeSet(CMD_G2_CH1_PORT, CMD_G2_CH1_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(CMD_G2_CH2_PORT, CMD_G2_CH2_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(CMD_G3_CH1_PORT, CMD_G3_CH1_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(CMD_G3_CH2_PORT, CMD_G3_CH2_PIN, gpioModePushPull, 0);

  GPIO_PinModeSet(IO_RF_STOP_PORT, IO_RF_STOP_PIN, gpioModeInput, 0);

  GPIO_PinOutClear(CMD_GV_P_PORT, CMD_GV_P_PIN);
  GPIO_PinOutClear(CMD_GV_N_PORT, CMD_GV_N_PIN);

  GPIO->P_CLR[CMD_H1_PORT].DOUT = (1 << CMD_L1_PIN) | (1 << CMD_L2_PIN) | (1 << CMD_H1_PIN) | (1 << CMD_H2_PIN);

  Gpio_ClrAop();
  GPIO->P_CLR[CMD_110V_ON_OFF_PORT].DOUT = (1 << CMD_110V_ON_OFF_PIN);
  GPIO->P_SET[ON_OFF_BOOSTER_PORT].DOUT = (1 << ON_OFF_BOOSTER_PIN);

  On110V = false;

  /// Driver Init
  // Spi is initialised in funGpio_ClrAop();ction "sl_driver_init" locate in s"l_event_handler.h"
  initIADC();
  initTIMER();
  while (EFM32_STOP_IS_EN)
    ;
  SrlCommManagmntInit();
  StimManagementHacheurInit();
  BioManagementInit();
  CarteRf_WdgI_Init(eWDGI_1000MS);
  CarteRf_WdgI_Enable();

  DETECT_RES_CS_DIS;
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
  // spi_master_mode_init();
  BoardInit();
}

/**********************************************************************************
End of function BspInit
***********************************************************************************/
