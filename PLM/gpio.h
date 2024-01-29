/***********************************************************************************
FILE NAME   : Gpio.H
DESCRIPTION : GPIO header file

Copyright   : 2006 Renesas Technology Corporation.
          All Rights Reserved

***********************************************************************************/

/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/


#ifndef GPIO_H_INCLUDED
#define GPIO_H_INCLUDED

/***********************************************************************************
System Includes
***********************************************************************************/

/***********************************************************************************
User Includes
***********************************************************************************/
/* RSKR8C27def.h provides common defines for widely used items. */
#include "rskR8C27def.h"


/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

#define GPIO_SET_PIN_DIR(pin, pinDir)       ( pin = pinDir )
#define GPIO_SET_PORT_DATA(gpioPort, val, mask)   ( gpioPort = (gpioPort & (~mask)) | (mask & val) )

#define GPIO_SET_PIN_DATA(pin, pinData)       ( pin = pinData )

#define GPIO_TOGGLE_PIN(pin)            ( pin = !pin )

#define GPIO_SET_P1_DRIVE_CAP(val, mask)    { p1drr &= ~mask; \
                            p1drr |=  mask & val; }

#define GPIO_SET_P1_PIN_DRIVE_CAP(pin, pinData)   ( pin = pinData )

#define GPIO_DIR_IN                 0
#define GPIO_DIR_OUT                1
#define GPIO_DIR_ALL_IN               0x00
#define GPIO_DIR_ALL_OUT              0xFF

#define GPIO_PULL_UP_DIS              0
#define GPIO_PULL_UP_EN               1
#define GPIO_PULL_UP_ALL_DIS            0x00
#define GPIO_PULL_UP_ALL_EN             0xFF

#define GPIO_P00_2_P03_PULL_UP            BIT0
#define GPIO_P04_2_P07_PULL_UP            BIT1
#define GPIO_P10_2_P13_PULL_UP            BIT2
#define GPIO_P14_2_P17_PULL_UP            BIT3
#define GPIO_P31_N_P33_PULL_UP            BIT6
#define GPIO_P34_2_P37_PULL_UP            BIT7

#define GPIO_P45_PULL_UP              BIT1
#define GPIO_P53_PULL_UP              BIT2
#define GPIO_P54_PULL_UP              BIT3

#define DRV_CAP_ALL_H             0xFF
#define DRV_CAP_ALL_L             0x00
#define DRV_CAP_H             1
#define DRV_CAP_L             0

#define GPIO_P10_DRIVE_CAP              BIT0
#define GPIO_P11_DRIVE_CAP              BIT1
#define GPIO_P12_DRIVE_CAP              BIT2
#define GPIO_P13_DRIVE_CAP              BIT3
#define GPIO_P14_DRIVE_CAP              BIT4
#define GPIO_P15_DRIVE_CAP              BIT5
#define GPIO_P16_DRIVE_CAP              BIT6
#define GPIO_P17_DRIVE_CAP              BIT7


/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/
typedef enum
{
  gGpioPort0_c = 0,
  gGpioPort1_c,
  gGpioPort3_c,
  gGpioPort4_c,
  gGpioPort5_c,
  gGpioPortMax_c
} GpioPort_t;

typedef enum
{
  gGpioDirIn_c = 0,
  gGpioDirOut_c,
  gGpioDirMax_c
} GpioDirection_t;

typedef enum
{
  gGpioPinStateLow_c = 0,
  gGpioPinStateHigh_c,
  gGpioPinStateMax_c,
} GpioPinState_t;

typedef enum
{
  gGpioPullUpReg0_c = 0,
  gGpioPullUpReg1_c,
  gGpioPullUpRegMax_c
} GpioPullUpReg_t;

typedef enum
{
  gGpioErrNoError_c = 0,
  gGpioErrInvalidParameter_c
} GpioErr_t;
#endif    /*  GPIO_H_INCLUDED */
