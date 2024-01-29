/**
* @file   Bio_Management.h
* @ingroup  GrpBSP
* @brief  Bio Management header file.
*
* Copyright : 2009 Vivaltis.\n
*         All Rights Reserved
*/

/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/


#ifndef BIO_MANAGEMENT_H_INCLUDED
#define BIO_MANAGEMENT_H_INCLUDED

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
#include "em_timer.h"
#include "pin_config.h"
#include "em_gpio.h"
//#include "timer.h"
//#include "gpio.h"


/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/
/* CMD_Mi Management */

#define BIO_CMD_G_0_80uV    0x03
#define BIO_CMD_G_0_400uV   0x01
#define BIO_CMD_G_0_2000uV    0x02

#define BIO_CMD_G_PORT          p0
#define BIO_CMD_G_PIN_MASK(BIO_PORT)  (0x03 << (BIO_PORT*2))

#define BIO_PORT_0        1
#define BIO_PORT_1        0

#define BIO_CMD_SET_G1                               \
{                                                    \
      GPIO_PinOutSet(CMD_G2_CH1_PORT,CMD_G2_CH1_PIN);\
      GPIO_PinOutSet(CMD_G2_CH2_PORT,CMD_G2_CH2_PIN);\
      GPIO_PinOutSet(CMD_G3_CH1_PORT,CMD_G3_CH1_PIN);\
      GPIO_PinOutSet(CMD_G3_CH2_PORT,CMD_G3_CH2_PIN);\
}                                                    \

#define BIO_CMD_SET_G2                                 \
{                                                      \
      GPIO_PinOutClear(CMD_G2_CH1_PORT,CMD_G2_CH1_PIN);\
      GPIO_PinOutClear(CMD_G2_CH2_PORT,CMD_G2_CH2_PIN);\
      GPIO_PinOutSet(CMD_G3_CH1_PORT,CMD_G3_CH1_PIN);  \
      GPIO_PinOutSet(CMD_G3_CH2_PORT,CMD_G3_CH2_PIN);  \
}                                                      \

#define BIO_CMD_SET_G3                                 \
{                                                      \
      GPIO_PinOutSet(CMD_G2_CH1_PORT,CMD_G2_CH1_PIN);  \
      GPIO_PinOutSet(CMD_G2_CH2_PORT,CMD_G2_CH2_PIN);  \
      GPIO_PinOutClear(CMD_G3_CH1_PORT,CMD_G3_CH1_PIN);\
      GPIO_PinOutClear(CMD_G3_CH2_PORT,CMD_G3_CH2_PIN);\
}                                                      \
/* ADC Chip Select Management */

/* Bio Sampling Management */
#define BIO_SAMPLING_START        {TIMER_CounterSet(TIMER0,0);TIMER_Enable(TIMER0,true);}
#define BIO_SAMPLING_STOP         {TIMER_Enable(TIMER0,false);TIMER_CounterSet(TIMER0,0);}
#define BIO_SAMPLING_FREQUENCY      2500        /**< in Hz */ //2500
#define BIO_SAMPLING_FREQUENCY_100HZ 25          /**< in 100Hz */ //25
#define BIO_SAMPLING_FREQUENCY_kHZ    2.5      /**< in kHz */ //2.5

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

/** Biofeedback Management Error Id */
typedef enum
{
  gBioMngmntErrNoError_c = 0,
  gBioMngmntErrBioMax_c,
  gBioMngmntErrInIdMax_c,
  gBioMngmntErrMax_c

} BioMngmntErr_t;

/** Biofeedback in Id */
typedef enum
{
  gBioIn0_c = 0,
  gBioIn1_c,
  gBioInMax_c

} BioInputId_t;

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
extern const uint8_t gBioPort_c[gBioInMax_c];


/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/
void BioManagementInit(void);
bool_t BioManagementGetMeas(int16_t* pVoltageMeasurement);
BioMngmntErr_t BioManagementEnableInput(uint8_t *inId, uint8_t nInput);
void BioManagementDisableAllInput(void);

#endif    /*  BIO_MANAGEMENT_H_INCLUDED */
