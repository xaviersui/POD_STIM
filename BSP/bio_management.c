/**
* @file   bio_management.c
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
#include "bio_management.h"
#include "spidrv.h"
#include "sl_spidrv_instances.h"
#include "spi.h"
#include "pin_config.h"
#include "sl_sleeptimer.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_gpio.h"


/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define BIO_CMD_G_PORT_NB       gGpioPort0_c

/* ADC AD7323 Management */
/* ADC Chip Select Management */
#define BIO_ADC_CS_PORT_NB        gGpioPort1_c
#define BIO_ADC_CS_PIN_MASK       BIT0

#define BIO_ADC_CS_PIN          p1_0
#define BIO_ADC_CS_DIS          BIO_ADC_CS_PIN = gGpioPinStateHigh_c
#define BIO_ADC_CS_EN         BIO_ADC_CS_PIN = gGpioPinStateLow_c

#define BIO_ADC_SPI_SCLK_EDGE     ODD_EDGE_DL
#define BIO_ADC_SPI_SCLK_POLARITY   POL_HIGH

#define SET_BIO_ADC_SPI_CLK_PHASE   SPI_CLOCK_PHASE(BIO_ADC_SPI_SCLK_EDGE)      /* select clock phase */
#define SET_BIO_ADC_SPI_CLK_POLARITY  SPI_CLOCK_POLARITY(BIO_ADC_SPI_SCLK_POLARITY) /* select clock polarity */

#define CONFIG_BIO_ADC_SPI        { SET_BIO_ADC_SPI_CLK_PHASE;  \
                      SET_BIO_ADC_SPI_CLK_POLARITY; }

#define AD7322

#ifdef  AD7322
#define AD7322_WRITE_ON       BIT15

#define AD7322_RANGE_REG      BIT13
#define AD7322_CTRL_REG       (0*BIT13)

#define AD7322_CODING_STRAIGHT  BIT5
#define AD7322_REF              BIT4
#define AD7322_MODE             ((0*BIT8) | (0*BIT9))
#define AD7322_PM               ((0*BIT6) | (0*BIT7))
#define AD7322_SEQUENCER        (0*BIT3 |0*BIT2)
#define AD7322_SET_CHANNEL(val) (val*BIT10)

#define AD7322_DEFAULT_RANGE  BIT11 | BIT7 /*vin0 et vin5 range +-V*/

#define SET_ANALOG_INPUT_CHANNEL(chan)  (chan * BIT10)
#define CONVERT_SELECTED_CHANNEL(val) AD7322_WRITE_ON | AD7322_CTRL_REG | AD7322_CODING_STRAIGHT | AD7322_REF | AD7322_MODE | AD7322_PM | AD7322_SEQUENCER | SET_ANALOG_INPUT_CHANNEL(val)

#else
#define WRITE_ON            BIT15

#define CONTROL_REGISTER        0
#define RANGE_REGISTER          BIT13
#define SEQUENCE_REGISTER       BIT14 | BIT13

#define ANALOG_INPUT_CHANNEL0     0
#define ANALOG_INPUT_CHANNEL1     BIT10
#define ANALOG_INPUT_CHANNEL2     BIT11
#define ANALOG_INPUT_CHANNEL3     BIT11 | BIT10

#define SET_ANALOG_INPUT_CHANNEL(chan)  (chan * BIT10)

#define DEFAULT_MODE          0     /**< 4 Single-Ended I/Ps. */

#define POWER_MODE_FULL_SHUTDOWN    BIT7 | BIT6
#define POWER_MODE_AUTO_SHUTDOWN    BIT7
#define POWER_MODE_AUTO_STANDBY     BIT6
#define POWER_MODE_NORMAL       0

#define CODING_TWO_COMPLEMENT     0
#define CODING_STRAIGHT_BINARY      BIT5

#define REFERENCE_INTERNAL        BIT4
#define REFERENCE_EXTERNAL        0

#define SEQUENCE_NONE         0
#define SEQUENCE_SELECTED_CHANNEL   BIT2
#define SEQUENCE_CONSECUTIVE_CHANNEL  BIT3

#define SEQUENCE_SELECTED_CHANNEL0    BIT12
#define SEQUENCE_SELECTED_CHANNEL1    BIT11
#define SEQUENCE_SELECTED_CHANNEL2    BIT10
#define SEQUENCE_SELECTED_CHANNEL3    BIT9

#define DEFAULT_RANGE         BIT11 | BIT9 | BIT7 | BIT5    /**< Selects the +/-5 V input range on VIN0, VIN1, VIN2, VIN3. */

#define CONVERT_SELECTED_CHANNEL(val) 0x8030 | SET_ANALOG_INPUT_CHANNEL(val)    // WRITE_ON | CONTROL_REGISTER | SET_ANALOG_INPUT_CHANNEL(val) | DEFAULT_MODE | /*CODING_TWO_COMPLEMENT*/CODING_STRAIGHT_BINARY | REFERENCE_INTERNAL | SEQUENCE_NONE;
#define SIGN_BIT            BIT12
#endif
//#define BIO_SAMPLING_RELOAD_COUNT   tremin = /*250*/ 125 - 1

#define BIO_MEAS_BUFF_SIZE_MAX      /*100*/50

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef int16_t MeasBuff[BIO_MEAS_BUFF_SIZE_MAX];

/** Biofeedback Acquisition Structure. */
typedef struct
{
  MeasBuff  measBuff[gBioInMax_c];
  uint8_t   readIdx;
  uint8_t   writeIdx;
  uint8_t   nAvailableMeas;
  bool_t    bAvailableMeas;
  uint8_t   nBio;
  uint8_t   inputId[gBioInMax_c];

} BioSampling_t;

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static volatile BioSampling_t gBioSampl_t;

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
static void BioManagementGpioInit(void);
static void BioManagementTimerInit(void);
static void BioManagementADCInit(void);

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
const uint8_t gBioPort_c[gBioInMax_c] = {1,0};
#ifdef TEST_MODE_0_BIO
uint8_t nTest = 0;
#endif

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/************************************************************************************
* Name :  BioManagementGpioInit   *//**
* @brief  Description
* @param  .
* @return .
************************************************************************************/
void BioManagementGpioInit(void)
{
  //GPIO_PinModeSet(COM_CAN_BIO_SPI_CS_PORT,COM_CAN_BIO_SPI_CS_PIN,gpioModePushPull, 1);
}

/************************************************************************************
* Name :  BioManagementTimerInit  *//**
* @brief  Sets a sampling frequency at 2500 Hz
* @param  .
* @return .
************************************************************************************/
void BioManagementTimerInit(void)
{
  CMU_ClockEnable(cmuClock_TIMER0,true);
  TIMER_Init_TypeDef TimBio = TIMER_INIT_DEFAULT;
  TimBio.enable = false;
  uint32_t TimClk = CMU_ClockFreqGet(cmuClock_TIMER0);
  TIMER_Init(TIMER0,&TimBio);

  uint32_t TimCnt = TimClk/5860;//5700
  TIMER_TopSet(TIMER0,TimCnt);

  TIMER_IntEnable(TIMER0,TIMER_IF_OF);

  NVIC_ClearPendingIRQ(TIMER0_IRQn);
  NVIC_EnableIRQ(TIMER0_IRQn);

  BIO_SAMPLING_STOP;
}

/**********************************************************************************
End of function
***********************************************************************************/

/********6****************************************************************************
* Name :  BioManagementADCInit  *//**
* @brief  Configure adc AD7323 in sequence selected channel
* @param  .
* @return .
************************************************************************************/
void BioManagementADCInit(void)
{
  uint16_t tmp = 0;
  uint8_t  cmd[2] = {0};
  uint8_t dummyrx[2] = {0};

  /** Write to range register to select the �5V input range for each analog input channel. */
#ifdef AD7322
  tmp = AD7322_WRITE_ON | AD7322_RANGE_REG | AD7322_DEFAULT_RANGE;
#else
  tmp = WRITE_ON | RANGE_REGISTER | DEFAULT_RANGE;
#endif

  cmd[0] = MSB(tmp);
  cmd[1] = LSB(tmp);

  // Configure Range register
  SPIDRV_MTransferB(sl_spidrv_usart_CAN_BIO_SPI_handle, cmd,dummyrx,2);
}

/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
* Name :  BioManagementInit   *//**
* @brief  Description
* @param  .
* @return .
************************************************************************************/
void BioManagementInit(void)
{
  DISABLE_IRQ;

  BioManagementGpioInit();
  BioManagementTimerInit();
  BioManagementADCInit();

  ENABLE_IRQ;
}

/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
* Name :  BioManagementEnableInput  *//**
* @brief  Description
* @param  .
* @return .
************************************************************************************/
BioMngmntErr_t BioManagementEnableInput(uint8_t *inId, uint8_t nInput)
{
  uint8_t i, j;


  DISABLE_IRQ;

  if(nInput > gBioInMax_c)
  {
    BioManagementDisableAllInput();
    return gBioMngmntErrBioMax_c;
  }

  for(i=0;i<nInput;i++)
  {
    if((BioInputId_t)inId[i] >= gBioInMax_c)
    {
      BioManagementDisableAllInput();
      return gBioMngmntErrInIdMax_c;
    }

    gBioSampl_t.inputId[i] = inId[i];

    for(j=0;j<BIO_MEAS_BUFF_SIZE_MAX;j++)
    {
      gBioSampl_t.measBuff[ inId[i] ][ j ] = 0;
    }
  }

  gBioSampl_t.readIdx = 0;
  gBioSampl_t.writeIdx = 0;
  gBioSampl_t.nAvailableMeas = 0;
  gBioSampl_t.bAvailableMeas = FALSE;
  gBioSampl_t.nBio = nInput;

  ENABLE_IRQ;

  return gBioMngmntErrNoError_c;
}

/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
* Name :  BioManagementDisableAllInput  *//**
* @brief  Description
* @param  .
* @return .
************************************************************************************/
void BioManagementDisableAllInput(void)
{
  DISABLE_IRQ;

  gBioSampl_t.nBio = 0;

  ENABLE_IRQ;
}

/**********************************************************************************
End of function
***********************************************************************************/

/************************************************************************************
* Name :  BioManagementGetMeas  *//**
* @brief  Description
* @param  .
* @return .
************************************************************************************/
bool_t BioManagementGetMeas(int16_t* pVoltageMeasurement)
{
  uint8_t i, inputId;
  bool_t  b = FALSE;



  if(gBioSampl_t.nAvailableMeas)
  {
#ifdef PIN_TEST_MODE_0_BIO
    p0_2 = 0; // pin test
#endif

#ifdef TEST_MODE_0_BIO
    if(nTest < gBioSampl_t.nAvailableMeas)
      nTest = 0;

    if(gBioSampl_t.writeIdx == BIO_MEAS_BUFF_SIZE_MAX-1)
      nTest++;

    if(gBioSampl_t.nAvailableMeas >= BIO_MEAS_BUFF_SIZE_MAX)
      *pVoltageMeasurement = 0;
#endif

    for(i=0;i<gBioSampl_t.nBio;i++)
    {
      inputId = gBioSampl_t.inputId[i];
      pVoltageMeasurement[inputId] = gBioSampl_t.measBuff[inputId][gBioSampl_t.readIdx];
    }
    gBioSampl_t.readIdx = (gBioSampl_t.readIdx + 1) % BIO_MEAS_BUFF_SIZE_MAX;

    DISABLE_IRQ;
    gBioSampl_t.nAvailableMeas--;
    ENABLE_IRQ;
    b = TRUE;

#ifdef PIN_TEST_MODE_0_BIO
    p0_2 = 1; // pin test
#endif

  }

  return b;
}

/**********************************************************************************
End of function
***********************************************************************************/


/************************************************************************************
* Name :  BioSamplingISR  *//**
* @brief  Description
* @param  .
* @return .
************************************************************************************/
//void TIMER0_IRQHandler(void)
//{
//  BIO_SAMPLING_STOP;
//
//  uint8_t j = 0;
//  uint8_t rxBuff[2] = {0};
//  uint16_t tmp = 0;
//  uint16_t res = 0;
//  int8_t cmd[2] = {0};
//
//
//#ifdef TEST_MODE_01
//  #define INTVAL0 -4000
//  #define INTVAL1 4000
//  #define INTVAL  1000
//  static int16_t intVal = INTVAL0;
//  static int16_t cnt = 0;
//#endif
//
//  #ifdef PIN_TEST_MODE_0
//  p4_5 = 1; // pin test
//#endif
//
//  if(gBioSampl_t.nAvailableMeas < BIO_MEAS_BUFF_SIZE_MAX)
//  {
//#ifdef TEST_MODE_01
//    /*intVal = intVal + 1;
//    if(intVal > INTVAL1)
//      intVal = INTVAL0; */
//    if(cnt<7958/2)
//      intVal = INTVAL/*50*/;
//    else
//    {
//      if(cnt<7958)
//        intVal = 0;
//      else
//      {
//        intVal = INTVAL/*50*/;
//        cnt = -1;
//      }
//    }
//    cnt++;
//
//#endif
//
//    for(j=0;j<gBioSampl_t.nBio+1;j++)
//    {
//      tmp = CONVERT_SELECTED_CHANNEL(gBioSampl_t.inputId[j % gBioSampl_t.nBio]);
//      cmd[0] = MSB(tmp);
//      cmd[1] = LSB(tmp);
//#if 1
//      SPIDRV_MTransferB(sl_spidrv_usart_CAN_BIO_SPI_handle, cmd,rxBuff,2);
//#else
//      SPIDRV_MTransmitB(sl_spidrv_usart_CAN_BIO_SPI_handle,cmd,1);
//      SPIDRV_MReceiveB(sl_spidrv_usart_CAN_BIO_SPI_handle,rxBuff,1);
//
//      SPIDRV_MTransmitB(sl_spidrv_usart_CAN_BIO_SPI_handle,cmd + 1,1);
//      SPIDRV_MReceiveB(sl_spidrv_usart_CAN_BIO_SPI_handle,rxBuff + 1,1);
//#endif
//      if(j > 0)
//      {
//          res = ( ((uint16_t)rxBuff[0] << 8) | rxBuff[1] );   /**< DAC Data */
//#ifdef TEST_MODE_01
//        gBioSampl_t.measBuff[gBioSampl_t.inputId[j-1]][gBioSampl_t.writeIdx] = intVal;
//#else
//        gBioSampl_t.measBuff[gBioSampl_t.inputId[j-1]][gBioSampl_t.writeIdx] = (int16_t)((((res & 0x1fff)*10000)/0x1fff) - 5000);
//#endif
//      }
//    }
//
//#ifdef TEST_MODE
//    if(gBioSampl_t.writeIdx == BIO_MEAS_BUFF_SIZE_MAX-1)
//      gBioSampl_t.writeIdx++;
//#endif
//    gBioSampl_t.writeIdx = (gBioSampl_t.writeIdx + 1) % BIO_MEAS_BUFF_SIZE_MAX;
//    gBioSampl_t.nAvailableMeas++;
//    gBioSampl_t.bAvailableMeas = TRUE;
//  }
//
//#ifdef PIN_TEST_MODE_0
//  p4_5 = 0; // pin test
//#endif
//  TIMER_IntClear(TIMER0,TIMER_IF_OF);
//  BIO_SAMPLING_START;
//}

/**********************************************************************************
End of function
***********************************************************************************/
