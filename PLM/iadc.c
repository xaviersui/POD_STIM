/*
 * iadc.c
 *
 *  Created on: 13 déc. 2023
 *      Author: ebt
 */
#include "iadc.h"
uint16_t mes = 0;
/**************************************************************************//**
 * @brief  IADC initialization
 *****************************************************************************/
void initIADC(void)
{
  // Declare initialization structures
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;

  // Single input structure
  IADC_SingleInput_t singleInput = IADC_SINGLEINPUT_DEFAULT;

  /*
   * Enable IADC0 and GPIO register clock.
   *
   * Note: On EFR32xG21 devices, CMU_ClockEnable() calls have no effect
   * as clocks are enabled/disabled on-demand in response to peripheral
   * requests.  Deleting such lines is safe on xG21 devices and will
   * reduce provide a small reduction in code size.
   */
  CMU_ClockEnable(cmuClock_IADC0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Use the FSRC0 as the IADC clock so it can run in EM2
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

  // Shutdown between conversions to reduce current
  init.warmup = iadcWarmupKeepWarm;

  /*
   * Configuration 0 is used by both scan and single conversions by
   * default.  Use internal bandgap as the reference and specify the
   * reference voltage in mV.
   *
   * Resolution is not configurable directly but is based on the
   * selected oversampling ratio (osrHighSpeed), which defaults to
   * 2x and generates 12-bit results.
   */
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx; // Use reference Voltage at 3.3V
    initAllConfigs.configs[0].vRef = 3300;
    initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x; // Oversampling at x2 to achieve 12 Bits résolution
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain1x;   // Anzlog gain x1 to achieve 3.3V full scale
    initAllConfigs.configs[0].digAvg = iadcDigitalAverage1; // Average on 8 samples

    initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,10000000,0,iadcCfgModeNormal,init.srcClkPrescale);

  /*
   * CLK_SRC_ADC must be prescaled by some value greater than 1 to
   * derive the intended CLK_ADC frequency.
   *
   * Based on the default 2x oversampling rate (OSRHS)...
   *
   * conversion time = ((4 * OSRHS) + 2) / fCLK_ADC
   *
   * ...which results in a maximum sampling rate of 833 ksps with the
   * 2-clock input multiplexer switching time is included.
   */

  /*
   * Specify the input channel.  When negInput = iadcNegInputGnd, the
   * conversion is single-ended.
   */
  singleInput.posInput   =  iadcPosInputPadAna0 /*| 1*/;
  singleInput.negInput   = 	iadcNegInputGnd;

  // Allocate the analog bus for ADC0 inputs
  GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD0_ADC0;

  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize a single-channel conversion
  IADC_initSingle(IADC0, &initSingle, &singleInput);
}

uint16_t IADC_Read_Current(void)
{
	static bool IADCInit = false;
	int32_t tmp = 0;
	uint16_t voltage = 0;
	if(!IADCInit)
	{
		initIADC();
		IADCInit = true;
	}

	IADC_command(IADC0, iadcCmdStartSingle);

	// Wait for conversion to be complete
	while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
			| _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively

	tmp = IADC_pullSingleFifoResult(IADC0).data;

	voltage = (uint16_t)(tmp * 3300)/0xFFF;

	return voltage;
}
