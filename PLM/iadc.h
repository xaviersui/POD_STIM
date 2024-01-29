/*
 * iadc.h
 *
 *  Created on: 13 déc. 2023
 *      Author: ebt
 */

#ifndef APP_IADC_H_
#define APP_IADC_H_
#include "em_iadc.h"
#include "em_cmu.h"
#include "stdint.h"

void initIADC(void);
uint16_t IADC_Read_Current(void);
void IADC_IRQHandler(void);

#endif /* APP_IADC_H_ */
