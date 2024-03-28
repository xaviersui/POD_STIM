/*
 * i2c.h
 *
 *  Created on: 13 déc. 2023
 *      Author: ebt
 */

#ifndef APP_I2C_H_
#define APP_I2C_H_
#include "em_i2c.h"
#include "em_cmu.h"
#include "stdint.h"
#include "pin_config.h"

void init_I2C();

void I2C_LeaderRead(uint16_t followerAddress, uint8_t targetAddress, uint8_t *rxBuff, uint8_t numBytes);
I2C_TransferReturn_TypeDef I2C_LeaderWrite(uint16_t followerAddress, uint8_t targetAddress, uint8_t *txBuff, uint8_t numBytes);


/*! Gain pour la fonction de transfert de l�intensit� en mA vers la tension du DAC (Vout) */
#define fGAIN_DAC 0.04259259f // 0.04259259f= 1.15/27 // 0.15625f = 2.25/27
/*! Offset pour la fonction de transfert de l�intensit� en mA vers la tension du DAC (Vout) */
#define fOFFSET_DAC -18.51f// = -500/27= -31.25f <-- pour 50mA  // 500/16 = -62.5f <-- pour 100mA

#define I2C_DAC_ADDR 76

/*! Liste des commande byte pour le composant DAC */
#define ui8WRITE_INPUT_REGISTER_COMMAND_BYTE 0x10
#define ui8UPDATE_DAC_REGISTER_COMMAND_BYTE 0x20
#define ui8WRITE_DAC_AND_INPUT_REGISTER_COMMAND_BYTE 0x30
#define ui8WRITE_CONTROL_REGISTER_COMMAND_BYTE 0x40

/*! Nombre de r��ssai de communication */
#define iNB_REESSAI_COMMUNICATION 3

#endif /* APP_I2C_H_ */
