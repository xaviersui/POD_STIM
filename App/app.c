/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
// #include "em_i2c.h"
// #include "pin_config.h"
// #include "em_cmu.h"
// #include "em_gpio.h"
// #include <string.h>

// //-----------------------------------------------------------------------------
// /* D�finition des fonctions et proc�dures non export�es */
// //-----------------------------------------------------------------------------
// // Fonction de lecture d�un registre du composant DAC
// int32_t SIUE1316_Ad5691r_ReadRegister(uint8_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToRead);
// // Fonction pour lire les registres du composant DAC
// int32_t SIUE1316_Ad5691r_WriteRegister(uint8_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToWrite);

// /*! Gain pour la fonction de transfert de l�intensit� en mA vers la tension du DAC (Vout) */
// #define fGAIN_DAC                                                         0.071875f // = 1.15/16
// /*! Offset pour la fonction de transfert de l�intensit� en mA vers la tension du DAC (Vout) */
// #define fOFFSET_DAC                                                             -31.25f       // = -500/16

// #define I2C_DAC_ADDR  76

// /*! Liste des commande byte pour le composant DAC */
// #define ui8WRITE_INPUT_REGISTER_COMMAND_BYTE                                    0x10
// #define ui8UPDATE_DAC_REGISTER_COMMAND_BYTE                                     0x20
// #define ui8WRITE_DAC_AND_INPUT_REGISTER_COMMAND_BYTE                            0x30
// #define ui8WRITE_CONTROL_REGISTER_COMMAND_BYTE                                  0x40

// /*! Nombre de r��ssai de communication */
// #define iNB_REESSAI_COMMUNICATION                                               3

// void I2C_LeaderRead(uint16_t followerAddress, uint8_t targetAddress, uint8_t *rxBuff, uint8_t numBytes)
// {
//   // Transfer structure
//   I2C_TransferSeq_TypeDef i2cTransfer;
//   I2C_TransferReturn_TypeDef result;

//   // Initialize I2C transfer
//   i2cTransfer.addr          = followerAddress;
//   i2cTransfer.flags         = I2C_FLAG_WRITE_READ; // must write target address before reading
//   i2cTransfer.buf[0].data   = &targetAddress;
//   i2cTransfer.buf[0].len    = 1;
//   i2cTransfer.buf[1].data   = rxBuff;
//   i2cTransfer.buf[1].len    = numBytes;

//   result = I2C_TransferInit(I2C0, &i2cTransfer);

//   // Send data
//   while (result == i2cTransferInProgress) {
//     result = I2C_Transfer(I2C0);
//   }
// }

// /***************************************************************************//**
//  * @brief I2C write numBytes to follower device starting at target address
//  ******************************************************************************/
// void I2C_LeaderWrite(uint16_t followerAddress, uint8_t targetAddress, uint8_t *txBuff, uint8_t numBytes)
// {
//   // Transfer structure
//   I2C_TransferSeq_TypeDef i2cTransfer = {0};
//   I2C_TransferReturn_TypeDef result;
//   uint8_t txBuffer[5] = {0};

//   txBuffer[0] = targetAddress;
//   for(int i = 0; i < numBytes; i++)
//   {
//       txBuffer[i + 1] = txBuff[i];
//   }

//   // Initialize I2C transfer
//   i2cTransfer.addr          = followerAddress;
//   i2cTransfer.flags         = I2C_FLAG_WRITE;
//   i2cTransfer.buf[0].data   = txBuffer;
//   i2cTransfer.buf[0].len    = numBytes + 1;
//   i2cTransfer.buf[1].data   = NULL;
//   i2cTransfer.buf[1].len    = 0;

//   result = I2C_TransferInit(I2C0, &i2cTransfer);

//   // Send data
//   while (result == i2cTransferInProgress) {
//     result = I2C_Transfer(I2C0);
//   }
// }


// typedef union{
//   /*! Valeur 16 bits */
//   uint16_t ui16bit;
//   /*! Tableau de 2 octets */
//   uint8_t u8bit[2];
// } Data16bit_t;

// void app_init(void)
// {
//   CMU_ClockEnable(cmuClock_I2C0, true);
//   CMU_ClockEnable(cmuClock_GPIO, true);

//   // Use default settings
//   I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

//  // Enable GPIO pins PA5 (SDA) and PA6 (SCL)
//  GPIO_PinModeSet(GEN_COURANT_I2C_SCL_PORT, GEN_COURANT_I2C_SCL_PIN, gpioModeWiredAndPullUpFilter, 1);
//  GPIO_PinModeSet(GEN_COURANT_I2C_SDA_PORT, GEN_COURANT_I2C_SDA_PIN, gpioModeWiredAndPullUpFilter, 1);

//  // Route I2C pins to GPIO
//  GPIO->I2CROUTE[0].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE & ~_GPIO_I2C_SDAROUTE_MASK)
//                        | (GEN_COURANT_I2C_SDA_PORT << _GPIO_I2C_SDAROUTE_PORT_SHIFT
//                        | (GEN_COURANT_I2C_SDA_PIN << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
//  GPIO->I2CROUTE[0].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE & ~_GPIO_I2C_SCLROUTE_MASK)
//                        | (GEN_COURANT_I2C_SCL_PORT << _GPIO_I2C_SCLROUTE_PORT_SHIFT
//                        | (GEN_COURANT_I2C_SCL_PIN << _GPIO_I2C_SCLROUTE_PIN_SHIFT));

//  GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;

//  // Initialize the I2C
//  I2C_Init(I2C0, &i2cInit);

//  // Enable automatic STOP on NACK
//  I2C0->CTRL = I2C_CTRL_AUTOSN;

//  Data16bit_t DataBuffer = {0};
//  uint8_t ui8Data = 0;


//  I2C_LeaderWrite(I2C_DAC_ADDR << 1, ui8WRITE_CONTROL_REGISTER_COMMAND_BYTE, DataBuffer.u8bit, 2);

//  //cmd bio 1 et 2 a 1
//  // cs voie 1 et 2 a 1
//  //vo1stimvs1 v01stimvs2 a 1
//  //vo2stimvs1 v02stimvs2 a 1
//  //GPIO_PinOutSet(CS_VOIE1_PORT,CS_VOIE1_PIN);
//  //GPIO_PinOutSet(CS_VOIE2_PORT,CS_VOIE2_PIN);

//  }
// #define CLR_ALL_H_BRIDGE GPIO->P_CLR[CMD_H1_PORT].DOUT = ((1 << CMD_H1_PIN) | (1 << CMD_L2_PIN) | (1 << CMD_H2_PIN) | (1 << CMD_L1_PIN) )
// #define POS_PULSE GPIO->P_SET[CMD_H1_PORT].DOUT = ((1 << CMD_H1_PIN) | (1 << CMD_L2_PIN))
// #define NEG_PULSE GPIO->P_SET[CMD_H1_PORT].DOUT = ((1 << CMD_H2_PIN) | (1 << CMD_L1_PIN))

// /***************************************************************************//**
//  * App ticking function.
//  ******************************************************************************/
// void app_process_action(void)
// {
//   static uint16_t tension = 4; // 192 mV = 3
//   static uint16_t d = 0;
//   static Data16bit_t DataBuffer = {0};

//   d = (tension*4096)/2.5;
//   DataBuffer.u8bit[0] =  (d & 0xff00) >> 8;
//   DataBuffer.u8bit[1] =  (d & 0x00ff);

//   I2C_LeaderWrite(I2C_DAC_ADDR << 1, ui8WRITE_DAC_AND_INPUT_REGISTER_COMMAND_BYTE, DataBuffer.u8bit, 2);
//   CLR_ALL_H_BRIDGE;
//   while(1)
//     {
//       CLR_ALL_H_BRIDGE;
//       POS_PULSE;
//       sl_udelay_wait(5);

//       CLR_ALL_H_BRIDGE;
//       sl_sleeptimer_delay_millisecond(20);

//       CLR_ALL_H_BRIDGE;
//       NEG_PULSE;
//       sl_udelay_wait(5);

//       CLR_ALL_H_BRIDGE;
//       sl_sleeptimer_delay_millisecond(20);

//     }
//   /*while(1)
//     {
//       GPIO_PinOutSet(CMD_H1_PORT, CMD_H1_PIN);
//       //GPIO_PinOutClear(CMD_L1_PORT, CMD_L1_PIN);

//       //GPIO_PinOutClear(CMD_H2_PORT, CMD_H2_PIN);
//       GPIO_PinOutSet(CMD_L2_PORT, CMD_L2_PIN);

//       sl_sleeptimer_delay_millisecond(10);

//       GPIO_PinOutClear(CMD_H1_PORT, CMD_H1_PIN);
//       GPIO_PinOutSet(CMD_L1_PORT, CMD_L1_PIN);

//       GPIO_PinOutClear(CMD_L2_PORT, CMD_L2_PIN);
//       GPIO_PinOutSet(CMD_H2_PORT, CMD_H2_PIN);

//       sl_sleeptimer_delay_millisecond(10);
//       //cmd_h2 cmd_l1
//       //cmd_h1 cmd_l2
//     }*/

// }
