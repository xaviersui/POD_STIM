/*
 * flash.c
 *
 *  Created on: 30 nov. 2023
 *      Author: ebt
 */


/* Librairies FOURNISSEUR */
#include "em_cmu.h"
#include "em_msc.h"
#include "flash.h"
#include "string.h"

void flash_init(void)
{
  MSC_Init();
}

void flash_read_user_data(flash_user_data_t *flashData)
{
  uint32_t *addr = NULL;

  flash_init();
  addr = (uint32_t *)(USERDATA_BASE);
  memcpy(flashData,addr,sizeof(flash_user_data_t));
}
void flash_write_data(flash_user_data_t flashData) // Store new value
{
  uint32_t *userDataAddr = NULL;

  userDataAddr = (uint32_t *)(USERDATA_BASE);

  flash_init();
  MSC_ErasePage(userDataAddr);
  MSC_WriteWord(userDataAddr,&flashData,sizeof(flash_user_data_t));
}
