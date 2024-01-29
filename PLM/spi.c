/***********************************************************************************
FILE NAME   : spi.C
DESCRIPTION : SPI

Copyright   : 2006 Renesas Technology Corporation.
          All Rights Reserved

***********************************************************************************/
/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/

/***********************************************************************************
Compiler Directives
***********************************************************************************/

/***********************************************************************************
System Includes
***********************************************************************************/

/***********************************************************************************
User Includes
***********************************************************************************/
/* sfr_r827.h provides a structure to access all of the device registers. */
#include "spidrv.h"
#include "sl_spidrv_instances.h"
/* RSKR8C27def.h provides common defines for widely used items. */
#include "rskR8C27def.h"
#include "spi.h"

/***********************************************************************************
Macro defines
***********************************************************************************/

/**********************************************************************************
Global variables
***********************************************************************************/
/* Definition of RAM area */

/*******************************************************************************
User Program Code
***********************************************************************************/
/* Declaration of function prototype */
void spi_master_TrData(unsigned char * Data8, unsigned short length);
void spi_master_ReData(unsigned char * Data8, unsigned short length);

/**********************************************************************************
Function Name : spi_master_TrData
Description   : spi master Data Transmission function
Parameters    : Data to transmit,
          Number of Data
Return value  : none
***********************************************************************************/
void spi_master_TrData(unsigned char * Data8, unsigned short length)
{
  unsigned short i;
  int remain = 0;

  for(i=0;i<length;i++)
  {
      SPI_WAIT_TRANSFER_FINISH(remain);
      SPIDRV_MTransmitB(sl_spidrv_usart_CAN_BIO_SPI_handle,Data8 + i,1);
  }

  SPI_WAIT_TRANSFER_FINISH(remain);        /* wait transmission end */
}

/**********************************************************************************
Function Name : spi_master_ReData
Description   : spi master Data Reception function
Parameters    : Data to transmit,
          Number of Data
Return value  : none
***********************************************************************************/
void spi_master_ReData(unsigned char * Data8, unsigned short length)
{

  unsigned short i;
  int remain = 0;
  SPIDRV_MReceiveB(sl_spidrv_usart_CAN_BIO_SPI_handle,Data8,1); /* read a dummy data to start reading operation */

  for(i=0;i<length-1;i++)     /* Maintains receive operation after receiving 1 byte */
  {
      SPI_WAIT_TRANSFER_FINISH(remain);
      SPIDRV_MReceiveB(sl_spidrv_usart_CAN_BIO_SPI_handle, Data8 + i, 1);
  }
  SPI_WAIT_TRANSFER_FINISH(remain);
  SPIDRV_MReceiveB(sl_spidrv_usart_CAN_BIO_SPI_handle, Data8 + i, 1);

}

/***********************************************************************************
End of function Initialise
***********************************************************************************/
