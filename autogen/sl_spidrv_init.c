#include "spidrv.h"
#include "sl_spidrv_instances.h"
#include "sl_assert.h"


#include "sl_spidrv_usart_CAN_BIO_SPI_config.h"

SPIDRV_HandleData_t sl_spidrv_usart_CAN_BIO_SPI_handle_data;
SPIDRV_Handle_t sl_spidrv_usart_CAN_BIO_SPI_handle = &sl_spidrv_usart_CAN_BIO_SPI_handle_data;

SPIDRV_Init_t sl_spidrv_usart_init_CAN_BIO_SPI = {
  .port = SL_SPIDRV_USART_CAN_BIO_SPI_PERIPHERAL,
#if defined(_USART_ROUTELOC0_MASK)
  .portLocationTx = SL_SPIDRV_USART_CAN_BIO_SPI_TX_LOC,
  .portLocationRx = SL_SPIDRV_USART_CAN_BIO_SPI_RX_LOC,
  .portLocationClk = SL_SPIDRV_USART_CAN_BIO_SPI_CLK_LOC,
#if defined(SL_SPIDRV_USART_CAN_BIO_SPI_CS_LOC)
  .portLocationCs = SL_SPIDRV_USART_CAN_BIO_SPI_CS_LOC,
#endif
#elif defined(_GPIO_USART_ROUTEEN_MASK)
  .portTx = SL_SPIDRV_USART_CAN_BIO_SPI_TX_PORT,
  .portRx = SL_SPIDRV_USART_CAN_BIO_SPI_RX_PORT,
  .portClk = SL_SPIDRV_USART_CAN_BIO_SPI_CLK_PORT,
#if defined(SL_SPIDRV_USART_CAN_BIO_SPI_CS_PORT)
  .portCs = SL_SPIDRV_USART_CAN_BIO_SPI_CS_PORT,
#endif
  .pinTx = SL_SPIDRV_USART_CAN_BIO_SPI_TX_PIN,
  .pinRx = SL_SPIDRV_USART_CAN_BIO_SPI_RX_PIN,
  .pinClk = SL_SPIDRV_USART_CAN_BIO_SPI_CLK_PIN,
#if defined(SL_SPIDRV_USART_CAN_BIO_SPI_CS_PIN)
  .pinCs = SL_SPIDRV_USART_CAN_BIO_SPI_CS_PIN,
#endif
#else
  .portLocation = SL_SPIDRV_USART_CAN_BIO_SPI_ROUTE_LOC,
#endif
  .bitRate = SL_SPIDRV_USART_CAN_BIO_SPI_BITRATE,
  .frameLength = SL_SPIDRV_USART_CAN_BIO_SPI_FRAME_LENGTH,
  .dummyTxValue = 0,
  .type = SL_SPIDRV_USART_CAN_BIO_SPI_TYPE,
  .bitOrder = SL_SPIDRV_USART_CAN_BIO_SPI_BIT_ORDER,
  .clockMode = SL_SPIDRV_USART_CAN_BIO_SPI_CLOCK_MODE,
  .csControl = SL_SPIDRV_USART_CAN_BIO_SPI_CS_CONTROL,
  .slaveStartMode = SL_SPIDRV_USART_CAN_BIO_SPI_SLAVE_START_MODE,
};

void sl_spidrv_init_instances(void) {
#if !defined(SL_SPIDRV_USART_CAN_BIO_SPI_CS_PIN)
  EFM_ASSERT(sl_spidrv_usart_init_CAN_BIO_SPI.csControl == spidrvCsControlApplication);
#endif
  SPIDRV_Init(sl_spidrv_usart_CAN_BIO_SPI_handle, &sl_spidrv_usart_init_CAN_BIO_SPI);
}
