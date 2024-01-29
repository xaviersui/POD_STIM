#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// $[CMU]
// [CMU]$

// $[LFXO]
// [LFXO]$

// $[PRS.ASYNCH0]
// [PRS.ASYNCH0]$

// $[PRS.ASYNCH1]
// [PRS.ASYNCH1]$

// $[PRS.ASYNCH2]
// [PRS.ASYNCH2]$

// $[PRS.ASYNCH3]
// [PRS.ASYNCH3]$

// $[PRS.ASYNCH4]
// [PRS.ASYNCH4]$

// $[PRS.ASYNCH5]
// [PRS.ASYNCH5]$

// $[PRS.ASYNCH6]
// [PRS.ASYNCH6]$

// $[PRS.ASYNCH7]
// [PRS.ASYNCH7]$

// $[PRS.ASYNCH8]
// [PRS.ASYNCH8]$

// $[PRS.ASYNCH9]
// [PRS.ASYNCH9]$

// $[PRS.ASYNCH10]
// [PRS.ASYNCH10]$

// $[PRS.ASYNCH11]
// [PRS.ASYNCH11]$

// $[PRS.SYNCH0]
// [PRS.SYNCH0]$

// $[PRS.SYNCH1]
// [PRS.SYNCH1]$

// $[PRS.SYNCH2]
// [PRS.SYNCH2]$

// $[PRS.SYNCH3]
// [PRS.SYNCH3]$

// $[GPIO]
// [GPIO]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// [TIMER1]$

// $[TIMER2]
// [TIMER2]$

// $[TIMER3]
// [TIMER3]$

// $[TIMER4]
// [TIMER4]$

// $[USART0]
// USART0 CLK on PB02
#ifndef USART0_CLK_PORT                         
#define USART0_CLK_PORT                          gpioPortB
#endif
#ifndef USART0_CLK_PIN                          
#define USART0_CLK_PIN                           2
#endif

// USART0 CS on PD03
#ifndef USART0_CS_PORT                          
#define USART0_CS_PORT                           gpioPortD
#endif
#ifndef USART0_CS_PIN                           
#define USART0_CS_PIN                            3
#endif

// USART0 RX on PB01
#ifndef USART0_RX_PORT                          
#define USART0_RX_PORT                           gpioPortB
#endif
#ifndef USART0_RX_PIN                           
#define USART0_RX_PIN                            1
#endif

// USART0 TX on PB03
#ifndef USART0_TX_PORT                          
#define USART0_TX_PORT                           gpioPortB
#endif
#ifndef USART0_TX_PIN                           
#define USART0_TX_PIN                            3
#endif

// [USART0]$

// $[I2C1]
// [I2C1]$

// $[EUSART1]
// [EUSART1]$

// $[EUSART2]
// [EUSART2]$

// $[LCD]
// [LCD]$

// $[KEYSCAN]
// [KEYSCAN]$

// $[LETIMER0]
// [LETIMER0]$

// $[IADC0]
// [IADC0]$

// $[ACMP0]
// [ACMP0]$

// $[ACMP1]
// [ACMP1]$

// $[VDAC0]
// [VDAC0]$

// $[PCNT0]
// [PCNT0]$

// $[LESENSE]
// [LESENSE]$

// $[HFXO0]
// [HFXO0]$

// $[I2C0]
// I2C0 SCL on PA06
#ifndef I2C0_SCL_PORT                           
#define I2C0_SCL_PORT                            gpioPortA
#endif
#ifndef I2C0_SCL_PIN                            
#define I2C0_SCL_PIN                             6
#endif

// I2C0 SDA on PA05
#ifndef I2C0_SDA_PORT                           
#define I2C0_SDA_PORT                            gpioPortA
#endif
#ifndef I2C0_SDA_PIN                            
#define I2C0_SDA_PIN                             5
#endif

// [I2C0]$

// $[EUSART0]
// EUSART0 RX on PA07
#ifndef EUSART0_RX_PORT                         
#define EUSART0_RX_PORT                          gpioPortA
#endif
#ifndef EUSART0_RX_PIN                          
#define EUSART0_RX_PIN                           7
#endif

// EUSART0 TX on PA08
#ifndef EUSART0_TX_PORT                         
#define EUSART0_TX_PORT                          gpioPortA
#endif
#ifndef EUSART0_TX_PIN                          
#define EUSART0_TX_PIN                           8
#endif

// [EUSART0]$

// $[CUSTOM_PIN_NAME]
#ifndef CMD_110V_ON_OFF_PORT                    
#define CMD_110V_ON_OFF_PORT                     gpioPortA
#endif
#ifndef CMD_110V_ON_OFF_PIN                     
#define CMD_110V_ON_OFF_PIN                      0
#endif

#ifndef _PORT                                   
#define _PORT                                    gpioPortA
#endif
#ifndef _PIN                                    
#define _PIN                                     1
#endif

#ifndef SDA_GEN_COURANT_PORT                    
#define SDA_GEN_COURANT_PORT                     gpioPortA
#endif
#ifndef SDA_GEN_COURANT_PIN                     
#define SDA_GEN_COURANT_PIN                      5
#endif

#ifndef SCL_GEN_COURANT_PORT                    
#define SCL_GEN_COURANT_PORT                     gpioPortA
#endif
#ifndef SCL_GEN_COURANT_PIN                     
#define SCL_GEN_COURANT_PIN                      6
#endif

#ifndef COM_RF_UART_RX_PORT                     
#define COM_RF_UART_RX_PORT                      gpioPortA
#endif
#ifndef COM_RF_UART_RX_PIN                      
#define COM_RF_UART_RX_PIN                       7
#endif

#ifndef COM_RF_UART_TX_PORT                     
#define COM_RF_UART_TX_PORT                      gpioPortA
#endif
#ifndef COM_RF_UART_TX_PIN                      
#define COM_RF_UART_TX_PIN                       8
#endif

#ifndef CAN_BIO_SPI_MISO_PORT                   
#define CAN_BIO_SPI_MISO_PORT                    gpioPortB
#endif
#ifndef CAN_BIO_SPI_MISO_PIN                    
#define CAN_BIO_SPI_MISO_PIN                     1
#endif

#ifndef CAN_BIO_SPI_SCLK_PORT                   
#define CAN_BIO_SPI_SCLK_PORT                    gpioPortB
#endif
#ifndef CAN_BIO_SPI_SCLK_PIN                    
#define CAN_BIO_SPI_SCLK_PIN                     2
#endif

#ifndef CAN_BIO_SPI_MOSI_PORT                   
#define CAN_BIO_SPI_MOSI_PORT                    gpioPortB
#endif
#ifndef CAN_BIO_SPI_MOSI_PIN                    
#define CAN_BIO_SPI_MOSI_PIN                     3
#endif

#ifndef MESURE_COURANT_PORT                     
#define MESURE_COURANT_PORT                      gpioPortB
#endif
#ifndef MESURE_COURANT_PIN                      
#define MESURE_COURANT_PIN                       5
#endif

#ifndef IO_RF_STOP_PORT                         
#define IO_RF_STOP_PORT                          gpioPortB
#endif
#ifndef IO_RF_STOP_PIN                          
#define IO_RF_STOP_PIN                           6
#endif

#ifndef SW_DETECT_PORT                          
#define SW_DETECT_PORT                           gpioPortC
#endif
#ifndef SW_DETECT_PIN                           
#define SW_DETECT_PIN                            0
#endif

#ifndef CS_VOIE1_PORT                           
#define CS_VOIE1_PORT                            gpioPortC
#endif
#ifndef CS_VOIE1_PIN                            
#define CS_VOIE1_PIN                             1
#endif

#ifndef CS_VOIE2_PORT                           
#define CS_VOIE2_PORT                            gpioPortC
#endif
#ifndef CS_VOIE2_PIN                            
#define CS_VOIE2_PIN                             2
#endif

#ifndef CMD_GV_P_PORT                           
#define CMD_GV_P_PORT                            gpioPortC
#endif
#ifndef CMD_GV_P_PIN                            
#define CMD_GV_P_PIN                             3
#endif

#ifndef CMD_GV_N_PORT                           
#define CMD_GV_N_PORT                            gpioPortC
#endif
#ifndef CMD_GV_N_PIN                            
#define CMD_GV_N_PIN                             4
#endif

#ifndef CMD_G3_CH1_PORT                         
#define CMD_G3_CH1_PORT                          gpioPortC
#endif
#ifndef CMD_G3_CH1_PIN                          
#define CMD_G3_CH1_PIN                           5
#endif

#ifndef CMD_G2_CH1_PORT                         
#define CMD_G2_CH1_PORT                          gpioPortC
#endif
#ifndef CMD_G2_CH1_PIN                          
#define CMD_G2_CH1_PIN                           6
#endif

#ifndef CMD_G3_CH2_PORT                         
#define CMD_G3_CH2_PORT                          gpioPortC
#endif
#ifndef CMD_G3_CH2_PIN                          
#define CMD_G3_CH2_PIN                           7
#endif

#ifndef CMD_G2_CH2_PORT                         
#define CMD_G2_CH2_PORT                          gpioPortC
#endif
#ifndef CMD_G2_CH2_PIN                          
#define CMD_G2_CH2_PIN                           8
#endif

#ifndef CMD_H2_PORT                             
#define CMD_H2_PORT                              gpioPortD
#endif
#ifndef CMD_H2_PIN                              
#define CMD_H2_PIN                               0
#endif

#ifndef CMD_L1_PORT                             
#define CMD_L1_PORT                              gpioPortD
#endif
#ifndef CMD_L1_PIN                              
#define CMD_L1_PIN                               1
#endif

#ifndef CMD_H1_PORT                             
#define CMD_H1_PORT                              gpioPortD
#endif
#ifndef CMD_H1_PIN                              
#define CMD_H1_PIN                               2
#endif

#ifndef COM_CAN_BIO_SPI_CS_PORT                 
#define COM_CAN_BIO_SPI_CS_PORT                  gpioPortD
#endif
#ifndef COM_CAN_BIO_SPI_CS_PIN                  
#define COM_CAN_BIO_SPI_CS_PIN                   3
#endif

#ifndef CMD_L2_PORT                             
#define CMD_L2_PORT                              gpioPortD
#endif
#ifndef CMD_L2_PIN                              
#define CMD_L2_PIN                               4
#endif

#ifndef CMD_AOP_PORT                            
#define CMD_AOP_PORT                             gpioPortD
#endif
#ifndef CMD_AOP_PIN                             
#define CMD_AOP_PIN                              5
#endif

// [CUSTOM_PIN_NAME]$

#endif // PIN_CONFIG_H

