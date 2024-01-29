/***********************************************************************************
FILE NAME   : RSKR8C27def.h
DESCRIPTION : RSK specific definitions

Copyright   : 2006 Renesas Technology Corporation.
          All Rights Reserved

***********************************************************************************/

/***********************************************************************************
Revision History
DD.MM.YYYY OSO-UID Description
07.07.2006 RSO-PIS First Release
***********************************************************************************/

#ifndef RSKR8C27DEF_H
#define RSKR8C27DEF_H

/* Peripheral Clock Speed set up in ConfigureOperatingFrequency() */
#define f1_CLK_SPEED    20000000
#define CPU_CLK_SOURCE    f1_CLK_SPEED
#define XIN_FREQ      f1_CLK_SPEED

#define f1_CLK_SPEED_MHZ  20
#define f1_CLK_SPEED_10MHZ  2

#define ENDIANESS

//#define OLD_VERSION_BIO
//#define TEST_MODE

//#define TEST_MODE_0
//#define TEST_MODE_01
//#define TEST_MODE_0_BIO
//#define PIN_TEST_MODE_0
//#define PIN_TEST_MODE_0_BIO
//#define PIN_TEST_MODE_01

//#define TEST_MODE_1
//#define PIN_TEST_MODE_1
//#define TEST_MODE_1_BIO1
//#define TEST_MODE_1_BIO2
//#define TEST_MODE_1_BIO3
//#define PIN_TEST_MODE_1_BIO
//#define TEST_MODE_1_SRL0
//#define TEST_MODE_1_BIO_OFFSET

//#define TEST_MODE_2
//#define TEST_MODE_21
//#define TEST_MODE_2_BIO1
//#define TEST_MODE_2_BIO2
//#define TEST_MODE_2_STIM01
//#define TEST_MODE_2_BIO_OFFSET


//#define PIN_TEST_MODE_2
//#define PIN_TEST_MODE_2_BIO1


//#define TEST_MODE_2_MANAGEMENT
//#define TEST_MODE_1_MANAGEMENT

#define new
#define NEW_TRANSISTOR
#define V016

//#define SET_VERSION_ID
#ifdef SET_VERSION_ID
  #define TEST_MODE_2_MANAGEMENT
  #define TEST_MODE_1_MANAGEMENT
#endif

//#define GET_VERSION_ID
#ifdef GET_VERSION_ID
  #define TEST_MODE_2_MANAGEMENT
  #define TEST_MODE_1_MANAGEMENT
#endif

/* Switches */
#define SW1       p1_7
#define SW2       p3_3
#define SW3       p1_0
#define SW1_DDR     pd1_7
#define SW2_DDR     pd3_3
#define SW3_DDR     pd1_0

/* LEDs */
#define LED0      p0_0
#define LED1      p0_1
#define LED2      p0_2
#define LED3      p0_3

#define LED0_DDR    pd0_0
#define LED1_DDR    pd0_1
#define LED2_DDR    pd0_2
#define LED3_DDR    pd0_3

/* LED port settings */
#define LED_PORT_DR   p0    /* LED Port data register */
#define LED_BIT     (0x0F)      /* Port bit to toggle for flashing LED */

#define LED_ON      0
#define LED_OFF     1
#define LEDS_ON     (0x00)
#define LEDS_OFF    (0x0F)
#define SET_BIT_HIGH  (1)
#define SET_BIT_LOW   (0)
#define SET_BYTE_HIGH (0xFF)
#define SET_BYTE_LOW  (0x00)


#define ENABLE_IRQ    __enable_irq()
#define DISABLE_IRQ   __disable_irq()

/************************************************************************************
*
*       TYPE DEFINITIONS
*
************************************************************************************/

typedef signed    char  int8_t;
typedef unsigned  char  uint8_t;
typedef signed    short int16_t;
typedef unsigned  short uint16_t;
typedef signed    long  int32_t;
typedef unsigned  long  uint32_t;
typedef signed    long long int64_t;
typedef unsigned  long long uint64_t;

typedef signed    char  intn8_t;
typedef unsigned  char  uintn8_t;
typedef signed    short intn16_t;
typedef unsigned  short uintn16_t;
typedef signed    long  intn32_t;
typedef unsigned  long  uintn32_t;
typedef signed    long long intn64_t;
typedef unsigned  long long uintn64_t;

/* boolean types */
typedef uint8_t   bool_t;
typedef uintn8_t  booln_t;
/* used for indexing into an array in the most efficient manner for the platform */
typedef uint8_t   index_t;

typedef void(*callback_t)(void);

#define BIT0    0x00000001UL
#define BIT1    0x00000002UL
#define BIT2    0x00000004UL
#define BIT3    0x00000008UL
#define BIT4    0x00000010UL
#define BIT5    0x00000020UL
#define BIT6    0x00000040UL
#define BIT7    0x00000080UL
#define BIT8    0x00000100UL
#define BIT9    0x00000200UL
#define BIT10   0x00000400UL
#define BIT11   0x00000800UL
#define BIT12   0x00001000UL
#define BIT13   0x00002000UL
#define BIT14   0x00004000UL
#define BIT15   0x00008000UL
#define BIT16   0x00010000UL
#define BIT17   0x00020000UL
#define BIT18   0x00040000UL
#define BIT19   0x00080000UL
#define BIT20   0x00100000UL
#define BIT21   0x00200000UL
#define BIT22   0x00400000UL
#define BIT23   0x00800000UL
#define BIT24   0x01000000UL
#define BIT25   0x02000000UL
#define BIT26   0x04000000UL
#define BIT27   0x08000000UL
#define BIT28   0x10000000UL
#define BIT29   0x20000000UL
#define BIT30   0x40000000UL
#define BIT31   0x80000000UL

                                                                /* ------------------- OCTET DEFINES ------------------ */
#define  OCTET_NBR_BITS                                8
#define  OCTET_MASK                                 0xFF

#define  NIBBLE_NBR_BITS                               4
#define  NIBBLE_MASK                                0x0F

                                                                /* ------------------ INTEGER DEFINES ----------------- */
#define  INT_08_NBR_BITS                               8
#define  INT_08_MASK                                0xFF

#define  INT_08U_MIN_VAL                               0u
#define  INT_08U_MAX_VAL                             255u

#define  INT_08S_MIN_VAL                            -128
#define  INT_08S_MAX_VAL                             127

#define  INT_08S_MIN_VAL_ONES_CPL                   -127
#define  INT_08S_MAX_VAL_ONES_CPL                    127


#define  INT_16_NBR_BITS                              16
#define  INT_16_MASK                              0xFFFF

#define  INT_16U_MIN_VAL                               0u
#define  INT_16U_MAX_VAL                           65535u

#define  INT_16S_MIN_VAL                          -32768
#define  INT_16S_MAX_VAL                           32767

#define  INT_16S_MIN_VAL_ONES_CPL                 -32767
#define  INT_16S_MAX_VAL_ONES_CPL                  32767


#define  INT_32_NBR_BITS                              32
#define  INT_32_MASK                          0xFFFFFFFF

#define  INT_32U_MIN_VAL                               0u
#define  INT_32U_MAX_VAL                      4294967295u

#define  INT_32S_MIN_VAL                     -2147483648
#define  INT_32S_MAX_VAL                      2147483647

#define  INT_32S_MIN_VAL_ONES_CPL            -2147483647
#define  INT_32S_MAX_VAL_ONES_CPL             2147483647


#define  INT_64_NBR_BITS                              64
#define  INT_64_MASK                  0xFFFFFFFFFFFFFFFF

#define  INT_64U_MIN_VAL                               0u
#define  INT_64U_MAX_VAL            18446744073709551615u

#define  INT_64S_MIN_VAL            -9223372036854775808
#define  INT_64S_MAX_VAL             9223372036854775807

#define  INT_64S_MIN_VAL_ONES_CPL   -9223372036854775807
#define  INT_64S_MAX_VAL_ONES_CPL    9223372036854775807

/* Common Defines */
#ifndef TRUE
#define TRUE      1
#endif
#ifndef FALSE
#define FALSE     0
#endif

#define  DEF_DISABLED                                      0
#define  DEF_ENABLED                                       1

#define  DEF_FALSE                                         0
#define  DEF_TRUE                                          1

#define  DEF_NO                                            0
#define  DEF_YES                                           1

#define  DEF_OFF                                           0
#define  DEF_ON                                            1

#define  DEF_CLR                                           0
#define  DEF_SET                                           1

#define  DEF_ACTIVE                                        0
#define  DEF_INACTIVE                                      1

#define  DEF_FAIL                                          0
#define  DEF_OK                                            1

#ifndef NULL
#define NULL (( void * )( 0 ))
#endif

/* ------------------- TIME DEFINES ------------------- */
#define  DEF_TIME_NBR_HR_PER_DAY                          24uL

#define  DEF_TIME_NBR_MIN_PER_HR                          60uL
#define  DEF_TIME_NBR_MIN_PER_DAY                       (DEF_TIME_NBR_MIN_PER_HR  * DEF_TIME_NBR_HR_PER_DAY)

#define  DEF_TIME_NBR_SEC_PER_MIN                         60uL
#define  DEF_TIME_NBR_SEC_PER_HR                        (DEF_TIME_NBR_SEC_PER_MIN * DEF_TIME_NBR_MIN_PER_HR)
#define  DEF_TIME_NBR_SEC_PER_DAY                       (DEF_TIME_NBR_SEC_PER_HR  * DEF_TIME_NBR_HR_PER_DAY)

#define  DEF_TIME_NBR_mS_PER_SEC          1000uL
#define  DEF_TIME_NBR_uS_PER_SEC          1000000uL
#define  DEF_TIME_NBR_100nS_PER_SEC         10000000uL
#define  DEF_TIME_NBR_nS_PER_SEC          1000000000uL

#define  DEF_TIME_NBR_nS_PER_uS           1000uL

typedef uint32_t zbClock32_t;
typedef uint32_t zbClock24_t;
typedef uint16_t zbClock16_t;

/* common macros to reduce code size in S08 */
#define IsEqual2Bytes(aVal1, aVal2) (*((uint16_t *)(aVal1)) == *((uint16_t *)(aVal2)))
#define Copy2Bytes(aVal1, aVal2)    (*((uint16_t *)(aVal1)) = *((uint16_t *)(aVal2)))
#define Set2Bytes(aVal1, iVal2)     (*((uint16_t *)(aVal1)) = (iVal2))
#define IsEqual2BytesInt(aVal1, iVal2) (*((uint16_t *)(aVal1)) == (iVal2))
#define TwoBytesToUint16(aVal)      ( *((uint16_t *)(aVal)) )
#define TwoBytes2Byte(aVal)         ( (uint8_t)( *( ( uint16_t * )(aVal) ) ) )
#define Cmp2BytesToZero(aVal)       (!( TwoBytesToUint16(aVal) ))
#define Inc4Bytes(aVal1, iVal2)     ( *((uint32_t *)(aVal1)) += (iVal2))

/*
*********************************************************************************************************
*                                             BIT MACRO'S
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              DEF_BIT()
*
* Description : Create bit mask with single, specified bit set.
*
* Argument(s) : bit         Bit number of bit to set.
*
* Return(s)   : Bit mask with single, specified bit set.
*
* Caller(s)   : various.
*
* Note(s)     : (1) 'bit' values that overflow the target CPU &/or compiler environment (e.g. negative
*                   or greater-than-CPU-data-size values) MAY generate compiler warnings &/or errors.
*********************************************************************************************************
*/

#define  DEF_BIT(bit)                            (1u << (bit))


/*$PAGE*/
/*
*********************************************************************************************************
*                                           DEF_BIT_MASK()
*
* Description : Shift a bit mask.
*
* Argument(s) : bit_mask    Bit mask to shift.
*
*               bit_shift   Number of bit positions to left-shift bit mask.
*
* Return(s)   : Shifted bit mask.
*
* Caller(s)   : various.
*
* Note(s)     : (1) 'bit_shift' values that overflow the target CPU &/or compiler environment (e.g. negative
*                   or greater-than-CPU-data-size values) MAY generate compiler warnings &/or errors.
*********************************************************************************************************
*/

#define  DEF_BIT_MASK(bit_mask, bit_shift)             ((bit_mask)       << (bit_shift))


/*
*********************************************************************************************************
*                                           DEF_BIT_FIELD()
*
* Description : Create & shift a contiguous bit field.
*
* Argument(s) : bit_field   Number of contiguous bits to set in the bit field.
*
*               bit_shift   Number of bit positions   to left-shift bit field.
*
* Return(s)   : Shifted bit field.
*
* Caller(s)   : various.
*
* Note(s)     : (1) 'bit_field'/'bit_shift' values that overflow the target CPU &/or compiler environment
*                   (e.g. negative or greater-than-CPU-data-size values) MAY generate compiler warnings
*                   &/or errors.
*********************************************************************************************************
*/

#define  DEF_BIT_FIELD(bit_field, bit_shift)         ((((bit_field) >= DEF_INT_CPU_NBR_BITS) ? (DEF_INT_CPU_U_MAX_VAL)   \
                                                                                             : (DEF_BIT(bit_field) - 1)) \
                                                                                                    << (bit_shift))


/*$PAGE*/
/*
*********************************************************************************************************
*                                            DEF_BIT_SET()
*
* Description : Set specified bit(s) in a value.
*
* Argument(s) : val         Value to modify by setting specified bit(s).
*
*               mask        Mask of bits to set.
*
* Return(s)   : Modified value with specified bit(s) set.
*
* Caller(s)   : various.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_SET(val, mask)                       ( (val) |=  (mask); )


/*
*********************************************************************************************************
*                                            DEF_BIT_CLR()
*
* Description : Clear specified bit(s) in a value.
*
* Argument(s) : val         Value to modify by clearing specified bit(s).
*
*               mask        Mask of bits to clear.
*
* Return(s)   : Modified value with specified bit(s) clear.
*
* Caller(s)   : various.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_CLR(val, mask)                       ( (val) &= ~(mask); )


/*$PAGE*/
/*
*********************************************************************************************************
*                                          DEF_BIT_IS_SET()
*
* Description : Determine if specified bit(s) in a value are set.
*
* Argument(s) : val         Value to check for specified bit(s) set.
*
*               mask        Mask of bits to check if set.
*
* Return(s)   : DEF_YES, if ALL specified bit(s) are     set in value.
*
*               DEF_NO,  if ALL specified bit(s) are NOT set in value.
*
* Caller(s)   : various.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_IS_SET(val, mask)                   ((((val) & (mask)) == (mask)) ? (DEF_YES) : (DEF_NO ))


/*
*********************************************************************************************************
*                                          DEF_BIT_IS_CLR()
*
* Description : Determine if specified bit(s) in a value are clear.
*
* Argument(s) : val         Value to check for specified bit(s) clear.
*
*               mask        Mask of bits to check if clear.
*
* Return(s)   : DEF_YES, if ALL specified bit(s) are     clear in value.
*
*               DEF_NO,  if ALL specified bit(s) are NOT clear in value.
*
* Caller(s)   : various.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_IS_CLR(val, mask)                    (((val) & (mask))            ? (DEF_NO ) : (DEF_YES))


/*$PAGE*/
/*
*********************************************************************************************************
*                                        DEF_BIT_IS_SET_ANY()
*
* Description : Determine if any specified bit(s) in a value are set.
*
* Argument(s) : val         Value to check for specified bit(s) set.
*
*               mask        Mask of bits to check if set.
*
* Return(s)   : DEF_YES, if ANY specified bit(s) are     set in value.
*
*               DEF_NO,  if ALL specified bit(s) are NOT set in value.
*
* Caller(s)   : various.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_IS_SET_ANY(val, mask)                (((val) & (mask))            ? (DEF_YES) : (DEF_NO ))


/*
*********************************************************************************************************
*                                        DEF_BIT_IS_CLR_ANY()
*
* Description : Determine if any specified bit(s) in a value are clear.
*
* Argument(s) : val         Value to check for specified bit(s) clear.
*
*               mask        Mask of bits to check if clear.
*
* Return(s)   : DEF_YES, if ANY specified bit(s) are     clear in value.
*
*               DEF_NO,  if ALL specified bit(s) are NOT clear in value.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_BIT_IS_CLR_ANY(val, mask)               ((((val) & (mask)) != (mask)) ? (DEF_YES) : (DEF_NO ))


/*$PAGE*/
/*
*********************************************************************************************************
*                                        DEF_VAL()
*
* Description :
*
* Argument(s) :
*
*
*
* Return(s)   : various.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_VAL(var, mask, val)                { DEF_BIT_CLR(var, mask) \
                          DEF_BIT_SET(var, mask & val)  }

/*
*********************************************************************************************************
*                                            MATH MACRO'S
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              DEF_MIN()
*
* Description : Determine the minimum of two values.
*
* Argument(s) : a           First  value.
*
*               b           Second value.
*
* Return(s)   : Minimum of the two values.
*
* Caller(s)   : various.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_MIN(a, b)                                  (((a) < (b)) ? (a) : (b))


/*
*********************************************************************************************************
*                                              DEF_MAX()
*
* Description : Determine the maximum of two values.
*
* Argument(s) : a           First  value.
*
*               b           Second value.
*
* Return(s)   : Maximum of the two values.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_MAX(a, b)                                  (((a) > (b)) ? (a) : (b))


/*$PAGE*/
/*
*********************************************************************************************************
*                                              DEF_ABS()
*
* Description : Determine the absolute value of a value.
*
* Argument(s) : a           Value to calculate absolute value.
*
* Return(s)   : Absolute value of the value.
*
* Caller(s)   : various.
*
* Note(s)     : none.
*********************************************************************************************************
*/

#define  DEF_ABS(a)                                     (((a) < 0) ? (-(a)) : (a))


/*$PAGE*/
/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                     MY MACROS
*********************************************************************************************************
*/

#define HMSB(u) ((uint16_t)((u&0xFFFF0000) >> 16))
#define LMSB(u) ((uint16_t)(u&0x0000FFFF))
//#define MSBH(u) ((CPU_INT08U)((u&0xFF000000) >> 24))
//#define MSBL(u) ((CPU_INT08U)((u&0x00FF0000) >> 16))
//#define LSBH(u) ((CPU_INT08U)((u&0x0000FF00) >> 8))
//#define LSBL(u) ((CPU_INT08U)(u&0x000000FF))
#define MSB(u) ((uint8_t)((u&0xFF00) >> 8))
#define LSB(u) ((uint8_t)(u&0x00FF))
//#define LSBh(u) ((CPU_INT08U)((u&0xF0) >> 4))
//#define LSBl(u) ((CPU_INT08U)(u&0x0F))
#define MAKE_UINT(MSB,LSB) ((((uint16_t)(MSB))<< 8)|(LSB))

#ifdef ENDIANESS
#define ENDIAN_UINT16(Val) ( Val = (uint16_t)( ( (Val & 0xFF00)>>8 ) + ( (Val & 0x00FF)<<8 ) ) )
#define ENDIAN_UINT32(Val) ( Val = (uint32_t)( ((Val & 0xFF000000) >> 24) + ((Val & 0x00FF0000) >> 8) + ((Val & 0x0000FF00) << 8) + ((Val & 0x000000FF) << 24) ) )
#else
#define ENDIAN_UINT16(Val) ( Val = Val )
#define ENDIAN_UINT32(Val) ( Val = Val )
#endif


#endif /* RSKR8C27DEF_H_INCLUDED */

