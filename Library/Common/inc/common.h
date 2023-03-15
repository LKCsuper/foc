/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2023-02-26 21:30:25
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-02 23:18:35
 */
/**
* @file         
* @brief		This is a brief description.
* @details	    This is the detail description.
* @author		author
* @date		    date
* @version	    v1.0
* @par Copyright(c): 	abc corporation
* @par History:         
*	version: author, date, desc\n
*/
#ifndef COMMON_H 
#define COMMON_H 
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------0.1------------------------------------------------*/
// typedef unsigned char boolean0; /* Boolean value type. */
// typedef unsigned long int uint32; /* Unsigned 32 bit value */
// typedef unsigned short uint16; /* Unsigned 16 bit value */
// typedef unsigned char uint8; /* Unsigned 8 bit value */
// typedef signed long int int32; /* Signed 32 bit value */
// typedef signed short int16; /* Signed 16 bit value */
// typedef signed char int8; /* Signed 8 bit value */

typedef unsigned char boolean; /* Boolean value type. */
typedef unsigned long int uint32; /* Unsigned 32 bit value */
typedef unsigned short uint16; /* Unsigned 16 bit value */
typedef unsigned char uint8; /* Unsigned 8 bit value */
typedef signed long int int32; /* Signed 32 bit value */
typedef signed short int16; /* Signed 16 bit value */
typedef signed char int8; /* Signed 8 bit value */

#define VOID void
#define UCHAR unsigned char
#define CHAR int8_t
#define LONG int32_t
#define ULONG unsigned long
#define STATIC static
#define SHORT int16_t
#define USHORT unsigned short
#define BOOL bool

/* 设置位 */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
/* 清除位 */
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
/* 读位 */
#define READ_BIT(REG, BIT)    ((REG) & (BIT))

/* 取地址数据 */
#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
#define REG16(addr)                  (*(volatile uint16_t *)(uint32_t)(addr))
#define REG8(addr)                   (*(volatile uint8_t *)(uint32_t)(addr))

/* 取几位 */
#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))
/* 取好几位 */
#define BITS(start, end)             ((0xFFFFFFFFUL << (start)) & (0xFFFFFFFFUL >> (31U - (uint32_t)(end)))) 
#define GET_BITS(regval, start, end) (((regval) & BITS((start),(end))) >> (start))

#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

/* 得到一个字的高位和低位 */
#define WORD_LO(xxx) ((byte) ((word)(xxx) & 255))
#define WORD_HI(xxx) ((byte) ((word)(xxx) >> 8))

/* 最大值最小值 */
#define MAX( x, y ) ( ((x) > (y)) ? (x) : (y) )
#define MIN( x, y ) ( ((x) < (y)) ? (x) : (y) )

/*得到指定地址上的一个字节或字*/
#define MEM_B( x ) ( *( (byte *) (x) ) )
#define MEM_W( x ) ( *( (word *) (x) ) )

/* 得到fiele在结构体的偏移量 */

/* 得到一个结构体中field所占用的字节数 */
#define FSIZ( type, field ) sizeof( ((type *) 0)->field )

/* 数字转换为字符串 */



/* Compiler Related Definitions */
#ifdef __CC_ARM                         /* ARM Compiler */
    #define __WEAK                     __weak
#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
    #define __WEAK                     __weak
#elif defined (__GNUC__)                /* GNU GCC Compiler */
    #define __WEAK                     __attribute__((weak))
#elif defined (__ADSPBLACKFIN__)        /* for VisualDSP++ Compiler */
    #define __WEAK                     __attribute__((weak))
#elif defined (_MSC_VER)
    #define __WEAK
#elif defined (__TI_COMPILER_VERSION__)
    #define __WEAK
#else
    #error not supported tool chain
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#endif // 宏结束行
