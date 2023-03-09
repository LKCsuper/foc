/*
 * @Description: 
 * @Date: 2022-11-28 19:28:49
 * @LastEditTime: 2023-03-02 23:18:15
 * @FilePath: \YMODEM\ThirtyParty\ymodem\ymodem_common.h
 */
/**
  ******************************************************************************
  * @file    ymodem_common.h
  * @author
  * @version V1.0.0
  * @date
  * @brief   ymodem 通用
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMON_H
#define __COMMON_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Constants used by Serial Command Line Mode */
#define CMD_STRING_SIZE       128

/* Common routines */
#define IS_AF(c)  ((c >= 'A') && (c <= 'F'))
#define IS_af(c)  ((c >= 'a') && (c <= 'f'))
#define IS_09(c)  ((c >= '0') && (c <= '9'))
#define ISVALIDHEX(c)  IS_AF(c) || IS_af(c) || IS_09(c)
#define ISVALIDDEC(c)  IS_09(c)
#define CONVERTDEC(c)  (c - '0')

#define CONVERTHEX_alpha(c)  (IS_AF(c) ? (c - 'A'+10) : (c - 'a'+10))
#define CONVERTHEX(c)   (IS_09(c) ? (c - '0') : CONVERTHEX_alpha(c))

#define SerialPutString(x) Serial_PutString((uint8_t*)(x))

#define VOID void
#define UCHAR unsigned char
#define CHAR int8_t
#define LONG int32_t
#define ULONG unsigned long
#define STATIC static
#define SHORT int16_t
#define USHORT unsigned short

/* Exported functions ------------------------------------------------------- */
void Int2Str(uint8_t* str, int32_t intnum);
uint32_t Str2Int(uint8_t *inputstr, int32_t *intnum);
uint32_t GetIntegerInput(int32_t * num);
uint8_t GetKey(void);
void Serial_PutString(uint8_t *s);
void GetInputString(uint8_t * buffP);

#ifdef __cplusplus
};
#endif
#endif  /* __COMMON_H */
