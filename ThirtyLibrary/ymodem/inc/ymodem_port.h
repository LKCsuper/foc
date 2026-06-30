/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-03 22:51:45
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Ymodem_Uart_Init(uint32_t ulBound);
void Ymodem_PutChar(uint8_t c);
uint32_t Ymodem_GetChar(uint8_t *key);
void Ymodem_Delay(unsigned int ms);
void Ymodem_Flash_Set_Flag(bool isSet);
uint32_t Ymodem_Flash_Get_Flag(void);
void Ymodem_Flash_Init(void);
void Ymodem_Flash_Erase_App(void);
uint32_t Ymodem_Flash_Write(uint32_t *Data, uint32_t DataLength);
void Ymodem_Info(void);
void Ymodem_Jump_App(void);

#ifdef __cplusplus
}
#endif
