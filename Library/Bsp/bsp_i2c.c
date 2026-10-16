/*
 * @Description: BM75 外设口 I2C2 驱动（PB10/PB11）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "bsp_i2c.h"
/* Private define ------------------------------------------------------------*/
#define BSP_I2C2_TIMEOUT_LOOPS (20000U)
#define BSP_I2C2_CLOCK_HZ      (400000U)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static bool Bsp_I2c2_WaitEvent(uint32_t event);
static bool Bsp_I2c2_WaitFlag(uint32_t flag, FlagStatus status);
static void Bsp_I2c2_Recover(void);
/* Private functions ---------------------------------------------------------*/

static bool Bsp_I2c2_WaitEvent(uint32_t event)
{
    uint32_t timeout = BSP_I2C2_TIMEOUT_LOOPS;

    while (I2C_CheckEvent(I2C2, event) == ERROR) {
        if (timeout == 0U) return false;
        timeout--;
    }
    return true;
}

static bool Bsp_I2c2_WaitFlag(uint32_t flag, FlagStatus status)
{
    uint32_t timeout = BSP_I2C2_TIMEOUT_LOOPS;

    while (I2C_GetFlagStatus(I2C2, flag) != status) {
        if (timeout == 0U) return false;
        timeout--;
    }
    return true;
}

static void Bsp_I2c2_Recover(void)
{
    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_SoftwareResetCmd(I2C2, ENABLE);
    I2C_SoftwareResetCmd(I2C2, DISABLE);
    Bsp_I2c2_Init();
}

/**
 * @description: 初始化 BM75 外设口 I2C2。
 * @detail: PB10=SCL、PB11=SDA，开漏复用，400 kHz；不涉及电机 ADC/PWM 外设。
 * @return {*}
 * @author: lkc
 */
void Bsp_I2c2_Init(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef i2c;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_High_Speed;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);

    I2C_DeInit(I2C2);
    I2C_StructInit(&i2c);
    i2c.I2C_ClockSpeed = BSP_I2C2_CLOCK_HZ;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0U;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2, &i2c);
    I2C_Cmd(I2C2, ENABLE);
}

/**
 * @description: 从 7 位地址设备读取一个寄存器。
 * @param {uint8_t} address_7bit  7 位从机地址
 * @param {uint8_t} reg           寄存器地址
 * @param {uint8_t *} value       输出数据，不可为空
 * @return {bool} true=读取成功
 * @author: lkc
 */
bool Bsp_I2c2_ReadRegister(uint8_t address_7bit, uint8_t reg, uint8_t *value)
{
    uint32_t timeout = BSP_I2C2_TIMEOUT_LOOPS;
    uint32_t primask;

    if (value == NULL) return false;

    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == SET) {
        if (timeout == 0U) {
            Bsp_I2c2_Recover();
            return false;
        }
        timeout--;
    }

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    I2C_GenerateSTART(I2C2, ENABLE);
    if (!Bsp_I2c2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) {
        goto fail;
    }
    I2C_Send7bitAddress(I2C2, (uint8_t)(address_7bit << 1U), I2C_Direction_Transmitter);
    if (!Bsp_I2c2_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        goto fail;
    }
    I2C_SendData(I2C2, reg);
    if (!Bsp_I2c2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        goto fail;
    }

    I2C_GenerateSTART(I2C2, ENABLE);
    if (!Bsp_I2c2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) {
        goto fail;
    }
    I2C_Send7bitAddress(I2C2, (uint8_t)(address_7bit << 1U), I2C_Direction_Receiver);
    if (!Bsp_I2c2_WaitFlag(I2C_FLAG_ADDR, SET)) {
        goto fail;
    }

    /* 单字节接收必须在清 ADDR 前关闭 ACK，并紧接着产生 STOP。 */
    primask = __get_PRIMASK();
    __disable_irq();
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    (void)I2C2->SR1;
    (void)I2C2->SR2;
    I2C_GenerateSTOP(I2C2, ENABLE);
    if (primask == 0U) {
        __enable_irq();
    }
    if (!Bsp_I2c2_WaitFlag(I2C_FLAG_RXNE, SET)) {
        goto fail;
    }
    *value = I2C_ReceiveData(I2C2);
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return true;

fail:
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    Bsp_I2c2_Recover();
    return false;
}

#ifdef __cplusplus
}
#endif
