/*
 * @Description: BM75 MT6701 SSI 板级驱动
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "bsp_mt6701_ssi.h"
/* Private define ------------------------------------------------------------*/
#define BSP_MT6701_SSI_SPI           SPI1
#define BSP_MT6701_SSI_GPIO          GPIOA
#define BSP_MT6701_SSI_CLK_PIN       GPIO_Pin_5
#define BSP_MT6701_SSI_DO_PIN        GPIO_Pin_6
#define BSP_MT6701_SSI_MOSI_PIN      GPIO_Pin_7
#define BSP_MT6701_SSI_CSN_GPIO      GPIOB
#define BSP_MT6701_SSI_CSN_PIN       GPIO_Pin_11
#define BSP_MT6701_SSI_TIMEOUT_LOOPS (2000U)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static bool Bsp_MT6701_SsiTransferByte(uint8_t tx, uint8_t *rx);
/* Private functions ---------------------------------------------------------*/

static bool Bsp_MT6701_SsiTransferByte(uint8_t tx, uint8_t *rx)
{
    uint32_t timeout = BSP_MT6701_SSI_TIMEOUT_LOOPS;

    while (SPI_I2S_GetFlagStatus(BSP_MT6701_SSI_SPI, SPI_I2S_FLAG_TXE) == RESET) {
        if (timeout-- == 0U) {
            return false;
        }
    }
    SPI_I2S_SendData(BSP_MT6701_SSI_SPI, tx);

    timeout = BSP_MT6701_SSI_TIMEOUT_LOOPS;
    while (SPI_I2S_GetFlagStatus(BSP_MT6701_SSI_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
        if (timeout-- == 0U) {
            return false;
        }
    }
    *rx = (uint8_t)SPI_I2S_ReceiveData(BSP_MT6701_SSI_SPI);
    return true;
}

/**
 * @description: 初始化 SPI1，供 MT6701 SSI 读数。
 * @detail: PA5=CLK、PA6=DO、PA7=MOSI、PB11=CSN。时钟约 5.25 MHz，mode 1。不改 TIM1/ADC。
 * @return {*}
 * @author: lkc
 */
void Bsp_MT6701_SsiInit(void)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef spi;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    gpio.GPIO_Pin = BSP_MT6701_SSI_CLK_PIN | BSP_MT6701_SSI_DO_PIN | BSP_MT6701_SSI_MOSI_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_High_Speed;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BSP_MT6701_SSI_GPIO, &gpio);

    gpio.GPIO_Pin = BSP_MT6701_SSI_CSN_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_High_Speed;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BSP_MT6701_SSI_CSN_GPIO, &gpio);
    GPIO_SetBits(BSP_MT6701_SSI_CSN_GPIO, BSP_MT6701_SSI_CSN_PIN);

    SPI_I2S_DeInit(BSP_MT6701_SSI_SPI);
    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    /* MT6701 在上升沿更新 DO，STM32 在下降沿采样：SPI mode 1。 */
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; /* 84 MHz / 16 = 5.25 MHz */
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7U;
    SPI_Init(BSP_MT6701_SSI_SPI, &spi);
    SPI_Cmd(BSP_MT6701_SSI_SPI, ENABLE);
}

/**
 * @description: 读取一帧 24 位 SSI 数据。
 * @param {uint32_t *} frame  输出原始帧，不可为空
 * @return {bool} true=三字节都读到
 * @author: lkc
 */
bool Bsp_MT6701_SsiReadFrame(uint32_t *frame)
{
    uint8_t rx[3];
    uint32_t timeout;

    if (frame == NULL) {
        return false;
    }

    GPIO_ResetBits(BSP_MT6701_SSI_CSN_GPIO, BSP_MT6701_SSI_CSN_PIN);
    __NOP();
    __NOP();
    __NOP();
    if (!Bsp_MT6701_SsiTransferByte(0x00U, &rx[0]) || !Bsp_MT6701_SsiTransferByte(0x00U, &rx[1]) ||
        !Bsp_MT6701_SsiTransferByte(0x00U, &rx[2])) {
        GPIO_SetBits(BSP_MT6701_SSI_CSN_GPIO, BSP_MT6701_SSI_CSN_PIN);
        return false;
    }

    timeout = BSP_MT6701_SSI_TIMEOUT_LOOPS;
    while (SPI_I2S_GetFlagStatus(BSP_MT6701_SSI_SPI, SPI_I2S_FLAG_BSY) == SET) {
        if (timeout-- == 0U) {
            GPIO_SetBits(BSP_MT6701_SSI_CSN_GPIO, BSP_MT6701_SSI_CSN_PIN);
            return false;
        }
    }
    __NOP();
    __NOP();
    GPIO_SetBits(BSP_MT6701_SSI_CSN_GPIO, BSP_MT6701_SSI_CSN_PIN);

    *frame = ((uint32_t)rx[0] << 16U) | ((uint32_t)rx[1] << 8U) | (uint32_t)rx[2];
    return true;
}

#ifdef __cplusplus
}
#endif
