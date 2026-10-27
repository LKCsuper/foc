/*
 * @Description: Keil Cortex-M4 ISR 周期仿真夹具；固定 ADC/寄存器输入，执行正式中断函数。
 * 不启动硬件初始化或 RTOS。外设空间由调试器映射为 RAM；不模拟 DMA 搬运、真实时序或电机。
 * 通过普通函数调用测 IRQ 包装及其调用链，硬件异常进入/退出不在计数范围内。
 */
#if defined(__CC_ARM)
/* 固定测量夹具为 O1，避免正式目标优化等级变化影响计时脚手架。 */
#pragma O1
#endif
#include "includes.h"
#include "motor.h"
#include "motor_feedback.h"
#include "motor_protect.h"
#include "motor_timing.h"
#include "motor_timing_internal.h"
uint32_t SystemCoreClock = 168000000U;
#ifndef SIM_SAMPLES
#define SIM_SAMPLES 512U
#endif
/* 调试器读取的测试编号、状态及标记；统计采集在测量区间之外。 */
volatile uint32_t sim_case, sim_iteration, sim_fault, sim_done, sim_control_hash;
motor_profile_snapshot_t sim_profile;
volatile float sim_iq, sim_vd, sim_vq, sim_vbus;
/**
 * @description: 仿真启动时仅启用 FPU，不配置时钟或板级外设。
 * @return {void} 无
 */
void SystemInit(void) { SCB->CPACR |= 0xF00000U; }
/**
 * @description: 提供仿真夹具的固定 168 MHz 时钟声明，不写实际时钟树。
 * @return {void} 无
 */
void SystemCoreClockUpdate(void) { SystemCoreClock = 168000000U; }
/**
 * @description: 标记测量起点；夹具开销由 case 0 空基线近似扣除。
 * @return {void} 无
 */
__attribute__((noinline)) void Sim_Begin(void) { __asm { NOP } }
/**
 * @description: 标记测量终点；到达本函数入口后停止累计。
 * @return {void} 无
 */
__attribute__((noinline)) void Sim_End(void) { __asm { NOP } }
/**
 * @description: 标记非计时区元数据已就绪；读取状态不计入 ISR 开销。
 * @return {void} 无
 */
__attribute__((noinline)) void Sim_After(void) { __asm { NOP } }
/**
 * @description: 填充虚拟寄存器和 ADC 缓冲，不调用板级外设初始化。
 * @detail: 固定 TIM1 周期 5600、计数侧和位置；ADC 输入约为 24 V 母线及零相电流。
 * @return {void} 无
 */
static void Sim_Registers(void) {
    unsigned i;
    TIM1->ARR=5600U; TIM1->PSC=0U; TIM1->CR1=TIM_CR1_DIR; TIM1->CNT=3600U;
    TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=2800U;
    DMA2_Stream4->CR=DMA_SxCR_TCIE;
    *(volatile uint32_t *)&DMA2->HISR=DMA_HISR_TCIF4;
    UART4->SR=USART_SR_TXE|USART_SR_TC;
    for(i=0;i<HW_ADC_CHANNELS+HW_ADC_CHANNELS_EXTRA;i++) ADC_Value[i]=2045U;
    ADC_Value[ADC_IND_VIN_SENS]=1126U;
}
/**
 * @description: 在非计时预热区推进虚拟微秒时基并调用真实 IRQ 函数。
 * @detail: 每拍增加 67 us；DMA TC 标志及 TC 中断使能位由夹具给出。
 * @return {void} 无
 */
static void Sim_Invoke(void) {
    TIM5->CNT += 67U;
    *(volatile uint32_t *)&DMA2->HISR=DMA_HISR_TCIF4;
    DMA2_Stream4_IRQHandler();
}
/**
 * @description: 在计时区外计算控制结果原码摘要，供同输入优化前后逐拍对照。
 * @detail: 包含电流反馈、目标、积分、角度、母线和三相比较值；不含耗时统计。
 * @return {uint32_t} FNV-1a 摘要，无量纲
 */
static uint32_t Sim_ControlHash(void)
{
    const float values[] = {motor.ia, motor.ib, motor.ic, motor.Id, motor.Iq,
        motor.id_target, motor.iq_target, motor.ol_vd_int, motor.ol_vq_int,
        motor.theta, motor.ol_theta, motor.ol_current_rpm, motor.v_bus,
        (float)motor.duty1, (float)motor.duty2, (float)motor.duty3};
    const unsigned char *bytes = (const unsigned char *)values;
    uint32_t hash = 2166136261U;
    unsigned i;
    for (i = 0; i < sizeof(values); i++) hash = (hash ^ bytes[i]) * 16777619U;
    return hash;
}
/**
 * @description: 依次执行空基线及八种 ISR 场景，每场景采集 SIM_SAMPLES 拍。
 * @detail: 运行场景先预热 4096 拍；不启动电机模型，不代表全部运行状态或最坏执行时间。
 * @return {int} 测完置 sim_done 后停留，不返回
 */
int main(void) {
    unsigned i;
    __disable_irq();
    Sim_Registers();
    for(sim_case=0;sim_case<9;sim_case++) {
        MOTOR_Param_Init();
        motor.ol_enable=false;
        Motor_Protect_ClearFault();
        Motor_Timing_ProfileSetDetailed(false);
        g_motor_diag.performance_enabled=true;
        Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_OPENLOOP);
        TIM1->CR1=TIM_CR1_DIR;
        for(i=0;i<4;i++) Sim_Invoke();
        motor.currentOffset[0]=motor.currentOffset[1]=motor.currentOffset[2]=2045.0f;
        motor.v_bus=24.0f;
        motor.ol_enable=sim_case>=3;
        motor.ol_mode=sim_case==3?MOTOR_OL_MODE_VF:MOTOR_OL_MODE_IF;
        motor.ol_target_rpm=300.0f;
        motor.ol_current_rpm=300.0f;
        motor.ol_iq_set=0.6f;
        motor.sensorless_enable=sim_case==5 || sim_case==7 || sim_case==8;
        motor.sensorless_takeover=false;
        if(motor.sensorless_enable) Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_SENSORLESS);
        if(sim_case>=3) for(i=0;i<4096;i++) Sim_Invoke();
        if(sim_case==1) TIM1->CR1=0U;
        if(sim_case==6) Motor_Protect_TriggerFault(FAULT_CODE_ABS_OVER_CURRENT,&motor);
        if(sim_case==7) Motor_Timing_ProfileSetDetailed(true);
        if(sim_case==8) g_motor_diag.performance_enabled=false;
        for(sim_iteration=0;sim_iteration<SIM_SAMPLES;sim_iteration++) {
            TIM5->CNT+=67U;
            *(volatile uint32_t *)&DMA2->HISR=DMA_HISR_TCIF4;
            Sim_Begin();
            if(sim_case!=0) DMA2_Stream4_IRQHandler();
            Sim_End();
            sim_control_hash=Sim_ControlHash();
            sim_fault=(uint32_t)Motor_Protect_GetFault();
            sim_iq=motor.Iq;sim_vd=motor.modD;sim_vq=motor.modQ;sim_vbus=motor.v_bus;
            Motor_Timing_ProfileSnapshot(&sim_profile);
            Sim_After();
        }
    }
    sim_done=1;
    for(;;) {}
}
