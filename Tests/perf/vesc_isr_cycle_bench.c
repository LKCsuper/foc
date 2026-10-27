/*
 * @Description: 只读 VESC HD75 固件的周期仿真夹具；实际函数来自已有 ELF，不替换控制算法。
 * 外设仅为模拟器中的内存输入，不执行硬件初始化或 RTOS 调度。地址由配套脚本从原 map 生成。
 */
#include "vesc_symbols.h"
#include "mcpwm_foc.h"
#include "confgenerator.h"
#include <string.h>
extern volatile uint16_t ADC_Value[];
_Static_assert(sizeof(motor_all_state_t)==VESC_STATE_SIZE,"ELF/source state layout mismatch");
#define MOTOR ((motor_all_state_t *)VESC_MOTOR_ADDR)
#define CONFIG ((mc_configuration *)VESC_CONFIG_ADDR)
#define INIT_DONE (*(volatile unsigned char *)VESC_INIT_DONE_ADDR)
volatile unsigned v_case, v_iter, v_fault, v_done;
volatile float v_iq, v_vbus;
extern void confgenerator_set_defaults_mcconf(mc_configuration *);
extern void foc_precalc_values(motor_all_state_t *);
/**
 * @description: 标记回调周期测量起点；夹具空基线在汇总时扣除。
 * @return {void} 无
 */
__attribute__((noinline)) void VBegin(void) { __asm volatile("nop"); }
/**
 * @description: 标记回调周期测量终点；不计入后续元数据采集。
 * @return {void} 无
 */
__attribute__((noinline)) void VEnd(void) { __asm volatile("nop"); }
/**
 * @description: 标记故障、母线与目标电流元数据已就绪。
 * @return {void} 无
 */
__attribute__((noinline)) void VAfter(void) { __asm volatile("nop"); }
/**
 * @description: 准备固定输入并运行六个场景，各测 512 拍。
 * @detail: 默认参数来自实际 VESC 函数；补齐未运行后台任务导致缺失的限流/温补/观测增益状态。
 *          相电压无物理模型，因此关闭相电压滤波；运行场景先预热 4096 拍。
 * @return {void} 测完置 v_done 并停留，不返回
 */
void VMain(void) {
    __asm volatile("cpsid i");
    SCB->CPACR |= 0xf00000;
    for (v_case=0; v_case<6; v_case++) {
        memset(MOTOR,0,sizeof(*MOTOR));
        confgenerator_set_defaults_mcconf(CONFIG);
        CONFIG->motor_type=MOTOR_TYPE_FOC;
        CONFIG->foc_f_zv=30000.0f;
        CONFIG->foc_control_sample_mode=FOC_CONTROL_SAMPLE_MODE_V0;
        CONFIG->foc_current_sample_mode=FOC_CURRENT_SAMPLE_MODE_LONGEST_ZERO;
        CONFIG->foc_sensor_mode=FOC_SENSOR_MODE_SENSORLESS;
        CONFIG->foc_offsets_current[0]=CONFIG->foc_offsets_current[1]=CONFIG->foc_offsets_current[2]=2045.0f;
        CONFIG->lo_current_max=5.0f;CONFIG->lo_current_min=-5.0f;
        CONFIG->lo_in_current_max=5.0f;CONFIG->lo_in_current_min=-5.0f;
        CONFIG->foc_temp_comp=false;
        CONFIG->foc_phase_filter_enable=false;
        MOTOR->m_conf=CONFIG;
        foc_precalc_values(MOTOR);
        MOTOR->m_res_temp_comp=CONFIG->foc_motor_r;
        MOTOR->m_current_ki_temp_comp=CONFIG->foc_current_ki;
        MOTOR->m_gamma_now=CONFIG->foc_observer_gain;
        MOTOR->m_motor_state.v_bus=24.0f;
        MOTOR->m_motor_state.max_duty=CONFIG->l_max_duty;
        MOTOR->m_state=v_case>=3?MC_STATE_RUNNING:MC_STATE_OFF;
        MOTOR->m_control_mode=v_case==3?CONTROL_MODE_OPENLOOP:CONTROL_MODE_CURRENT;
        MOTOR->m_iq_set=0.6f;
        MOTOR->m_openloop_speed=300.0f*CONFIG->si_motor_poles*0.5f*6.28318530718f/60.0f;
        INIT_DONE=1;
        TIM1->ARR=5600;TIM1->CR1=v_case==1?0:TIM_CR1_DIR;TIM1->CNT=3600;
        TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=2800;
        GPIOB->IDR=1<<7;
        for(unsigned i=0;i<HW_ADC_CHANNELS+HW_ADC_CHANNELS_EXTRA;i++)ADC_Value[i]=2045;
        ADC_Value[ADC_IND_VIN_SENS]=1126;
        if(v_case>=3) for(unsigned j=0;j<4096;j++) mcpwm_foc_adc_int_handler(0,0);
        for(v_iter=0;v_iter<512;v_iter++) {
            if(v_case==5 && (v_iter&255U)==0U) g_foc_profile.trigger_start=true;
            VBegin();
            if(v_case) mcpwm_foc_adc_int_handler(0,0);
            VEnd();
            v_iq=MOTOR->m_motor_state.iq_target;
            v_vbus=MOTOR->m_motor_state.v_bus;
            v_fault=*(volatile unsigned *)((char*)CONFIG+sizeof(mc_configuration));
            VAfter();
        }
    }
    v_done=1;
    for(;;){}
}
