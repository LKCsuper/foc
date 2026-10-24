#ifndef MOTOR_DIAG_PHASE_H
#define MOTOR_DIAG_PHASE_H
#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PHASE_CAPACITY 64U
typedef struct {
    uint32_t sequence;
    uint32_t time_us;
    uint32_t count;
    uint32_t cr1;
    uint32_t divider;
    uint32_t flags;
    uint32_t irq_entry_us;
    uint32_t irq_exit_us;
    uint32_t dma_status;
    uint32_t adc_status;
    uint32_t dma_remaining;
    uint32_t irq_pending;
} motor_phase_event_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t head;
    uint32_t count;
    uint32_t seen;
    uint32_t trigger_sequence;
    uint32_t reason;
    motor_phase_event_t events[MOTOR_PHASE_CAPACITY];
} motor_phase_debug_t;

extern motor_phase_debug_t g_motor_phase_debug;
extern volatile uint32_t g_motor_phase_active;
void Motor_Timing_PhaseEdgeArm(void);
void Motor_Timing_PhaseEdgeReserve3000(void);
void Motor_Timing_PhaseEdgePoll(bool running, bool ready, uint32_t time_us);
void Motor_Timing_PhaseEdgeStop(void);
void Motor_Timing_PhaseIrqBegin(uint32_t time_us);
void Motor_Timing_PhaseIrqEnd(uint32_t time_us, uint32_t dma_status, uint32_t adc_status,
                              uint32_t remaining, uint32_t pending);
void Motor_Timing_PhaseMarkStatus(void);
void Motor_Timing_PhaseEdgeNote(uint32_t time_us, uint32_t count, uint32_t cr1,
                            uint32_t divider, uint32_t flags, bool running);
#endif
