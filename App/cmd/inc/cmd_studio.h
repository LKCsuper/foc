/* 网页调试接入：只在任务上下文调用，不进入 FOC ISR。 */
#ifndef CMD_STUDIO_H
#define CMD_STUDIO_H
#include <stdbool.h>
#include "motor_config.h"
#if MOTOR_CFG_CMD
int Cmd_Studio(int argc, char *argv[]);
bool Cmd_StudioIsActive(void);
bool Cmd_StudioOwnsControl(void);
void Cmd_StudioPoll(void);
void Cmd_StudioTargetLine(const char *line);
bool Cmd_SerialPlotEnsureTask(void);
#else
static inline bool Cmd_StudioIsActive(void) { return false; }
static inline bool Cmd_StudioOwnsControl(void) { return false; }
static inline void Cmd_StudioPoll(void) {}
#endif
#endif
