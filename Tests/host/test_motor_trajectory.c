#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_trajectory.h"

int main(void)
{
    motor_trajectory_config_t config = {60.0f, 120.0f, 180.0f, 0.02f, 0.05f, 0.05f};
    motor_trajectory_t trajectory;
    float previous_position;
    int i;

    assert(Motor_Trajectory_Init(&trajectory, 0.0f, 0.0f));
    assert(Motor_Trajectory_SetGoal(&trajectory, 90.0f));
    previous_position = trajectory.position;
    for (i = 0; i < 3000 && trajectory.active; i++) {
        assert(Motor_Trajectory_Step(&trajectory, &config, 0.001f));
        assert(trajectory.position >= previous_position - 1e-5f);
        assert(trajectory.position <= 90.0f + 1e-5f);
        assert(fabsf(trajectory.velocity) <= 60.001f);
        assert(fabsf(trajectory.acceleration) <= 180.001f);
        previous_position = trajectory.position;
    }
    assert(!trajectory.active);
    assert(fabsf(trajectory.position - 90.0f) < 1e-5f);
    assert(fabsf(trajectory.velocity) < 1e-5f);

    assert(Motor_Trajectory_SetGoal(&trajectory, -30.0f));
    for (i = 0; i < 3000 && trajectory.active; i++) {
        assert(Motor_Trajectory_Step(&trajectory, &config, 0.001f));
        assert(trajectory.position >= -30.0f - 1e-5f);
    }
    assert(!trajectory.active);
    assert(fabsf(trajectory.position + 30.0f) < 1e-5f);
    puts("test_motor_trajectory: pass");
    return 0;
}
