#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float prev_current;
    float integral_limit;
    float output_limit;
    float raw_output;
    float error;
    uint8_t wrap_angle;
    uint8_t d_on_measurement;
    float    integral_decay;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
float pid_calc(pid_t *pid, float target, float current, float dt);

#ifdef __cplusplus
}
#endif

#endif