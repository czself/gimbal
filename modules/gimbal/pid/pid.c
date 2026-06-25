#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_current = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->wrap_angle = 0;
    pid->d_on_measurement = 0;
}

float pid_calc(pid_t *pid, float target, float current, float dt)
{
    float error = target - current;

    if (pid->wrap_angle) {
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;
    }

    float output;

    pid->error = error;

    if (error > -0.1f && error < 0.1f) {
        pid->integral *= 0.98f;
    } else {
        pid->integral += error * dt;
    }

    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }

    float d_term;
    if (pid->d_on_measurement) {
        d_term = -pid->kd * (current - pid->prev_current) / dt;
    } else {
        d_term = pid->kd * (error - pid->prev_error) / dt;
    }

    output = pid->kp * error
           + pid->ki * pid->integral
           + d_term;

    pid->raw_output = output;

    pid->prev_error = error;
    pid->prev_current = current;

    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    return output;
}