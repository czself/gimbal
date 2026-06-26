#ifndef __BSP_VOFA_H
#define __BSP_VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define VOFA_RX_CHANNELS  4

uint32_t pack_vofa_justfloat(uint8_t* buffer, float* data, uint8_t channels);

void vofa_init(void);
void vofa_send_all(
    float pitch_angle, float pitch_target, float pitch_actual_speed,
    float pitch_speed_target, float pitch_output,
    float pitch_angle_kp, float pitch_angle_ki, float pitch_angle_kd,
    float pitch_speed_kp, float pitch_speed_ki,
    float yaw_angle, float yaw_target, float yaw_actual_speed,
    float yaw_speed_target, float yaw_output,
    float yaw_angle_kp, float yaw_angle_ki, float yaw_angle_kd,
    float yaw_speed_kp, float yaw_speed_ki);

void vofa_send_imu(float accel_x, float accel_y, float accel_z,
    float gyro_x, float gyro_y, float gyro_z,
    float roll, float pitch, float yaw,
    float gimbal_imu_pitch, float gimbal_imu_yaw,
    float gyro_lpf_y, float gyro_lpf_z);

void vofa_rx_byte(uint8_t byte);
uint8_t vofa_rx_available(void);
void vofa_rx_get(float *data);

extern uint8_t vofa_rx_idx_debug;
extern uint8_t vofa_rx_ready_debug;
extern uint32_t vofa_rx_err_debug;

#ifdef __cplusplus
}
#endif

#endif