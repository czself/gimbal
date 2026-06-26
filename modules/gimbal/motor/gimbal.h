#ifndef __GIMBAL_H
#define __GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"

#define GM6020_MAX_CURRENT  16384
#define GM6020_MAX_VOLTAGE  25000
#define GM6020_ECD_MAX      8192

#define GM6020_VOLT_CTRL_ID_1_4   0x1FF
#define GM6020_VOLT_CTRL_ID_5_7   0x2FF
#define GM6020_CURR_CTRL_ID_1_4   0x1FE
#define GM6020_CURR_CTRL_ID_5_7   0x2FE
#define GM6020_FEEDBACK_ID_BASE   0x204

typedef struct {
    uint16_t ecd;
    int16_t  speed_rpm;
    int16_t  current;
    uint8_t  temperature;
    float    angle_deg;
    uint8_t  online;
} gm6020_motor_t;

void gimbal_init(void);
void gimbal_set_target(uint8_t id, float angle_deg);
void gimbal_set_ctrl_mode(uint8_t mode);
void gimbal_update(void);
void gimbal_imu_calibrate(float pitch, float yaw);

extern gm6020_motor_t gimbal_motor[8];
extern float gimbal_target[8];
extern float gimbal_output[8];
extern pid_t gimbal_pid[8];
extern pid_t gimbal_speed_pid[8];
extern float gimbal_speed_target[8];
extern float gimbal_actual_speed[8];
extern uint8_t gimbal_ctrl_mode;
extern uint32_t gimbal_can_send_cnt;
extern uint32_t gimbal_can_fail_cnt;
extern uint32_t gimbal_can_recv_cnt;
extern uint32_t gimbal_can_recv_all;
extern uint8_t  gimbal_can_rx_buf[8];

extern float gimbal_imu_offset_pitch;
extern float gimbal_imu_offset_yaw;
extern uint8_t gimbal_imu_calibrated;
extern float gimbal_imu_pitch;
extern float gimbal_imu_yaw;

extern float gimbal_ff_output[8];
extern uint8_t gimbal_pitch_enabled;

#ifdef __cplusplus
}
#endif

#endif