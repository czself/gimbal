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
void vofa_send_all(float yaw, float yaw_target, float yaw_current,
    float yaw_angle_integral, float yaw_speed_integral,
    float yaw_angle_kp, float yaw_angle_ki, float yaw_angle_kd,
    float yaw_speed_kp, float yaw_speed_ki, float yaw_speed_kd,
    float yaw_ff_output, float pitch_ff_output);

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