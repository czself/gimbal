#ifndef __AHRS_H
#define __AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} ahrs_quat_t;

extern ahrs_quat_t ahrs_quat;

void ahrs_init(void);
void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void ahrs_get_euler(float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif