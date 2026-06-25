#include "ahrs.h"
#include <math.h>

ahrs_quat_t ahrs_quat = {1.0f, 0.0f, 0.0f, 0.0f};

static float twoKp = 2.0f * 0.5f;
static float twoKi = 2.0f * 0.0f;
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;

static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    int32_t i = *(int32_t *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void ahrs_init(void)
{
    ahrs_quat.q0 = 1.0f;
    ahrs_quat.q1 = 0.0f;
    ahrs_quat.q2 = 0.0f;
    ahrs_quat.q3 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = ahrs_quat.q1 * ahrs_quat.q3 - ahrs_quat.q0 * ahrs_quat.q2;
        halfvy = ahrs_quat.q0 * ahrs_quat.q1 + ahrs_quat.q2 * ahrs_quat.q3;
        halfvz = ahrs_quat.q0 * ahrs_quat.q0 - 0.5f + ahrs_quat.q3 * ahrs_quat.q3;

        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;
        halfez = ax * halfvy - ay * halfvx;

        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = ahrs_quat.q0;
    qb = ahrs_quat.q1;
    qc = ahrs_quat.q2;

    ahrs_quat.q0 += (-qb * gx - qc * gy - ahrs_quat.q3 * gz);
    ahrs_quat.q1 += (qa * gx + qc * gz - ahrs_quat.q3 * gy);
    ahrs_quat.q2 += (qa * gy - qb * gz + ahrs_quat.q3 * gx);
    ahrs_quat.q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = inv_sqrt(ahrs_quat.q0 * ahrs_quat.q0 + ahrs_quat.q1 * ahrs_quat.q1
                         + ahrs_quat.q2 * ahrs_quat.q2 + ahrs_quat.q3 * ahrs_quat.q3);
    ahrs_quat.q0 *= recipNorm;
    ahrs_quat.q1 *= recipNorm;
    ahrs_quat.q2 *= recipNorm;
    ahrs_quat.q3 *= recipNorm;
}

void ahrs_get_euler(float *roll, float *pitch, float *yaw)
{
    *roll = atan2f(2.0f * (ahrs_quat.q0 * ahrs_quat.q1 + ahrs_quat.q2 * ahrs_quat.q3),
                   1.0f - 2.0f * (ahrs_quat.q1 * ahrs_quat.q1 + ahrs_quat.q2 * ahrs_quat.q2));
    *pitch = asinf(2.0f * (ahrs_quat.q0 * ahrs_quat.q2 - ahrs_quat.q3 * ahrs_quat.q1));
    *yaw = atan2f(2.0f * (ahrs_quat.q0 * ahrs_quat.q3 + ahrs_quat.q1 * ahrs_quat.q2),
                  1.0f - 2.0f * (ahrs_quat.q2 * ahrs_quat.q2 + ahrs_quat.q3 * ahrs_quat.q3));
}