#include "ahrs.h"

#include <math.h>

float _inv_sqrt(float x);

/* IMU algorithm update */

void se_madgwick_ahrs_update_imu_with_mag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float delta_time, float beta, float q[4]) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    /*
     * Use IMU algorithm if magnetometer measurement is invalid
     * (avoids NaN in magnetometer normalization)
     */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        se_madgwick_ahrs_update_imu(gx, gy, gz, ax, ay, az, delta_time, beta, q);
        return;
    }

    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
    qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
    qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
    qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

    /*
     * Compute feedback only if accelerometer measurement is valid
     * (avoids NaN in accelerometer normalization)
     */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        /* Normalize accelerometer measurement */
        recipNorm = _inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Normalize magnetometer measurement */
        recipNorm = _inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        /* Auxiliary variables to avoid repeated arithmetic */
        _2q0mx = 2.0f * q[0] * mx;
        _2q0my = 2.0f * q[0] * my;
        _2q0mz = 2.0f * q[0] * mz;
        _2q1mx = 2.0f * q[1] * mx;
        _2q0 = 2.0f * q[0];
        _2q1 = 2.0f * q[1];
        _2q2 = 2.0f * q[2];
        _2q3 = 2.0f * q[3];
        _2q0q2 = 2.0f * q[0] * q[2];
        _2q2q3 = 2.0f * q[2] * q[3];
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];

        /* Reference direction of Earth's magnetic field */
        hx = mx * q0q0 - _2q0my * q[3] + _2q0mz * q[2] + mx * q1q1 + _2q1 * my * q[2] + _2q1 * mz * q[3] - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q[3] + my * q0q0 - _2q0mz * q[1] + _2q1mx * q[2] - my * q1q1 + my * q2q2 + _2q2 * mz * q[3] - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q[2] + _2q0my * q[1] + mz * q0q0 + _2q1mx * q[3] - mz * q1q1 + _2q2 * my * q[3] - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        /* Gradient decent algorithm corrective step */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             _2bz * q[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = _inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        /* Normalize step magnitude */
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* Apply feedback step */
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    /* Integrate rate of change of quaternion to yield quaternion */
    q[0] += qDot1 * delta_time;
    q[1] += qDot2 * delta_time;
    q[2] += qDot3 * delta_time;
    q[3] += qDot4 * delta_time;

    /* Normalize quaternion */
    recipNorm = _inv_sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

void se_madgwick_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float delta_time, float beta, float q[4]) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
    qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
    qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
    qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

    /*
*Compute feedback only if accelerometer measurement valid
*(avoids NaN in accelerometer normalisation)
*/
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = _inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q[0];
        _2q1 = 2.0f * q[1];
        _2q2 = 2.0f * q[2];
        _2q3 = 2.0f * q[3];
        _4q0 = 4.0f * q[0];
        _4q1 = 4.0f * q[1];
        _4q2 = 4.0f * q[2];
        _8q1 = 8.0f * q[1];
        _8q2 = 8.0f * q[2];
        q0q0 = q[0] * q[0];
        q1q1 = q[1] * q[1];
        q2q2 = q[2] * q[2];
        q3q3 = q[3] * q[3];

        /* Gradient decent algorithm corrective step */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;
        /* normalize step magnitude */
        recipNorm = _inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* Apply feedback step */
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    /* Integrate rate of change of quaternion to yield quaternion */
    q[0] += qDot1 * delta_time;
    q[1] += qDot2 * delta_time;
    q[2] += qDot3 * delta_time;
    q[3] += qDot4 * delta_time;

    /* Normalise quaternion */
    recipNorm = _inv_sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

void se_mahony_ahrs_update_imu_with_mag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float delta_time, float ki_2, float kp_2, float fb_i[3], float q[4]) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /*
     * Use IMU algorithm if magnetometer measurement is invalid
     * (avoids NaN in magnetometer normalization)
     */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        se_mahony_ahrs_update_imu(gx, gy, gz, ax, ay, az, delta_time, ki_2, kp_2, fb_i, q);
        return;
    }

    /*
     * Compute feedback only if accelerometer measurement is valid
     * (avoids NaN in accelerometer normalization)
     */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        /* Normalise accelerometer measurement */
        recipNorm = _inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Normalise magnetometer measurement */
        recipNorm = _inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        /* Auxiliary variables to avoid repeated arithmetic */
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];

        /* Reference direction of Earth's magnetic field */
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        /* Estimated direction of gravity and magnetic field */
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        /*
         * Error is sum of cross product between estimated direction and measured
         * direction of field vectors
         */
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        /* Compute and apply integral feedback if enabled */
        if (ki_2 > 0.0f) {
            fb_i[0] += ki_2 * halfex * delta_time; /* integral error scaled by Ki */
            fb_i[1] += ki_2 * halfey * delta_time;
            fb_i[2] += ki_2 * halfez * delta_time;
            gx += fb_i[0]; /* apply integral feedback */
            gy += fb_i[1];
            gz += fb_i[2];
        } else {
            fb_i[0] = 0.0f; /* prevent integral windup */
            fb_i[1] = 0.0f;
            fb_i[2] = 0.0f;
        }

        /* Apply proportional feedback */
        gx += kp_2 * halfex;
        gy += kp_2 * halfey;
        gz += kp_2 * halfez;
    }

    /* Integrate rate of change of quaternion */
    gx *= 0.5f * delta_time; /* pre-multiply common factors */
    gy *= 0.5f * delta_time;
    gz *= 0.5f * delta_time;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += -qb * gx - qc * gy - q[3] * gz;
    q[1] += qa * gx + qc * gz - q[3] * gy;
    q[2] += qa * gy - qb * gz + q[3] * gx;
    q[3] += qa * gz + qb * gy - qc * gx;

    /* Normalize quaternion */
    recipNorm = _inv_sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

void se_mahony_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float delta_time, float ki_2, float kp_2, float fb_i[3], float q[4]) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /*
*Compute feedback only if accelerometer measurement valid
*(avoids NaN in accelerometer normalisation)
*/
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        /* Normalize accelerometer measurement */
        recipNorm = _inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /*
* Estimated direction of gravity and vector perpendicular to magnetic flux
*/
        halfvx = q[1] * q[3] - q[0] * q[2];
        halfvy = q[0] * q[1] + q[2] * q[3];
        halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

        /*
* Error is sum of cross product between estimated and measured direction
* of gravity
*/
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        /* Compute and apply integral feedback if enabled */
        if (ki_2 > 0.0f) {
            /* integral error scaled by Ki */
            fb_i[0] += ki_2 * halfex * delta_time;
            fb_i[1] += ki_2 * halfey * delta_time;
            fb_i[2] += ki_2 * halfez * delta_time;
            /* apply integral feedback */
            gx += fb_i[0];
            gy += fb_i[1];
            gz += fb_i[2];
        } else {
            /* prevent integral windup */
            fb_i[0] = 0.0f;
            fb_i[1] = 0.0f;
            fb_i[2] = 0.0f;
        }

        /* Apply proportional feedback */
        gx += kp_2 * halfex;
        gy += kp_2 * halfey;
        gz += kp_2 * halfez;
    }

    /* Integrate rate of change of quaternion */
    /* pre-multiply common factors */
    gx *= 0.5f * delta_time;
    gy *= 0.5f * delta_time;
    gz *= 0.5f * delta_time;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += -qb * gx - qc * gy - q[3] * gz;
    q[1] += qa * gx + qc * gz - q[3] * gy;
    q[2] += qa * gy - qb * gz + q[3] * gx;
    q[3] += qa * gz + qb * gy - qc * gx;

    // Normalise quaternion
    recipNorm = _inv_sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

#ifndef SE_NON_IEEE_STANDARD_FLOATS

/*
 * Fast inverse square-root
 * See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
 */

float _inv_sqrt(float x) {
    float halfx = 0.5f * (float)x;
    float y = (float)x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return (float)y;
}

#else

float _inv_sqrt(float x) {
    return 1.0f / sqrt(x);
}

#endif
