#include "ahrs.h"

#include <cmath>

void se_madgwick_ahrs_update_imu_with_mag(const Vector3<float>& g, Vector3<float> a, Vector3<float> m, float delta_time, float beta, Quaternion<float>& q);

void se_madgwick_ahrs_update_imu(const Vector3<float>& g, Vector3<float> a, float delta_time, float beta, Quaternion<float>& q);

void se_mahony_ahrs_update_imu_with_mag(Vector3<float> g, Vector3<float> a, Vector3<float> m, float delta_time, float ki_2, float kp_2, Vector3<float> fb_i, Quaternion<float>& q);

void se_mahony_ahrs_update_imu(Vector3<float> g, Vector3<float> a, float delta_time, float ki_2, float kp_2, Vector3<float> fb_i, Quaternion<float>& q);

namespace {
inline void madgwick(Quaternion<float>& q, float beta, float dt, const Vector3<float>& g, const Vector3<float>& a, const Vector3<float>& m) {
    se_madgwick_ahrs_update_imu_with_mag(g, a, m, dt, beta, q);
}
inline void madgwick(Quaternion<float>& q, float beta, float dt, const Vector3<float>& g, const Vector3<float>& a) {
    se_madgwick_ahrs_update_imu(g, a, dt, beta, q);
}
inline void mahony(Quaternion<float>& q, Vector3<float>& ifb, float ki, float kp, float dt, const Vector3<float>& g, const Vector3<float>& a, const Vector3<float>& m) {
    se_mahony_ahrs_update_imu_with_mag(g, a, m, dt, ki, kp, ifb, q);
}
inline void mahony(Quaternion<float>& q, Vector3<float>& ifb, float ki, float kp, float dt, const Vector3<float>& g, const Vector3<float>& a) {
    se_mahony_ahrs_update_imu(g, a, dt, ki, kp, ifb, q);
}
}

void Ahrs::update(uint32_t timestamp) {
    if (!accelerometer_.ready || !gyroscope_.ready) {
        return;
    }

    if (timestamp <= last_update_timestamp_) {
        last_update_timestamp_ = timestamp;
        return;
    }

    float dt = (timestamp - last_update_timestamp_) / 1000000.0f;
    last_update_timestamp_ = timestamp;

    if (dt > max_delta_time_) {
        dt = max_delta_time_;
    }

    accelerometer_.consume();
    gyroscope_.consume();

    if (magnetometer_.ready) {
        switch (type_) {
            case Type::Madgwick: {
                madgwick(pose_, parameter_1_, dt, gyroscope_.value, accelerometer_.value, magnetometer_.value);
            } break;
            case Type::Mahony: {
                mahony(pose_, integral_feedback_, parameter_1_, parameter_2_, dt, gyroscope_.value, accelerometer_.value, magnetometer_.value);
            } break;
        }
        magnetometer_.consume();
    } else {
        switch (type_) {
            case Type::Madgwick: {
                madgwick(pose_, parameter_1_, dt, gyroscope_.value, accelerometer_.value);
            } break;
            case Type::Mahony: {
                mahony(pose_, integral_feedback_, parameter_1_, parameter_2_, dt, gyroscope_.value, accelerometer_.value);
            } break;
        }
    }
}

float _inv_sqrt(float x);

/* IMU algorithm update */

void se_madgwick_ahrs_update_imu_with_mag(const Vector3<float>& g, Vector3<float> a, Vector3<float> m, float delta_time, float beta, Quaternion<float>& q) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    /*
     * Use IMU algorithm if magnetometer measurement is invalid
     * (avoids NaN in magnetometer normalization)
     */
    if ((m.x == 0.0f) && (m.y == 0.0f) && (m.z == 0.0f)) {
        se_madgwick_ahrs_update_imu(g, a, delta_time, beta, q);
        return;
    }

    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-q.x * g.x - q.y * g.y - q.z * g.z);
    qDot2 = 0.5f * (q.w * g.x + q.y * g.z - q.z * g.y);
    qDot3 = 0.5f * (q.w * g.y - q.x * g.z + q.z * g.x);
    qDot4 = 0.5f * (q.w * g.z + q.x * g.y - q.y * g.x);

    /*
     * Compute feedback only if accelerometer measurement is valid
     * (avoids NaN in accelerometer normalization)
     */
    if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {
        /* Normalize accelerometer measurement */
        a *= _inv_sqrt(a.lengthSq());

        /* Normalize magnetometer measurement */
        m *= _inv_sqrt(m.lengthSq());

        /* Auxiliary variables to avoid repeated arithmetic */
        _2q0mx = 2.0f * q.w * m.x;
        _2q0my = 2.0f * q.w * m.y;
        _2q0mz = 2.0f * q.w * m.z;
        _2q1mx = 2.0f * q.x * m.x;
        _2q0 = 2.0f * q.w;
        _2q1 = 2.0f * q.x;
        _2q2 = 2.0f * q.y;
        _2q3 = 2.0f * q.z;
        _2q0q2 = 2.0f * q.w * q.y;
        _2q2q3 = 2.0f * q.y * q.z;
        q0q0 = q.w * q.w;
        q0q1 = q.w * q.x;
        q0q2 = q.w * q.y;
        q0q3 = q.w * q.z;
        q1q1 = q.x * q.x;
        q1q2 = q.x * q.y;
        q1q3 = q.x * q.z;
        q2q2 = q.y * q.y;
        q2q3 = q.y * q.z;
        q3q3 = q.z * q.z;

        /* Reference direction of Earth's magnetic field */
        hx = m.x * q0q0 - _2q0my * q.z + _2q0mz * q.y + m.x * q1q1 + _2q1 * m.y * q.y + _2q1 * m.z * q.z - m.x * q2q2 - m.x * q3q3;
        hy = _2q0mx * q.z + m.y * q0q0 - _2q0mz * q.x + _2q1mx * q.y - m.y * q1q1 + m.y * q2q2 + _2q2 * m.z * q.z - m.y * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q.y + _2q0my * q.x + m.z * q0q0 + _2q1mx * q.z - m.z * q1q1 + _2q2 * m.y * q.z - m.z * q2q2 + m.z * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        /* Gradient decent algorithm corrective step */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q1 * (2.0f * q0q1 + _2q2q3 - a.y) - _2bz * q.y * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) +
             (-_2bx * q.z + _2bz * q.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + _2bx * q.y * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q0 * (2.0f * q0q1 + _2q2q3 - a.y) - 4.0f * q.x * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a.z) +
             _2bz * q.z * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (_2bx * q.y + _2bz * q.w) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) +
             (_2bx * q.z - _4bz * q.x) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q3 * (2.0f * q0q1 + _2q2q3 - a.y) - 4.0f * q.y * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a.z) +
             (-_4bx * q.y - _2bz * q.w) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (_2bx * q.x + _2bz * q.z) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) +
             (_2bx * q.w - _4bz * q.y) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q2 * (2.0f * q0q1 + _2q2q3 - a.y) + (-_4bx * q.z + _2bz * q.x) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) +
             (-_2bx * q.w + _2bz * q.y) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + _2bx * q.x * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
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
    q.w += qDot1 * delta_time;
    q.x += qDot2 * delta_time;
    q.y += qDot3 * delta_time;
    q.z += qDot4 * delta_time;

    /* Normalize quaternion */
    recipNorm = _inv_sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
}

void se_madgwick_ahrs_update_imu(const Vector3<float>& g, Vector3<float> a, float delta_time, float beta, Quaternion<float>& q) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-q.x * g.x - q.y * g.y - q.z * g.z);
    qDot2 = 0.5f * (q.w * g.x + q.y * g.z - q.z * g.y);
    qDot3 = 0.5f * (q.w * g.y - q.x * g.z + q.z * g.x);
    qDot4 = 0.5f * (q.w * g.z + q.x * g.y - q.y * g.x);

    /*
*Compute feedback only if accelerometer measurement valid
*(avoids NaN in accelerometer normalisation)
*/
    if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {
        // Normalize accelerometer measurement
        a *= _inv_sqrt(a.lengthSq());

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q.w;
        _2q1 = 2.0f * q.x;
        _2q2 = 2.0f * q.y;
        _2q3 = 2.0f * q.z;
        _4q0 = 4.0f * q.w;
        _4q1 = 4.0f * q.x;
        _4q2 = 4.0f * q.y;
        _8q1 = 8.0f * q.x;
        _8q2 = 8.0f * q.y;
        q0q0 = q.w * q.w;
        q1q1 = q.x * q.x;
        q2q2 = q.y * q.y;
        q3q3 = q.z * q.z;

        /* Gradient decent algorithm corrective step */
        s0 = _4q0 * q2q2 + _2q2 * a.x + _4q0 * q1q1 - _2q1 * a.y;
        s1 = _4q1 * q3q3 - _2q3 * a.x + 4.0f * q0q0 * q.x - _2q0 * a.y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a.z;
        s2 = 4.0f * q0q0 * q.y + _2q0 * a.x + _4q2 * q3q3 - _2q3 * a.y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a.z;
        s3 = 4.0f * q1q1 * q.z - _2q1 * a.x + 4.0f * q2q2 * q.z - _2q2 * a.y;
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
    q.w += qDot1 * delta_time;
    q.x += qDot2 * delta_time;
    q.y += qDot3 * delta_time;
    q.z += qDot4 * delta_time;

    /* Normalize quaternion */
    recipNorm = _inv_sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
}

void se_mahony_ahrs_update_imu_with_mag(Vector3<float> g, Vector3<float> a, Vector3<float> m, float delta_time, float ki_2, float kp_2, Vector3<float> fb_i, Quaternion<float>& q) {
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
    if ((m.x == 0.0f) && (m.y == 0.0f) && (m.z == 0.0f)) {
        se_mahony_ahrs_update_imu(g, a, delta_time, ki_2, kp_2, fb_i, q);
        return;
    }

    /*
     * Compute feedback only if accelerometer measurement is valid
     * (avoids NaN in accelerometer normalization)
     */
    if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {
        /* Normalize accelerometer measurement */
        a *= _inv_sqrt(a.lengthSq());

        /* Normalize magnetometer measurement */
        m *= _inv_sqrt(m.lengthSq());

        /* Auxiliary variables to avoid repeated arithmetic */
        q0q0 = q.w * q.w;
        q0q1 = q.w * q.x;
        q0q2 = q.w * q.y;
        q0q3 = q.w * q.z;
        q1q1 = q.x * q.x;
        q1q2 = q.x * q.y;
        q1q3 = q.x * q.z;
        q2q2 = q.y * q.y;
        q2q3 = q.y * q.z;
        q3q3 = q.z * q.z;

        /* Reference direction of Earth's magnetic field */
        hx = 2.0f * (m.x * (0.5f - q2q2 - q3q3) + m.y * (q1q2 - q0q3) + m.z * (q1q3 + q0q2));
        hy = 2.0f * (m.x * (q1q2 + q0q3) + m.y * (0.5f - q1q1 - q3q3) + m.z * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (m.x * (q1q3 - q0q2) + m.y * (q2q3 + q0q1) + m.z * (0.5f - q1q1 - q2q2));

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
        halfex = (a.y * halfvz - a.z * halfvy) + (m.y * halfwz - m.z * halfwy);
        halfey = (a.z * halfvx - a.x * halfvz) + (m.z * halfwx - m.x * halfwz);
        halfez = (a.x * halfvy - a.y * halfvx) + (m.x * halfwy - m.y * halfwx);

        /* Compute and apply integral feedback if enabled */
        if (ki_2 > 0.0f) {
            fb_i.x += ki_2 * halfex * delta_time; /* integral error scaled by Ki */
            fb_i.y += ki_2 * halfey * delta_time;
            fb_i.z += ki_2 * halfez * delta_time;
            g += fb_i; /* apply integral feedback */
        } else {
            fb_i = Vector3<float>(); /* prevent integral windup */
        }

        /* Apply proportional feedback */
        g.x += kp_2 * halfex;
        g.y += kp_2 * halfey;
        g.z += kp_2 * halfez;
    }

    /* Integrate rate of change of quaternion */
    g *= 0.5f * delta_time; /* pre-multiply common factors */
    qa = q.w;
    qb = q.x;
    qc = q.y;
    q.w += -qb * g.x - qc * g.y - q.z * g.z;
    q.x += qa * g.x + qc * g.z - q.z * g.y;
    q.y += qa * g.y - qb * g.z + q.z * g.x;
    q.z += qa * g.z + qb * g.y - qc * g.x;

    /* Normalize quaternion */
    recipNorm = _inv_sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
}

void se_mahony_ahrs_update_imu(Vector3<float> g, Vector3<float> a, float delta_time, float ki_2, float kp_2, Vector3<float> fb_i, Quaternion<float>& q) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /*
*Compute feedback only if accelerometer measurement valid
*(avoids NaN in accelerometer normalisation)
*/
    if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {
        /* Normalize accelerometer measurement */
        a *= _inv_sqrt(a.lengthSq());

        /*
* Estimated direction of gravity and vector perpendicular to magnetic flux
*/
        halfvx = q.x * q.z - q.w * q.y;
        halfvy = q.w * q.x + q.y * q.z;
        halfvz = q.w * q.w - 0.5f + q.z * q.z;

        /*
* Error is sum of cross product between estimated and measured direction
* of gravity
*/
        halfex = (a.y * halfvz - a.z * halfvy);
        halfey = (a.z * halfvx - a.x * halfvz);
        halfez = (a.x * halfvy - a.y * halfvx);

        /* Compute and apply integral feedback if enabled */
        if (ki_2 > 0.0f) {
            /* integral error scaled by Ki */
            fb_i.x += ki_2 * halfex * delta_time;
            fb_i.y += ki_2 * halfey * delta_time;
            fb_i.z += ki_2 * halfez * delta_time;
            /* apply integral feedback */
            g += fb_i;
        } else {
            /* prevent integral windup */
            fb_i.x = 0.0f;
            fb_i.y = 0.0f;
            fb_i.z = 0.0f;
        }

        /* Apply proportional feedback */
        g.x += kp_2 * halfex;
        g.y += kp_2 * halfey;
        g.z += kp_2 * halfez;
    }

    /* Integrate rate of change of quaternion */
    /* pre-multiply common factors */
    g *= 0.5f * delta_time;
    qa = q.w;
    qb = q.x;
    qc = q.y;
    q.w += -qb * g.x - qc * g.y - q.z * g.z;
    q.x += qa * g.x + qc * g.z - q.z * g.y;
    q.y += qa * g.y - qb * g.z + q.z * g.x;
    q.z += qa * g.z + qb * g.y - qc * g.x;

    // Normalize quaternion
    recipNorm = _inv_sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
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
