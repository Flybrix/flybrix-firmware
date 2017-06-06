#include "ahrs.h"

#include <cmath>

void se_mahony_ahrs_update_imu_with_mag(Vector3<float> g, Vector3<float> a, Vector3<float> m, float delta_time, float ki_2, float kp_2, Vector3<float> fb_i, Quaternion<float>& q);

void se_mahony_ahrs_update_imu(Vector3<float> g, Vector3<float> a, float delta_time, float ki_2, float kp_2, Vector3<float> fb_i, Quaternion<float>& q);

float _inv_sqrt(float x);

namespace {
template <typename T>
inline void normalize(T& v) {
    v *= _inv_sqrt(v.lengthSq());
}

inline Quaternion<float> madgwickStepA(const Quaternion<float>& q, Vector3<float> a) {
    normalize(a);
    // q* * [0 gx gy gz] * q; g = [0 0 1]
    Vector3<float> fa{
        2.f * (q.x * q.z - q.w * q.y),        // X
        2.f * (q.w * q.x + q.y * q.z),        // Y
        2.f * (0.5f - q.x * q.x - q.y * q.y)  // Z
    };
    // Subtract accelerometer measure from local frame
    fa -= a;
    // Multiply with transposed Jacobian
    return Quaternion<float>{
        -2.f * q.y * fa.x + 2.f * q.x * fa.y,                     // w
        2.f * q.z * fa.x + 2.f * q.w * fa.y - 4.f * q.x * fa.z,   // x
        -2.f * q.w * fa.x + 2.f * q.z * fa.y - 4.f * q.y * fa.z,  // y
        2.f * q.x * fa.x + 2.f * q.y * fa.y                       // z
    };
}

inline Quaternion<float> madgwickStepM(const Quaternion<float>& q, Vector3<float> m) {
    if (m.isZero()) {
        return Quaternion<float>{0, 0, 0, 0};
    }
    normalize(m);
    Quaternion<float> h = q * m * q.conj();
    if (h.isZero()) {
        return Quaternion<float>{0, 0, 0, 0};
    }
    float b1{2 / _inv_sqrt(h.x * h.x + h.y * h.y)};
    float b2{2 * h.z};
    Vector3<float> fm{
        b1 * (0.5f - q.y * q.y - q.z * q.z) + b2 * (q.x * q.z - q.w * q.y),  // X
        b1 * (q.x * q.y - q.w * q.z) + b2 * (q.w * q.x + q.y * q.z),         // Y
        b1 * (q.w * q.y + q.x * q.z) + b2 * (0.5f - q.x * q.x - q.y * q.y)   // Z
    };
    fm -= m;
    return Quaternion<float>{
        (-b2 * q.y) * fm.x + (-b1 * q.z + b2 * q.x) * fm.y + (b1 * q.y) * fm.z,                                   // W
        (b2 * q.z) * fm.x + (b1 * q.y + b2 * q.w) * fm.y + (b1 * q.z - 2.f * b2 * q.x) * fm.z,                    // X
        (-2.f * b1 * q.y - b2 * q.w) * fm.x + (b1 * q.x + b2 * q.z) * fm.y + (b1 * q.w - 2.f * b2 * q.y) * fm.z,  // Y
        (-2.f * b1 * q.z + b2 * q.x) * fm.x + (-b1 * q.w + b2 * q.y) * fm.y + (b1 * q.x) * fm.z                   // Z
    };
}

inline Quaternion<float> madgwick(Quaternion<float> q, float beta, float dt, const Vector3<float>& g, const Vector3<float>& a, const Vector3<float>& m = {0, 0, 0}) {
    if (a.isZero()) {
        return q;
    }
    Quaternion<float> step = madgwickStepA(q, a) + madgwickStepM(q, m);
    normalize(step);
    Quaternion<float> q_dot = q * (g * 0.5) - step * beta;
    q += q_dot * dt;
    normalize(q);
    return q;
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
                pose_ = madgwick(pose_, parameter_1_, dt, gyroscope_.value, accelerometer_.value, magnetometer_.value);
            } break;
            case Type::Mahony: {
                mahony(pose_, integral_feedback_, parameter_1_, parameter_2_, dt, gyroscope_.value, accelerometer_.value, magnetometer_.value);
            } break;
        }
        magnetometer_.consume();
    } else {
        switch (type_) {
            case Type::Madgwick: {
                pose_ = madgwick(pose_, parameter_1_, dt, gyroscope_.value, accelerometer_.value, {0, 0, 0});
            } break;
            case Type::Mahony: {
                mahony(pose_, integral_feedback_, parameter_1_, parameter_2_, dt, gyroscope_.value, accelerometer_.value);
            } break;
        }
    }
}

/* IMU algorithm update */

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
