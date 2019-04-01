/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef SE_AHRS_H_
#define SE_AHRS_H_

#include <cstdint>
#include "ClockTime.h"
#include "quaternion.h"
#include "vector3.h"

class Ahrs final {
   public:
    enum class Type {
        Madgwick = 0,
        Mahony = 1,
    };

    Ahrs& setType(Type type);
    Ahrs& setParameters(float p1, float p2);
    Ahrs& setMaxDeltaTime(float mdt);
    Ahrs& setTimestamp(ClockTime time);
    Ahrs& setAccelerometer(float x, float y, float z);
    Ahrs& setAccelerometer(const Vector3<float>& v);
    Ahrs& setGyroscope(float x, float y, float z);
    Ahrs& setGyroscope(const Vector3<float>& v);
    Ahrs& setMagnetometer(float x, float y, float z);
    Ahrs& setMagnetometer(const Vector3<float>& v);
    Quaternion<float>& pose();
    Vector3<float> gravity();
    void update(ClockTime timestamp);

    const Quaternion<float>& pose() const;

   private:
    class Measurement {
       public:
        void set(const Vector3<float>& v) {
            value_ = v;
            ready_ = true;
        }

        const Vector3<float>& consume() {
            ready_ = false;
            return value_;
        }

        bool isReady() const {
            return ready_;
        }

       private:
        Vector3<float> value_;
        bool ready_{false};
    };
    Quaternion<float> pose_;
    Vector3<float> integral_feedback_;
    Type type_{Type::Madgwick};
    Measurement accelerometer_;
    Measurement gyroscope_;
    Measurement magnetometer_;
    float max_delta_time_{0};
    float parameter_1_{0};
    float parameter_2_{0};
    ClockTime last_update_timestamp_{ClockTime::zero()};
};

#endif /* end of include guard: SE_AHRS_H_ */
