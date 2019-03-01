/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#ifndef SE_LOCALIZATION_H_
#define SE_LOCALIZATION_H_

#include "ahrs.h"
#include "ukf.h"
#include "ClockTime.h"
#include "vector3.h"

class Localization {
   public:
    Localization(float deltaTime, Ahrs::Type ahrsType, const float* ahrsParameters, float elevationVariance);

    Localization(float q0, float q1, float q2, float q3, float deltaTime, Ahrs::Type ahrsType, const float* ahrsParameters, float elevationVariance);

    void ProcessMeasurementElevation(float elevation);

    void ProcessMeasurementPT(float p_sl, float p, float t);

    void ProcessMeasurementIMU(ClockTime time, const Vector3<float>& gyroscope, const Vector3<float>& accelerometer);

    void ProcessMeasurementMagnetometer(const Vector3<float>& magnetometer);

    void setTime(ClockTime time);

    void setGravityEstimate(float gravity);

    void setGyroDriftEstimate(float x, float y, float z);

    void predictFilter(ClockTime time);

    void updateFilter();

    const Quaternion<float>& getAhrsQuaternion() const;

    float getElevation() const;

    Vector3<float> getVelocity() const;

   private:
    Ahrs ahrs_;
    UKF ukf_;
    Vector3<float> gyro_drift_;
    UKF::Measurement vu_;
    UKF::Measurement vv_;
    UKF::Measurement d_tof_;
    UKF::Measurement h_bar_;
    bool has_measurement_{false};
    float gravity_force_;
    float max_delta_time_;

    const float* ahrsParameters;
    float elevation_variance_;
    ClockTime timeNow;
};

#endif /* end of include guard: SE_LOCALIZATION_H_ */
