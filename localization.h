#ifndef SE_LOCALIZATION_H_
#define SE_LOCALIZATION_H_

#include "ahrs.h"
#include "utility/vector3.h"

class Localization {
   public:
    Localization(float deltaTime, Ahrs::Type ahrsType, const float* ahrsParameters, float elevationVariance);

    Localization(float q0, float q1, float q2, float q3, float deltaTime, Ahrs::Type ahrsType, const float* ahrsParameters, float elevationVariance);

    void ProcessMeasurementElevation(unsigned int time, float elevation);

    void ProcessMeasurementPT(unsigned int time, float p_sl, float p, float t);

    void ProcessMeasurementIMU(unsigned int time, const Vector3<float>& gyroscope, const Vector3<float>& accelerometer);

    void ProcessMeasurementMagnetometer(const Vector3<float>& magnetometer);

    void setTime(unsigned int time);

    void setGravityEstimate(float gravity);

    void setGyroDriftEstimate(float x, float y, float z);

    const Quaternion<float>& getAhrsQuaternion() const;

    float getElevation() const;

   private:
    Ahrs ahrs_;
    Vector3<float> gyro_drift_;
    float gravity_force_;
    float z[3];
    float zCovar[9];

    const float* ahrsParameters;
    float elevationVariance;
    unsigned int timeNow;
};

#endif /* end of include guard: SE_LOCALIZATION_H_ */
