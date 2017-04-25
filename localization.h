#ifndef SE_LOCALIZATION_H_
#define SE_LOCALIZATION_H_

#include "utility/vector3.h"

struct IMUState {
    float gyro_drift[3];
    float gravity_filter_weight;
    float gravity;
    float q[4];
    float fb_i[3];
};

/* Filter types */
enum class FilterType { Madgwick = 0, Mahony = 1 };

class Localization {
   public:
    Localization(float deltaTime, FilterType ahrsType, const float* ahrsParameters, float elevationVariance);

    Localization(float q0, float q1, float q2, float q3, float deltaTime, FilterType ahrsType, const float* ahrsParameters, float elevationVariance);

    void ProcessMeasurementElevation(unsigned int time, float elevation);

    void ProcessMeasurementPT(unsigned int time, float p_sl, float p, float t);

    void ProcessMeasurementIMU(unsigned int time, const Vector3<float>& gyroscope, const Vector3<float>& accelerometer);

    void ProcessMeasurementMagnetometer(const Vector3<float>& magnetometer);

    void setTime(unsigned int time);

    void setGravityEstimate(float gravity);

    void setGyroDriftEstimate(float x, float y, float z);

    const float* getAhrsQuaternion() const;

    float getElevation() const;

   private:
    IMUState imuState;
    float z[3];
    float zCovar[9];
    float magLastMeas[3];
    bool hasMagMeas;

    float deltaTime;
    const float* ahrsParameters;
    float elevationVariance;
    FilterType ahrsType;
    unsigned int timeNow;
};

#endif /* end of include guard: SE_LOCALIZATION_H_ */
