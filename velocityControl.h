#ifndef velocity_control_h
#define velocity_control_h

#include "utility/vector3.h"
#include "PID.h"

struct RcCommand;

class VelocityControl final {
   public:
    struct PIDParameters;

    explicit VelocityControl(const PIDParameters& config);
    void parseConfig();

    RcCommand calculateControlVectors(const Vector3<float>& velocity, RcCommand setpoint);

    struct __attribute__((packed)) PIDParameters {
        PIDParameters();
        bool verify() const {
            return true;
        }

        float vx[7];  // parameters are {P,I,D,integral windup guard, D filter delay sec, setpoint filter delay sec, command scaling factor}
        float vy[7];
        float vz[7];

        bool bypass;
    } pid_parameters;

    static_assert(sizeof(PIDParameters) == 4 * 3 * 7 + 1, "Data is not packed");

   private:
    PID vx_pid, vy_pid, vz_pid;
    bool bypass;
};

#endif  // velocity_control_h
