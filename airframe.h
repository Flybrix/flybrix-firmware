/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <airframe.h/cpp>

    Airframe translates the four control vectors (thrust force, pitch torque, roll torque, yaw torque) into Motor Levels
*/

#ifndef airframe_h
#define airframe_h

#include <cstdint>
#include "motors.h"

class ControlVectors;

class Airframe {
   public:
    void setMotor(size_t index, uint16_t value);
    void resetMotors();
    void enableMotors();
    void disableMotors();
    void setOverride(bool override);
    void applyChanges(const ControlVectors& control);

    bool motorsEnabled() const;
    bool motorsOverridden() const;

    template <typename Tstream>
    bool readMotor(size_t index, Tstream& input) {
        uint16_t buffer;
        if (!input.ParseInto(buffer)) {
            return false;
        }
        setMotor(index, buffer);
        return true;
    }

    template <typename Tstream>
    void writeMotorsTo(Tstream& output) {
        motors_.writeTo(output);
    }

    struct __attribute__((packed)) MixTable {
        MixTable();
        bool verify() const {
            return true;
        }
        int8_t fz[8];
        int8_t tx[8];
        int8_t ty[8];
        int8_t tz[8];
    } mix_table;

    static_assert(sizeof(MixTable) == 4 * 8, "Data is not packed");

   private:
    void setMotorsToMixTable(const ControlVectors& control);

    bool enabled_{false};
    bool override_{false};
    Motors motors_;
};

#endif
