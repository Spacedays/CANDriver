#include "MotionControl.h"

// RoverState::RoverState()
// {

// }

void RoverState::SetVector(MotionVector *mvec)
{
    targetVec = *mvec;
}

/// @brief Calculates next iterpolation to the target. Does not command servos or drivers, just updates target
/// @return
void RoverState::Update()
{
    if (targetVec == driveVec)
        return;

    unsigned long elapsed_t = millis() - last_t;
    MotionVector deltaVec = targetVec - driveVec;
    uint16_t max_angle_delta = max(max(abs(deltaVec.aFL), abs(deltaVec.aFR)), max(abs(deltaVec.aBL), abs(deltaVec.aBR)));

    // scale down output
    // TODO: check if velocity should be multiplied here or just angles
    if (max_angle_delta / (elapsed_t * 1000) > SERVO_DT_MAX)
    {
        float angle_scaler = (elapsed_t * 1000) / max_angle_delta;
        // deltaVec = deltaVec % angle_scaler;   // overridden; angle multiplication only
        deltaVec = deltaVec * angle_scaler;
    }

    driveVec = driveVec + deltaVec;

    // TODO: do some bounds checking here
}

int RoverState::InitServos(uint8_t pinFL, uint8_t pinFR, uint8_t pinBL, uint8_t pinBR)
{
    const int min = 0;
    const int max = 180;
    int returns = 0;

    returns = driveServos.FL.attach(pinFL, min, max, driveOffsets.FL);
    returns |= driveServos.FR.attach(pinFR, min, max, driveOffsets.FR);
    returns |= driveServos.BL.attach(pinBL, min, max, driveOffsets.BL);
    returns |= driveServos.BR.attach(pinBR, min, max, driveOffsets.BR);
    if (returns == 0)   // FIXME: confirm correct return code
        servos_enabled = true;
    return returns;
}

void RoverState::CommandServos()
{
    if (!servos_enabled)
        return;
    driveServos.FL.write(driveVec.aFL+driveOffsets.FL);
    driveServos.FR.write(driveVec.aFR+driveOffsets.FR);
    driveServos.BL.write(driveVec.aBL+driveOffsets.BL);
    driveServos.BR.write(driveVec.aBR+driveOffsets.BR);
}
