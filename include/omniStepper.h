#ifndef OMNISTEPPER_H
#define OMNISTEPPER_H

#include <AccelStepper.h>
#include <algorithm>

class OmniStepper {
public:
    // Constructor
    OmniStepper(AccelStepper& stepperLF, AccelStepper& stepperRF, AccelStepper& stepperB);

    // Drive the motors at same time
    void driveMotors(int steps[3]);

private:
    AccelStepper& stepperLF; // Left front stepper motor
    AccelStepper& stepperRF; // Right front stepper motor
    AccelStepper& stepperB;  // Back stepper motor

    const float maxSpeed = 1000.0; // Motor Speed

    /// max step helper
    long getMaxSteps(int steps[3]);
};

#endif // OMNISTEPPER_H