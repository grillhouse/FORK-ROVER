#include "omniStepper.h"

// Constructor
OmniStepper::OmniStepper(AccelStepper& stepperLF, AccelStepper& stepperRF, AccelStepper& stepperB)
    : stepperLF(stepperLF), stepperRF(stepperRF), stepperB(stepperB) {}

// Drive the motors simultaneously
void OmniStepper::driveMotors(int steps[3]) {
    long maxSteps = getMaxSteps(steps);

    // Motor position tragets
    stepperLF.move(steps[0]);
    stepperRF.move(steps[1]);
    stepperB.move(steps[2]);

    // Ratio for speed and runtime
    float speedLF = abs(static_cast<float>(steps[0]) / maxSteps);
    float speedRF = abs(static_cast<float>(steps[1]) / maxSteps);
    float speedB = abs(static_cast<float>(steps[2]) / maxSteps);

    // Max speed for each stepper (so runtime is correct)
    stepperLF.setMaxSpeed(speedLF * maxSpeed);
    stepperRF.setMaxSpeed(speedRF * maxSpeed);
    stepperB.setMaxSpeed(speedB * maxSpeed);

    // Run the stepper motors target positions
    while (stepperLF.distanceToGo() != 0 || stepperRF.distanceToGo() != 0 || stepperB.distanceToGo() != 0) {
        stepperLF.run();
        stepperRF.run();
        stepperB.run();
    }
}

// Helper function to get the maximum number of steps among the three motors
long OmniStepper::getMaxSteps(int steps[3]) {
    return *std::max_element(steps, steps + 3);
}