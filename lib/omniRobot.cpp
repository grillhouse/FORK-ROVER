#include "omniRobot.h"

// Constructor
OmniRobot::OmniRobot(float wheelDiameter, int stepsPerRevolution)
    : wheelDiameter(wheelDiameter), stepsPerRevolution(stepsPerRevolution) {
    wheelCircumference = M_PI * wheelDiameter; // Calculate wheel circumference
}

// Move the robot a certain distance in a specific direction
void OmniRobot::move(float distance, float angle) {
    calculateWheelSteps(distance, angle);
}

// Getter for reqSteps
int* OmniRobot::getReqSteps() {
    return reqSteps;
}

void OmniRobot::calculateWheelSteps(float distance, float ang) {
    float rad = ang * (M_PI / 180.0); // radians
    float rotations = distance / wheelCircumference; // rotations needed for a straight line to the target
    float stepsStraight = rotations * stepsPerRevolution; // Steps needed for a straight line, will be adjusted for direction and rotation

    // Steps for directional travel
    reqSteps[0] = static_cast<int>(stepsStraight * cos(rad + M_PI / 3));
    reqSteps[1] = static_cast<int>(stepsStraight * cos(rad - M_PI / 3));
    reqSteps[2] = static_cast<int>(stepsStraight * cos(rad - M_PI));
}