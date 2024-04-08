#ifndef OMNIROBOT_H
#define OMNIROBOT_H

#include <cmath>
#include <Arduino.h>

class OmniRobot {
public:
    // Constructor
    OmniRobot(float wheelDiameter, int stepsPerRevolution);

    //move to: position vector
    void move(float distance, float angle);

    // Getter for reqSteps
    int* getReqSteps();

private:
    float wheelDiameter;      // diameter in m
    int stepsPerRevolution;   // steps per rev of stepper
    float wheelCircumference; // Wheel Circ
    int reqSteps[3];          // req steps array left front = 0, right front = 1, back = 2

    void calculateWheelSteps(float distance, float ang);
};

#endif // OMNIROBOT_H