#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    MotorControl(int fwdPin, int bwdPin);
    void begin(int freq = 5000, int resolution = 8); // Default frequency and resolution
    void setMotor(int speed, bool forward); // Sets both speed and direction
    void stop();

private:
    int _fwdPin, _bwdPin;
    int _fwdChannel = 0;  // PWM channel for forward
    int _bwdChannel = 1;  // PWM channel for backward
};

#endif // MOTORCONTROL_H
