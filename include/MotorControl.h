#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    MotorControl(int fwdPin, int bwdPin);
    void begin(int freq = 5000, int resolution = 8);
    void setMotor(int speed, bool forward); 
    void stop();

private:
    int _fwdPin, _bwdPin;
    int _fwdChannel = 0;  
    int _bwdChannel = 1;  
};

#endif 
