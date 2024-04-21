#ifndef HOLONOMICROBOT_H
#define HOLONOMICROBOT_H

#include "MotorControl.h"
#include <ESP32Encoder.h>

class HolonomicRobot : public MotorControl {
public:
    HolonomicRobot(int fwdPin1, int bwdPin1, int fwdPin2, int bwdPin2, int fwdPin3, int bwdPin3);
    void begin(int freq = 30000, int resolution = 8);
    void moveRobot(float magnitude, float theta, int rot);
    void attachEncoder1(int dt, int clk);
    void attachEncoder2(int dt, int clk);
    void attachEncoder3(int dt, int clk);
    long getEncoder1Count();
    long getEncoder2Count();
    long getEncoder3Count();
        void setPIDTunings(float kp, float ki, float kd);
    void updatePID();

private:
    int _fwdPin2;
    int _bwdPin2;
    int _fwdPin3;
    int _bwdPin3;
    int _fwdChannel2;
    int _bwdChannel2;
    int _fwdChannel3;
    int _bwdChannel3;
    ESP32Encoder _encoder1;
    ESP32Encoder _encoder2;
    ESP32Encoder _encoder3;

        float _kp;
    float _ki;
    float _kd;
    long _prevError1;
    long _prevError2;
    long _prevError3;
    long _integral1;
    long _integral2;
    long _integral3;
};

#endif