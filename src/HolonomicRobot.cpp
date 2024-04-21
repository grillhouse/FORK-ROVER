#include "HolonomicRobot.h"
#include <Arduino.h>

HolonomicRobot::HolonomicRobot(int fwdPin1, int bwdPin1, int fwdPin2, int bwdPin2, int fwdPin3, int bwdPin3)
    : MotorControl(fwdPin1, bwdPin1), _fwdPin2(fwdPin2), _bwdPin2(bwdPin2), _fwdPin3(fwdPin3), _bwdPin3(bwdPin3) {
    _fwdChannel2 = 2;
    _bwdChannel2 = 3;
    _fwdChannel3 = 4;
    _bwdChannel3 = 5;
}

void HolonomicRobot::begin(int freq, int resolution) {
    MotorControl::begin(freq, resolution);
    ledcSetup(_fwdChannel2, freq, resolution);
    ledcAttachPin(_fwdPin2, _fwdChannel2);
    ledcSetup(_bwdChannel2, freq, resolution);
    ledcAttachPin(_bwdPin2, _bwdChannel2);
    ledcSetup(_fwdChannel3, freq, resolution);
    ledcAttachPin(_fwdPin3, _fwdChannel3);
    ledcSetup(_bwdChannel3, freq, resolution);
    ledcAttachPin(_bwdPin3, _bwdChannel3);
}

void HolonomicRobot::moveRobot(float magnitude, float theta, int rot) {
    theta = theta * 0.0174533; // deg to rad
    float vel_x = magnitude * cos(theta); // cartesian coordinates
    float vel_y = magnitude * sin(theta);

    const float sqrt3o2 = 1.0 * sqrt(3) / 2; // Vel xy from polar to cartesian
    float v1 = -vel_x + rot;
    float v2 = 0.5 * vel_x - sqrt3o2 * vel_y + rot;
    float v3 = 0.5 * vel_x + sqrt3o2 * vel_y + rot;

    int d1 = v1 < 0 ? -1 : 1;   // distance or velocity
    int d2 = v2 < 0 ? -1 : 1;
    int d3 = v3 < 0 ? -1 : 1;

    v1 = map(abs(v1), 0, 100, 0, 255);
    v2 = map(abs(v2), 0, 100, 0, 255);
    v3 = map(abs(v3), 0, 100, 0, 255);

    setMotor(v1, d1 > 0);
    ledcWrite(_fwdChannel2, d2 > 0 ? v2 : 0); // ESP32 analog out
    ledcWrite(_bwdChannel2, d2 < 0 ? v2 : 0);
    ledcWrite(_fwdChannel3, d3 > 0 ? v3 : 0);
    ledcWrite(_bwdChannel3, d3 < 0 ? v3 : 0);
}

//encoders counting below


void HolonomicRobot::attachEncoder1(int dt, int clk) {
    _encoder1.attachHalfQuad(dt, clk);
    _encoder1.setCount(0);
}

void HolonomicRobot::attachEncoder2(int dt, int clk) {
    _encoder2.attachHalfQuad(dt, clk);
    _encoder2.setCount(0);
}

void HolonomicRobot::attachEncoder3(int dt, int clk) {
    _encoder3.attachHalfQuad(dt, clk);
    _encoder3.setCount(0);
}

long HolonomicRobot::getEncoder1Count() {
    return _encoder1.getCount() / 2;
}

long HolonomicRobot::getEncoder2Count() {
    return _encoder2.getCount() / 2;
}

long HolonomicRobot::getEncoder3Count() {
    return _encoder3.getCount() / 2;
}

// PID stuff below
void HolonomicRobot::setPIDTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void HolonomicRobot::updatePID() {
    long currentCount1 = _encoder1.getCount();
    long currentCount2 = _encoder2.getCount();
    long currentCount3 = _encoder3.getCount();

    long error1 = currentCount1 - getEncoder1Count();
    long error2 = currentCount2 - getEncoder2Count();
    long error3 = currentCount3 - getEncoder3Count();

    _integral1 += error1;
    _integral2 += error2;
    _integral3 += error3;

    long derivative1 = error1 - _prevError1;
    long derivative2 = error2 - _prevError2;
    long derivative3 = error3 - _prevError3;

    int speed1 = _kp * error1 + _ki * _integral1 + _kd * derivative1;
    int speed2 = _kp * error2 + _ki * _integral2 + _kd * derivative2;
    int speed3 = _kp * error3 + _ki * _integral3 + _kd * derivative3;

    setMotor(speed1, speed1 > 0);
    ledcWrite(_fwdChannel2, speed2 > 0 ? speed2 : 0);
    ledcWrite(_bwdChannel2, speed2 < 0 ? -speed2 : 0);
    ledcWrite(_fwdChannel3, speed3 > 0 ? speed3 : 0);
    ledcWrite(_bwdChannel3, speed3 < 0 ? -speed3 : 0);

    _prevError1 = error1;
    _prevError2 = error2;
    _prevError3 = error3;
}