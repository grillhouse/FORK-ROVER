#include <ESP32MX1508.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include "HolonomicRobot.h"

HolonomicRobot::HolonomicRobot(int pin1A, int pin1B, int pin2A, int pin2B, int pin3A, int pin3B)
    : _motor1(pin1A, pin1B, 0, 1), _motor2(pin2A, pin2B, 2, 3), _motor3(pin3A, pin3B, 4, 5),
      _pid1(&_input1, &_output1, &_setpoint1, 0, 0, 0, DIRECT),
      _pid2(&_input2, &_output2, &_setpoint2, 0, 0, 0, DIRECT),
      _pid3(&_input3, &_output3, &_setpoint3, 0, 0, 0, DIRECT),
      _lastTime(0) {}

void HolonomicRobot::begin(int enc1A, int enc1B, int enc2A, int enc2B, int enc3A, int enc3B) {
    _encoder1.attachHalfQuad(enc1A, enc1B);
    _encoder2.attachHalfQuad(enc2A, enc2B);
    _encoder3.attachHalfQuad(enc3A, enc3B);
    _pid1.SetMode(AUTOMATIC);
    _pid2.SetMode(AUTOMATIC);
    _pid3.SetMode(AUTOMATIC);
    _lastTime = millis();
}

void HolonomicRobot::moveRobot(float magnitude, float theta, int rot) {
    theta = theta * 0.0174533; // deg to rad
    float vel_x = magnitude * cos(theta); // cartesian 
    float vel_y = magnitude * sin(theta);
    const float sqrt3o2 = 1.0 * sqrt(3) / 2; // v from rcos to to cartesian
    float v1 = -vel_x + rot;
    float v2 = 0.5 * vel_x - sqrt3o2 * vel_y + rot;
    float v3 = 0.5 * vel_x + sqrt3o2 * vel_y + rot;

    _setpoint1 = map(abs(v1), 0, 100, 0, 255) * (v1 < 0 ? -1 : 1);
    _setpoint2 = map(abs(v2), 0, 100, 0, 255) * (v2 < 0 ? -1 : 1);
    _setpoint3 = map(abs(v3), 0, 100, 0, 255) * (v3 < 0 ? -1 : 1);

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - _lastTime) / 1000.0; //secs
    _lastTime = currentTime;

    long encoderCount1 = _encoder1.getCount();
    long encoderCount2 = _encoder2.getCount();
    long encoderCount3 = _encoder3.getCount();

    float speed1 = (encoderCount1 - _prevEncoderCount1) / deltaTime;
    float speed2 = (encoderCount2 - _prevEncoderCount2) / deltaTime;
    float speed3 = (encoderCount3 - _prevEncoderCount3) / deltaTime;

    _prevEncoderCount1 = encoderCount1;
    _prevEncoderCount2 = encoderCount2;
    _prevEncoderCount3 = encoderCount3;

    _input1 = speed1;
    _input2 = speed2;
    _input3 = speed3;

    _pid1.Compute();
    _pid2.Compute();
    _pid3.Compute();

    int motorSpeed1 = constrain(_output1, -255, 255);
    int motorSpeed2 = constrain(_output2, -255, 255);
    int motorSpeed3 = constrain(_output3, -255, 255);

    _motor1.motorGo(motorSpeed1);
    _motor2.motorGo(motorSpeed2);
    _motor3.motorGo(motorSpeed3);
}

void HolonomicRobot::stop() {
    _motor1.motorBrake();
    _motor2.motorBrake();
    _motor3.motorBrake();
}

void HolonomicRobot::setPIDTunings(double kp, double ki, double kd) {
    _pid1.SetTunings(kp, ki, kd);
    _pid2.SetTunings(kp, ki, kd);
    _pid3.SetTunings(kp, ki, kd);
}