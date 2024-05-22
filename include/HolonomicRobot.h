#ifndef HOLONOMICROBOT_H
#define HOLONOMICROBOT_H

#include <ESP32MX1508.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>

class HolonomicRobot {
public:
    HolonomicRobot(int pin1A, int pin1B, int pin2A, int pin2B, int pin3A, int pin3B);
    void begin(int enc1A, int enc1B, int enc2A, int enc2B, int enc3A, int enc3B);
    void moveRobot(float magnitude, float theta, int rot);
    void stop();
    void setPIDTunings(double kp, double ki, double kd);

private:
    // Motor objects
    ESP32MX1508 _motor1;
    ESP32MX1508 _motor2;
    ESP32MX1508 _motor3;

    // PID control variables
    double _input1, _output1, _setpoint1;
    double _input2, _output2, _setpoint2;
    double _input3, _output3, _setpoint3;
    PID _pid1;
    PID _pid2;
    PID _pid3;

    // Encoder objects
    ESP32Encoder _encoder1;
    ESP32Encoder _encoder2;
    ESP32Encoder _encoder3;

    // Previous encoder counts
    long _prevEncoderCount1;
    long _prevEncoderCount2;
    long _prevEncoderCount3;

    // Last time the moveRobot function was called
    unsigned long _lastTime;

    // Helper function for mapping float values
    static float fmap(float x, float in_min, float in_max, float out_min, float out_max);
};

#endif // HOLONOMICROBOT_H
