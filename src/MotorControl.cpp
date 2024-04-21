#include "MotorControl.h"

MotorControl::MotorControl(int fwdPin, int bwdPin)
    : _fwdPin(fwdPin), _bwdPin(bwdPin) {
}

void MotorControl::begin(int freq, int resolution) {
    ledcSetup(_fwdChannel, freq, resolution);
    ledcAttachPin(_fwdPin, _fwdChannel);
    ledcSetup(_bwdChannel, freq, resolution);
    ledcAttachPin(_bwdPin, _bwdChannel);
}

void MotorControl::setMotor(int speed, bool forward) {
    if (forward) {
        digitalWrite(_fwdPin, HIGH);
        digitalWrite(_bwdPin, LOW);
        ledcWrite(_fwdChannel, speed);
        ledcWrite(_bwdChannel, 0);
    } else {
        digitalWrite(_fwdPin, LOW);
        digitalWrite(_bwdPin, HIGH);
        ledcWrite(_bwdChannel, speed);
        ledcWrite(_fwdChannel, 0);
    }
}

void MotorControl::stop() {
    ledcWrite(_fwdChannel, 0);
    ledcWrite(_bwdChannel, 0);
}
