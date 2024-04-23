#include <Arduino.h>
#include <ESP32MX1508.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include "HolonomicRobot.h"


const int PIN_MOTOR1A = 33;
const int PIN_MOTOR1B = 25;
const int PIN_MOTOR2A = 26;
const int PIN_MOTOR2B = 27;
const int PIN_MOTOR3A = 14;
const int PIN_MOTOR3B = 12;
const int PIN_ENC1A = 35;
const int PIN_ENC1B = 32;
const int PIN_ENC2A = 39;
const int PIN_ENC2B = 34;
const int PIN_ENC3A = 4;
const int PIN_ENC3B = 36;


HolonomicRobot robot(PIN_MOTOR1A, PIN_MOTOR1B, PIN_MOTOR2A, PIN_MOTOR2B, PIN_MOTOR3A, PIN_MOTOR3B);

void setup() {
   
    robot.begin(PIN_ENC1A, PIN_ENC1B, PIN_ENC2A, PIN_ENC2B, PIN_ENC3A, PIN_ENC3B);


    double kp = 2.0;
    double ki = 5.0;
    double kd = 1.0;
    robot.setPIDTunings(kp, ki, kd);
}

void loop() {


/* Taken all distance measuring logic out for now. The distance measuring class will be added with another PID
That will measure the distance and direction. Currently finding it difficult to do with rotation, but need to back encoder
 read back through the same maths... Speed will also be 
implemented with calcs of circumferenc e*/

// different movements to test the robot... 

    robot.moveRobot(200, 0, 0); // magnitude = 200, theta = 0, rotATION = 0   Position vector with rotation proving difficult
    delay(1000);


    robot.stop();
    delay(1000);


    robot.moveRobot(200, 180, 0);
    delay(1000);


    robot.stop();
    delay(1000);

    robot.moveRobot(0, 0, 150);
    delay(1000);


    robot.stop();
    delay(1000);

    robot.moveRobot(0, 0, -150);
    delay(1000);

    robot.stop();
    delay(1000);

    robot.moveRobot(200, 45, 0);
    delay(1000);

    robot.stop();
    delay(1000);
}