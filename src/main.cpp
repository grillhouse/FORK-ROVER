#include <Arduino.h>
#include <ESP32MX1508.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include "OmniDrive.h"
 #include <Wire.h>
 #include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Motor pins
const int motor1A = 23; //brown 
const int motor1B = 27;  //white 
const int motor2A = 4;
const int motor2B = 16;
const int motor3A = 26;
const int motor3B = 25;

const int motorArmA = 12;
const int motorArmB = 13;

// Encoder pins
const int encoder1A = 19;   //orange
const int encoder1B = 14;    //white  (I moved this wire for the I2c Port access)
const int encoder2A = 15;
const int encoder2B = 2;
const int encoder3A = 33;
const int encoder3B = 32;

// Motor and Encoder objects
MX1508 _motor1(motor1A, motor1B, 0, 1);
MX1508 _motor2(motor2A, motor2B, 2, 3);
MX1508 _motor3(motor3A, motor3B, 4, 5);
MX1508 _motorArm(motorArmA, motorArmB, 6, 7);
Adafruit_MPU6050 mpu;

double Setpoint, Input, Output;
double Kp = 5, Ki = 7, Kd = 0.1; // PID tuning parameters
PID angPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


const float wheelRadius = 0.062; // Example radius in meters 62mm


// Moving average filter
#define SAMPLE_SIZE 10
float pitchSamples[SAMPLE_SIZE];
int sampleIndex = 0;
float sumPitch = 0;

unsigned long previousMillis = 0;
const long interval = 100; // Adjust the interval as needed

const long toggleInterval = 5000; // Adjust the interval as needed  
unsigned long previousToggleMillis = 0;

// Time variables
int delayTime = 2000;
int currentTime = 0;


OmniDrive omniDrive(motor1A, motor1B, motor2A, motor2B, motor3A, motor3B, wheelRadius);


MX1508 MotorL(0, 0, 0, 0);


void maintainArmAngle(MX1508 &motor, int speed, int targetAngle);
float readPitchAngle();
void addSample(float pitch);
float getMeanPitch();


void setup() {


    angPID.SetMode(AUTOMATIC);
    angPID.SetOutputLimits(-255, 255); // Set the PID output limits between -255 and 255
    omniDrive.begin(encoder1A, encoder1B, encoder2A, encoder2B, encoder3A, encoder3B);


    omniDrive.setPIDTunings(1.0, 0.1, 0.01); //PID Vals


      Serial.println("Sensor init success");
        Serial.println("Found a MPU6050 sensor");
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    
 Setpoint = 10; // Example target angle
}

void loop() {

    currentTime = millis();
    if (currentTime < delayTime) {         //Vx = 1.0 m/s, Vy = 0, omega = 0 rad/s
        omniDrive.moveRobot(1.0, 0, 0);
    } else if (currentTime < 2 * delayTime) { //Vx = 0, Vy = 1.0 m/s, omega = 0 rad/s
        omniDrive.moveRobot(0, 1.0, 0);
    } else if (currentTime < 3 * delayTime) { //Vx = -1.0 m/s, Vy = 0, omega = 0 rad/s
        omniDrive.moveRobot(-1.0, 0, 0);
    } else if (currentTime < 4 * delayTime) { //Vx = 0, Vy = -1.0 m/s, omega = 0 rad/s
        omniDrive.moveRobot(0, -1.0, 0);
    } else if (currentTime < 5 * delayTime) {   //rotation 0.1 rad/s
        omniDrive.moveRobot(0, 0, 0.1);
    } else if (currentTime < 6 * delayTime) {   //rotation -0.1 rad/s
        omniDrive.moveRobot(0, 0, -0.01);
    } else if (currentTime < 7 * delayTime){    //reset robot and stop moving
        currentTime = 0;
        omniDrive.stop();
    } else currentTime = 0; // Stop robot in error
    delay(100);    


// Angle Test code:
    // unsigned long currentMillis = millis();
    // if (currentMillis - previousMillis >= interval) {
    //     previousMillis = currentMillis;
    //     maintainArmAngle(_motorArm, 200, Setpoint); // Maintain motor arm at the target angle
    // }
    // if(currentMillis - previousMillis >= toggleInterval){
    //     previousToggleMillis = currentMillis;
    //     Setpoint = -Setpoint;
    // }
    // delay(10);
}


void maintainArmAngle(MX1508 &motor, int speed, int targetAngle) {
    float currentAngle = getMeanPitch();

    // Set the current angle as input to the PID controller
    Input = currentAngle;
    angPID.Compute();

    // Drive the motor with the PID output
    if (Output > 0) {
        motor.motorGo(abs(Output));
    } else {
        motor.motorRev(abs(Output));
    }

    // Print the current angle for debugging
    Serial.print("Current angle: ");
    Serial.println(currentAngle);
}


//Digital Mean Proccessing (Filter) for pitch angle 
void addSample(float pitch) {
    sumPitch -= pitchSamples[sampleIndex];
    pitchSamples[sampleIndex] = pitch;
    sumPitch += pitch;
    sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;
}

float getMeanPitch() {
    float pitch = readPitchAngle();
    addSample(pitch);
    return sumPitch / SAMPLE_SIZE;
}
float readPitchAngle() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Convert accelerometer values to G
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    // Calculate the pitch angle
    float pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    return pitch;
}