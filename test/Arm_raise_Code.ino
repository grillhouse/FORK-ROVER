#include <SoftPWMServo.h>
#include "Wire.h"
#include <MPU6050_light.h>
const int ARM_PWM =10;
#define sensor A0
MPU6050 mpu(Wire);
int distance;


void setup()
{
  Serial.begin(9600);
  Wire.begin();
  byte status = mpu.begin(); //starts mpu
  mpu.calcOffsets();
  delay(500);
  pinMode(ARM_PWM, OUTPUT); // PWM output
  digitalWrite(ARM_PWM, LOW);
  
}

void loop()
{
  float volts = analogRead(sensor) * 0.0048828125; // this value was grabbed from the sharp ir data sheet, I don't know where they got it from but it spits out the right distance
  int distance = 13 * pow(volts, -1);
}

int Arm(){

  while (distance < 15){
    float alpha = 0.15;
    float Anglex = 1;
    float Accum = 1;
    mpu.update();
    Anglex = mpu.getAngleX();
    Accum = (alpha * Anglex) + (1 - alpha) * Accum;
    SoftPWMServoServoWrite(ARM_PWM, 500); // when distance <15cm, the arm will start to actuate
    if (Accum >= 30){ // if filtered angle is greater than 30, motor stops.
        SoftPWMServoServoWrite(ARM_PWM, 1500); //stops the motor
       }
    }
  }
