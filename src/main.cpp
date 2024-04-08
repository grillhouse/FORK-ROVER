#include <Arduino.h>
#include <AccelStepper.h>
#include "OmniRobot.h"
#include "OmniStepper.h"

// put function declarations here:
int myFunction(int, int);

//Exampl Pins
const int stepPinLF = 2;  
const int dirPinLF = 3;   
const int stepPinRF = 4;
const int dirPinRF = 5;    
const int stepPinB = 6;
const int dirPinB = 7;

const float wheelDiameter = 0.08;     // Wheel diameter in meters
const int stepsPerRevolution = 200;   // Steps per revolution of the stepper motors

AccelStepper stepperLF(AccelStepper::DRIVER, stepPinLF, dirPinLF);
AccelStepper stepperRF(AccelStepper::DRIVER, stepPinRF, dirPinRF);
AccelStepper stepperB(AccelStepper::DRIVER, stepPinB, dirPinB);

OmniRobot robot(wheelDiameter, stepsPerRevolution);
OmniStepper omniStepper(stepperLF, stepperRF, stepperB);

void setup()
{

  stepperLF.setMaxSpeed(1000);
  stepperLF.setAcceleration(500);
  stepperRF.setMaxSpeed(1000);
  stepperRF.setAcceleration(500);
  stepperB.setMaxSpeed(1000);
  stepperB.setAcceleration(500);

  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop()
{
      // Example Position Vector
    float distance = 0.5;   // Distance in meters
    float angle = 45.0;     // Angle in degrees




}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}

// ------   Example From Library showing multiple steppers... How it was used to create the class / objects ------//

    // // MultiStepper.pde
    // // -*- mode: C++ -*-
    // //
    // // Shows how to multiple simultaneous steppers
    // // Runs one stepper forwards and backwards, accelerating and decelerating
    // // at the limits. Runs other steppers at the same time
    // //
    // // Copyright (C) 2009 Mike McCauley
    // // $Id: MultiStepper.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

    // #include <AccelStepper.h>

    // // Define some steppers and the pins the will use
    // AccelStepper stepper1; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
    // AccelStepper stepper2(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
    // AccelStepper stepper3(AccelStepper::FULL2WIRE, 10, 11);

    // void setup()
    // {
    //     stepper1.setMaxSpeed(200.0);
    //     stepper1.setAcceleration(100.0);
    //     stepper1.moveTo(24);

    //     stepper2.setMaxSpeed(300.0);
    //     stepper2.setAcceleration(100.0);
    //     stepper2.moveTo(1000000);

    //     stepper3.setMaxSpeed(300.0);
    //     stepper3.setAcceleration(100.0);
    //     stepper3.moveTo(1000000);
    // }

    // void loop()
    // {
    //     // Change direction at the limits
    //     if (stepper1.distanceToGo() == 0)
    // 	stepper1.moveTo(-stepper1.currentPosition());
    //     stepper1.run();
    //     stepper2.run();
    //     stepper3.run();
    // }