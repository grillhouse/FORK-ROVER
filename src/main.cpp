#include "HolonomicRobot.h"
#include <Arduino.h>
#include <ESP32Encoder.h>

//motor pins
HolonomicRobot robot(5, 18, 32, 35, 25, 33);

//encoder pins
#define CLK1 16 
#define DT1 17 
#define CLK2 27 
#define DT2 26 
#define CLK3 12 
#define DT3 14  

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

void setup() {

    Serial.begin(115200);


    robot.begin();

    encoder1.attachHalfQuad(DT1, CLK1);
    encoder1.setCount(0);
    encoder2.attachHalfQuad(DT2, CLK2);
    encoder2.setCount(0);
    encoder3.attachHalfQuad(DT3, CLK3);
    encoder3.setCount(0);
}

void loop() {

    long newPosition1, newPosition2, newPosition3;


    for (int i = 0; i < 360; i += 10) {
        robot.moveRobot(350, i, 0);
        newPosition1 = encoder1.getCount() / 2; 
        newPosition2 = encoder2.getCount() / 2; 
        newPosition3 = encoder3.getCount() / 2; 
        Serial.print("Encoder 1: ");
        Serial.print(newPosition1);
        Serial.print(", Encoder 2: ");
        Serial.print(newPosition2);
        Serial.print(", Encoder 3: ");
        Serial.println(newPosition3);
        delay(100);
    }
    robot.stop();
    delay(1000); 
}