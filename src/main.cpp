#include "HolonomicRobot.h"
#include <Arduino.h>
#include <ESP32Encoder.h>

// Define motor control pins
HolonomicRobot robot(5, 18, 32, 35, 25, 33);

// Define encoder pins
#define CLK1 16 // CLK pin for encoder 1
#define DT1 17  // DT pin for encoder 1
#define CLK2 27 // CLK pin for encoder 2
#define DT2 26  // DT pin for encoder 2
#define CLK3 12 // CLK pin for encoder 3
#define DT3 14  // DT pin for encoder 3

// Create instances of ESP32Encoder
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

void setup() {
    // Start the serial connection
    Serial.begin(115200);

    // Initialize robot
    robot.begin();

    // Attach encoders and reset count
    encoder1.attachHalfQuad(DT1, CLK1);
    encoder1.setCount(0);
    encoder2.attachHalfQuad(DT2, CLK2);
    encoder2.setCount(0);
    encoder3.attachHalfQuad(DT3, CLK3);
    encoder3.setCount(0);
}

void loop() {
    // Variables to store the current encoder positions
    long newPosition1, newPosition2, newPosition3;

    // Move the robot in a circular path
    for (int i = 0; i < 360; i += 10) {
        robot.moveRobot(350, i, 0);
        newPosition1 = encoder1.getCount() / 2; // Read encoder 1 position
        newPosition2 = encoder2.getCount() / 2; // Read encoder 2 position
        newPosition3 = encoder3.getCount() / 2; // Read encoder 3 position
        Serial.print("Encoder 1: ");
        Serial.print(newPosition1);
        Serial.print(", Encoder 2: ");
        Serial.print(newPosition2);
        Serial.print(", Encoder 3: ");
        Serial.println(newPosition3);
        delay(100);
    }

    // Stop the robot and pause for a moment
    robot.stop();
    delay(1000); // Wait before the next loop
}