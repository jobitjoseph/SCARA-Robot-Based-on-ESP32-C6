/*
 * Project Name: Scara Robot
 * Project Brief: ESP32-C6 based SCARA RBOT.
 * Author: Jobit Joseph @ https://github.com/jobitjoseph
 * IDE: Arduino IDE 2.x.x
 * Arduino Core: ESP32 Arduino Core V 3.0.5
 *
 * Copyright © Jobit Joseph
 * Copyright © Semicon Media Pvt Ltd
 * Copyright © Circuitdigest.com
 * 
 * This code is licensed under the following conditions:
 *
 * 1. Non-Commercial Use:
 * This program is free software: you can redistribute it and/or modify it
 * for personal or educational purposes under the condition that credit is given 
 * to the original author. Attribution is required, and the original author 
 * must be credited in any derivative works or distributions.
 *
 * 2. Commercial Use:
 * For any commercial use of this software, you must obtain a separate license
 * from the original author. Contact the author for permissions or licensing
 * options before using this software for commercial purposes.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING 
 * FROM, OUT OF, OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 *
 * Author: Jobit Joseph
 * Date: 18 October 2024
 *
 * For commercial use or licensing requests, please contact [jobitjoseph1@gmail.com].
 */
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>

#define RX_PIN 17
#define TX_PIN 16

#define GRIPPER_PIN 7  // Pin for the servo controlling the gripper
#define WRIST_PIN 1    // Pin for the servo controlling wrist rotation

// Define the stepper motors and the pins they will use
AccelStepper stepper1(1, 21, 20); // (Type:driver, STEP, DIR)
AccelStepper stepper2(1, 19, 18);
AccelStepper stepper3(1, 22, 23);

Servo gripperServo;  // create servo object to control a servo
Servo wristServo;    // Servo object for the wrist rotation

// Variables for kinematics
double L1 = 91.6; // Length of arm 1
double L2 = 95.2; // Length of arm 2
double theta1, theta2, phi, z;

const float theta1AngleToSteps = 14.55;  // Conversion factor for theta1
const float theta2AngleToSteps = 11.43;  // Conversion factor for theta2
const float zAngleToSteps = 400;             // Conversion factor for phi
const int gripperCloseAngle = 140;
const int gripperOpenAngle = 100;

void setup() {
    Serial.begin(115200); // USB communication for debugging

    // Stepper motors max speed and acceleration
    stepper1.setMaxSpeed(4000);
    stepper1.setAcceleration(2000);
    stepper2.setMaxSpeed(4000);
    stepper2.setAcceleration(2000);
    stepper3.setMaxSpeed(4000);
    stepper3.setAcceleration(2000);

    gripperServo.attach(GRIPPER_PIN);
    gripperServo.write(gripperOpenAngle); // Open gripper by default

    wristServo.attach(WRIST_PIN); // Pin for wrist control
    wristServo.write(90); // Default wrist angle

    delay(1000);
    Serial.println("Initialization completed");
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        executeCommand(command);
    }
}

void executeCommand(String command) {
    // Home command
    if (command.equalsIgnoreCase("home")) {
        homing();
    }
    // Move command in format: move x y z gripper gripperz speed accel
    else if (command.startsWith("move")) {
        int x, y, z;
        if (sscanf(command.c_str(), "move %d %d %d", &x, &y, &z) == 3) {
            moveToPosition(x, y, z);
        } else {
            Serial.println("Invalid move command format. Use: move x y z gripper gripperz speed accel");
        }
    }
    else if (command.startsWith("gripperz")) {
        int gripperZ;
        if (sscanf(command.c_str(), "gripperz %d", &gripperZ) == 1) {
            setGripperZ(gripperZ);
        } else {
            Serial.println("Invalid acceleration command format. Use: set_accel <accel>");
        }
    }
    // Set speed command
    else if (command.startsWith("set_speed")) {
        int speed;
        if (sscanf(command.c_str(), "set_speed %d", &speed) == 1) {
            setSpeedAllMotors(speed);
        } else {
            Serial.println("Invalid speed command format. Use: set_speed <speed>");
        }
    }
    // Set acceleration command
    else if (command.startsWith("set_accel")) {
        int accel;
        if (sscanf(command.c_str(), "set_accel %d", &accel) == 1) {
            setAccelerationAllMotors(accel);
        } else {
            Serial.println("Invalid acceleration command format. Use: set_accel <accel>");
        }
    }
    // Stop command
    else if (command.equalsIgnoreCase("stop")) {
        stopAllMotors();
    }
    // Open gripper command
    else if (command.equalsIgnoreCase("open_gripper")) {
        gripperServo.write(gripperOpenAngle); // Open gripper
    }
    // Close gripper command
    else if (command.equalsIgnoreCase("close_gripper")) {
        gripperServo.write(gripperCloseAngle); // Close gripper
    }
    // Unknown command
    else {
        Serial.println("Unknown command");
    }
}

// Function to move to position
void moveToPosition(int x, int y, int z) {
  
    stepper1.moveTo(x * theta1AngleToSteps);
    stepper2.moveTo(y * theta2AngleToSteps);
    stepper3.moveTo(z * zAngleToSteps);
    
    while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
    }
}

void setGripperZ(int gripperZ){
wristServo.write(gripperZ); // Set wrist position
}

// Function to set speed for all motors
void setSpeedAllMotors(int speed) {
    stepper1.setSpeed(speed);
    stepper2.setSpeed(speed);
    stepper3.setSpeed(speed);
    Serial.print("Speed set for all motors: ");
    Serial.println(speed);
}

// Function to set acceleration for all motors
void setAccelerationAllMotors(int accel) {
    stepper1.setAcceleration(accel);
    stepper2.setAcceleration(accel);
    stepper3.setAcceleration(accel);
    Serial.print("Acceleration set for all motors: ");
    Serial.println(accel);
}

// Function to stop all motors
void stopAllMotors() {
    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
    Serial.println("Motors stopped");
}

// Function for homing the robot
void homing() {
    // Implement homing logic here (e.g., move to a known position)
    Serial.println("Homing all motors");
    // Example homing code: move to the origin
    stepper1.moveTo(0);
    stepper2.moveTo(0);
    stepper3.moveTo(0);
    while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
    }
    Serial.println("Homing completed");
}
