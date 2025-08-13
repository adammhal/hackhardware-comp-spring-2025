# Electromagnetic Sorting Rover
3rd Place at BU HackHardware & Rastic Collaboration (Spring 2025)
This repository contains the source code for the Electromagnetic Sorting Rover, a project developed by a team of four for the Boston University HackHardware and Rastic hackathon. The rover is designed to autonomously navigate, identify, and sort metal coins into designated colored bins using an innovative electromagnetic arm.

## Features
Omnidirectional Movement: Utilizes Mecanum wheels for 360-degree movement, allowing for precise and agile navigation.

Electromagnetic Arm: Equipped with a custom-built electromagnet to securely pick up and release metal coins.

Bluetooth Control: Fully controllable via a Bluetooth gamepad, with support for analog stick movement and button-activated arm/magnet control.

Dual-Speed Mode: Features a speed reduction toggle for high-precision maneuvers.

Modular Codebase: The software is written in C++ on the Arduino framework, with separate files for testing individual components and a main controller for integrated operation.

## Hardware
Microcontroller: ESP32

Motor Driver: Adafruit DC & Stepper Motor Shield

Chassis: Custom-built frame with Mecanum wheels

Actuators:

4x DC Motors for the wheels

2x Servos for arm and magnet control

Custom-wound electromagnet

Power: Portable battery pack

## Software
The project is programmed in C++ using the Arduino framework. Key libraries include:

Bluepad32.h: For seamless Bluetooth gamepad integration.

ShiftRegister74HC595.h: To manage motor direction control via the motor shield.

ESP32Servo.h: For precise control of the arm and magnet servos.

## Code Structure
MainControllerCode.ino: The primary file that integrates all systems, including Bluetooth input processing, Mecanum wheel kinematics, and servo control.

motorShieldTest.ino: A script for testing the basic movement functions of the rover (forward, backward, strafe, rotate).

EMTest.ino: A dedicated test script for calibrating and testing the servo-actuated electromagnetic arm.

IndividualMotorTest.ino: A utility script to test each of the four motors independently.

## Setup & Usage
Hardware Assembly: Connect the motors, servos, and motor shield to the ESP32 as defined by the pin constants at the top of MainControllerCode.ino.

IDE Setup: Open the .ino files in the Arduino IDE with the ESP32 board manager installed.

Library Installation: Install the Bluepad32, ShiftRegister74HC595, and ESP32Servo libraries through the Arduino Library Manager.

Flashing: Upload MainControllerCode.ino to the ESP32.

## Control:

Power on the rover. The system will automatically search for a Bluetooth controller.

Connect your gamepad.

Use the left analog stick for movement, the right analog stick for rotation, and the controller buttons (A, B, X, Y) to control the arm and electromagnet.

Team
This project was a collaborative effort by a team of four dedicated engineering students.
