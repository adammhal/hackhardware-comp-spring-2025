// Load SR library
// https://github.com/Simsso/ShiftRegister74HC595
#include <ShiftRegister74HC595.h>
#include <ESP32Servo.h>

Servo myServo;

// Pin Definitions
#define LED_BUILTIN 2

#define M1 17  //M1 PWM pin, Driver Board pin 11
#define M2 26  //M2 PWM pin, Driver Board pin 3
#define M3 13  //M3 PWM pin, Driver Board pin 6
#define M4 14  //M4 PWM pin, Driver Board pin 5

#define M1_REV false  // set to true to reverse M1 direction
#define M2_REV false   // set to true to reverse M2 direction
#define M3_REV true  // set to true to reverse M3 direction
#define M4_REV true  // set to true to reverse M4 direction

#define S1 18  //S1 PWM pin, Driver Board pin 9
#define S2 19  //S2 PWM pin, Driver Board pin 10

#define SR_DATA 23  // Serial Register data pin, Driver Board pin 8
#define SR_CLK 25   // Serial Register clock pin, Driver Board pin 4
#define SR_LTCH 16  // Serial Register Latch Pin, Driver Board pin 12
#define SR_EN 4     // Serial Register Enable, Driver Board pin 7
// ^ the SR_EN pin is moved from pin 12 to pin 4 to avoid ESP32 boot issues

#define OFF 0b00
#define FWD 0b01
#define REV 0b10

#define MOVESPEED 130  // default speed for movements

ShiftRegister74HC595<1> SR(SR_DATA, SR_CLK, SR_LTCH);  // Initialize Serial Register on Driver Board (Data, Clock, Latch)

uint8_t motor_state = 0x00;  // bitstring that defines the behavior of each motor driver via the Serial Register

void setM1(uint8_t speed, uint8_t dir)
// sets the speed and direction of a motor, given by the pin number of that motor.
{
  if (M1_REV) dir = ~dir;  // Swaps FWD and REV if motor turns the wrong way
  // Clear bits 2 and 3 in SR_state
  motor_state &= ~(0b00001100);  // Mask to clear bits 2 and 3
  // Set bits 2 and 3 with the value of dir
  motor_state |= (dir << 2);  // Shift dir to align with bits 2 and 3, then OR
  // create temporary const for the SR
  const uint8_t tmp = (motor_state);
  // Update the SR
  SR.setAll(&tmp);
  // set the speed
  analogWrite(M1, speed);
}

void setM2(uint8_t speed, uint8_t dir)
// sets the speed and direction of a motor, given by the pin number of that motor.
{
  if (M2_REV) dir = ~dir;  // Swaps FWD and REV if motor turns the wrong way
  // Clear bits 1 and 4 in SR_state
  motor_state &= ~(0b00010010);
  // Set bit 1
  motor_state |= (dir & 0b10);
  // Set bit 4
  motor_state |= (dir & 0b01) << 4;  // Shift left 4
  // create temporary const for the SR
  const uint8_t tmp = (motor_state);
  // Update the SR
  SR.setAll(&tmp);
  // set the speed
  analogWrite(M2, speed);
}

void setM3(uint8_t speed, uint8_t dir)
// sets the speed and direction of a motor, given by the pin number of that motor.
{
  if (M3_REV) dir = ~dir;  // Swaps FWD and REV if motor turns the wrong way
  // Clear bits 7 and 5 in SR_state
  motor_state &= ~(0b10100000);
  // Set bit 5
  motor_state |= (dir & 0b10) << 4;
  // Set bit 7
  motor_state |= (dir & 0b01) << 7;
  // create temporary const for the SR
  const uint8_t tmp = (motor_state);
  // Update the SR
  SR.setAll(&tmp);
  // set the speed
  analogWrite(M3, speed);
}
void setM4(uint8_t speed, uint8_t dir)
// sets the speed and direction of a motor, given by the pin number of that motor.
{
  if (M4_REV) dir = ~dir;  // Swaps FWD and REV if motor turns the wrong way
  // Clear bits 0 and 6 in SR_state
  motor_state &= ~(0b01000001);
  // Set bit 6
  motor_state |= (dir & 0b10) << 5;
  // Set bit 0
  motor_state |= (dir & 0b01);
  // create temporary const for the SR
  const uint8_t tmp = (motor_state);
  // Update the SR
  SR.setAll(&tmp);
  // set the speed
  analogWrite(M4, speed);
}

// Variables to store the current output state
bool isMoveLeftOn = 0;
bool isMoveRightOn = 0;
bool isMoveForwardOn = 0;
bool isMoveBackwardOn = 0;
bool isRotateLeftOn = 0;
bool isRotateRightOn = 0;


// Motion Functions
void moveLeft() {
  setM1(MOVESPEED, FWD);
  setM2(MOVESPEED, REV);
  setM3(MOVESPEED, FWD);
  setM4(MOVESPEED, REV);
  return;
}
void moveRight() {
  setM1(MOVESPEED, REV);
  setM2(MOVESPEED, FWD);
  setM3(MOVESPEED, REV);
  setM4(MOVESPEED, FWD);
  return;
}
void moveForward() {
  setM1(MOVESPEED, FWD);
  setM2(MOVESPEED, FWD);
  setM3(MOVESPEED, FWD);
  setM4(MOVESPEED, FWD);
  return;
}
void moveBackward() {
  setM1(MOVESPEED, REV);
  setM2(MOVESPEED, REV);
  setM3(MOVESPEED, REV);
  setM4(MOVESPEED, REV);
  return;
}
void rotateLeft() {
  setM1(MOVESPEED, FWD);
  setM2(MOVESPEED, REV);
  setM3(MOVESPEED, REV);
  setM4(MOVESPEED, FWD);
  return;
}
void rotateRight() {
  setM1(MOVESPEED, REV);
  setM2(MOVESPEED, FWD);
  setM3(MOVESPEED, FWD);
  setM4(MOVESPEED, REV);
  return;
}
void stationary() {
  setM1(0, OFF);
  setM2(0, OFF);
  setM3(0, OFF);
  setM4(0, OFF);
  return;
}

void moveServoForward() {
  myServo.writeMicroseconds(1700);
}

void moveServoBackward() {
  myServo.writeMicroseconds(1300);
}

void stopServo() {
  myServo.writeMicroseconds(1525);
  delay(100);
}

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(115200);
  // Sends a Hello World Debug message
  Serial.println("Starting RASTICxHackH Spring 2025 Controller...");

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(SR_DATA, OUTPUT);
  pinMode(SR_CLK, OUTPUT);
  pinMode(SR_LTCH, OUTPUT);
  pinMode(SR_EN, OUTPUT);

  const uint8_t tmp = 0;
  SR.setAll(&tmp);  // Set all output pin of shift register to 0.

  digitalWrite(SR_EN, LOW);  // Activate the Serial register (Active Low)

  myServo.attach(S1, 500, 2500);


}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming value as a string
    String input = Serial.readStringUntil('\n');  // Read until a newline
    
    // Convert the input string to an integer
    int microseconds = input.toInt();
    
    // Check if the input is within a valid range (500 to 2500 microseconds)
    if (microseconds >= 500 && microseconds <= 2500) {
      // Set the servo position using the received microseconds value
      myServo.writeMicroseconds(microseconds);
      Serial.print("Servo moved to ");
      Serial.print(microseconds);
      Serial.println(" microseconds.");
    } else {
      // If the input is out of range, print an error message
      Serial.println("Invalid input! Please enter a value between 500 and 2500.");
    }
  }
}

