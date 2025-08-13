#include <ShiftRegister74HC595.h>

// Motor Control Definitions
#define LED_BUILTIN 2

#define M1 17
#define M2 26
#define M3 13
#define M4 14

// Set these reversal flags as needed
#define M1_REV false
#define M2_REV false
#define M3_REV true
#define M4_REV true

#define SR_DATA 23
#define SR_CLK 25
#define SR_LTCH 16
#define SR_EN 4

// Motor directions
#define OFF 0b00
#define FWD 0b01
#define REV 0b10

// Create a shift register object
ShiftRegister74HC595<1> SR(SR_DATA, SR_CLK, SR_LTCH);
uint8_t motor_state = 0x00;

// Functions to control each motor
void setM1(uint8_t speed, uint8_t dir) {
  if (M1_REV) dir = ~dir;
  motor_state &= ~(0b00001100);
  motor_state |= (dir << 2);
  uint8_t tmp = motor_state;
  SR.setAll(&tmp);
  analogWrite(M1, speed);
}

void setM2(uint8_t speed, uint8_t dir) {
  if (M2_REV) dir = ~dir;
  motor_state &= ~(0b00010010);
  motor_state |= (dir & 0b10);
  motor_state |= (dir & 0b01) << 4;
  uint8_t tmp = motor_state;
  SR.setAll(&tmp);
  analogWrite(M2, speed);
}

void setM3(uint8_t speed, uint8_t dir) {
  if (M3_REV) dir = ~dir;
  motor_state &= ~(0b10100000);
  motor_state |= (dir & 0b10) << 4;
  motor_state |= (dir & 0b01) << 7;
  uint8_t tmp = motor_state;
  SR.setAll(&tmp);
  analogWrite(M3, speed);
}

void setM4(uint8_t speed, uint8_t dir) {
  if (M4_REV) dir = ~dir;
  motor_state &= ~(0b01000001);
  motor_state |= (dir & 0b10) << 5;
  motor_state |= (dir & 0b01);
  uint8_t tmp = motor_state;
  SR.setAll(&tmp);
  analogWrite(M4, speed);
}

// Helper function to stop all motors
void stopAllMotors() {
  setM1(0, OFF);
  setM2(0, OFF);
  setM3(0, OFF);
  setM4(0, OFF);
}

void setup() {
  Serial.begin(115200);
  // Initialize motor control pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(SR_DATA, OUTPUT);
  pinMode(SR_CLK, OUTPUT);
  pinMode(SR_LTCH, OUTPUT);
  pinMode(SR_EN, OUTPUT);
  
  digitalWrite(SR_EN, LOW);
  uint8_t tmp = 0;
  SR.setAll(&tmp);
  
  Serial.println("Motor test starting...");
}

void loop() {
  // Test Motor 1
  Serial.println("Testing Motor 1");
  setM1(255, FWD);
  delay(5000);
  setM1(0, OFF);
  delay(2000);

  // Test Motor 2
  Serial.println("Testing Motor 2");
  setM2(255, FWD);
  delay(5000);
  setM2(0, OFF);
  delay(2000);

  // Test Motor 3
  Serial.println("Testing Motor 3");
  setM3(255, FWD);
  delay(5000);
  setM3(0, OFF);
  delay(2000);

  // Test Motor 4
  Serial.println("Testing Motor 4");
  setM4(255, FWD);
  delay(5000);
  setM4(0, OFF);
  delay(2000);

  // Stop all motors before starting the cycle again
  stopAllMotors();
  Serial.println("Cycle complete. Restarting in 5 seconds...");
  delay(5000);
}
