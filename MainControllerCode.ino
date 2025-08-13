#include <Bluepad32.h>
#include <ShiftRegister74HC595.h>
#include <ESP32Servo.h>

#define LED_BUILTIN 2

#define M1 17
#define M2 26
#define M3 13
#define M4 14

#define M1_REV false
#define M2_REV false
#define M3_REV true
#define M4_REV true

#define S1 18
#define S2 19 

#define SR_DATA 23
#define SR_CLK 25
#define SR_LTCH 16
#define SR_EN 4

#define OFF 0b00
#define FWD 0b01
#define REV 0b10

ShiftRegister74HC595<1> SR(SR_DATA, SR_CLK, SR_LTCH);
uint8_t motor_state = 0x00;

Servo armControl;
Servo magnetControl;
const int SERVO_STOP = 1500;
const int SERVO_CW = 1700;    
const int SERVO_CCW = 1300;  
const int SERVO_SPEED = 80;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
const int deadzone = 50;
const int maxSpeed = 255;

bool speedReductionToggle = false;
int prevBrakeValue = 0;
unsigned long lastToggleTime = 0;
const unsigned long DEBOUNCE_TIME = 200;

void setM1(uint8_t speed, uint8_t dir) {
  if (M1_REV) dir = ~dir;
  motor_state &= ~(0b00001100);
  motor_state |= (dir << 2);
  const uint8_t tmp = (motor_state);
  SR.setAll(&tmp);
  analogWrite(M1, speed);
}

void setM2(uint8_t speed, uint8_t dir) {
  if (M2_REV) dir = ~dir;
  motor_state &= ~(0b00010010);
  motor_state |= (dir & 0b10);
  motor_state |= (dir & 0b01) << 4;
  const uint8_t tmp = (motor_state);
  SR.setAll(&tmp);
  analogWrite(M2, speed);
}

void setM3(uint8_t speed, uint8_t dir) {
  if (M3_REV) dir = ~dir;
  motor_state &= ~(0b10100000);
  motor_state |= (dir & 0b10) << 4;
  motor_state |= (dir & 0b01) << 7;
  const uint8_t tmp = (motor_state);
  SR.setAll(&tmp);
  analogWrite(M3, speed);
}

void setM4(uint8_t speed, uint8_t dir) {
  if (M4_REV) dir = ~dir;
  motor_state &= ~(0b01000001);
  motor_state |= (dir & 0b10) << 5;
  motor_state |= (dir & 0b01);
  const uint8_t tmp = (motor_state);
  SR.setAll(&tmp);
  analogWrite(M4, speed);
}

void armClockwise() {
  armControl.writeMicroseconds(SERVO_CW);
  Serial.println("Arm moving clockwise (100%)");
}

void armCounterClockwise() {
    armControl.writeMicroseconds(SERVO_CCW);
    Serial.println("Arm moving counter-clockwise (100%)");
}

void magnetClockwise() {
  magnetControl.writeMicroseconds(SERVO_CW);
  Serial.println("Servo moving clockwise");
}

void magnetCounterClockwise() {
  magnetControl.writeMicroseconds(SERVO_CCW);
  Serial.println("Servo moving counter-clockwise");
}

void stopArm() {
  armControl.writeMicroseconds(SERVO_STOP);
  Serial.println("Servo stopped");
}

void stopMagnet() {
  magnetControl.writeMicroseconds(SERVO_STOP);
  Serial.println("Servo stopped");
}

// Movement Functions
void MoveLeft(int speed) {
  setM1(speed, FWD);
  setM2(speed, REV);
  setM3(speed, FWD);
  setM4(speed, REV);
}

void MoveRight(int speed) {
  setM1(speed, REV);
  setM2(speed, FWD);
  setM3(speed, REV);
  setM4(speed, FWD);
}

void MoveForward(int speed) {
  setM1(speed, FWD);
  setM2(speed, FWD);
  setM3(speed, FWD);
  setM4(speed, FWD);
}

void MoveBackward(int speed) {
  setM1(speed, REV);
  setM2(speed, REV);
  setM3(speed, REV);
  setM4(speed, REV);
}

void RotateRight(int speed) {
  setM1(speed, FWD);
  setM2(speed, REV);
  setM3(speed, REV);
  setM4(speed, FWD);
}

void RotateLeft(int speed) {
  setM1(speed, REV);
  setM2(speed, FWD);
  setM3(speed, FWD);
  setM4(speed, REV);
}

void Stationary() {
  setM1(0, OFF);
  setM2(0, OFF);
  setM3(0, OFF);
  setM4(0, OFF);
}

// Bluetooth Callbacks
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("Controller %d connected\n", i);
      myControllers[i] = ctl;
      return;
    }
  }
  Serial.println("Controller connected, no available slot");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      Serial.printf("Controller %d disconnected\n", i);
      return;
    }
  }
}

// Controller Processing
void processController(ControllerPtr ctl) {
  if (ctl && ctl->isConnected()) {
    // Get axis values (-512 to 512)
    float axisLX = ctl->axisX();
    float axisLY = ctl->axisY();
    float axisRX = ctl->axisRX()*-1;

    // Handle left trigger speed toggle
    int currentBrake = ctl->brake();
    if (currentBrake > 500 && prevBrakeValue <= 500 && 
       (millis() - lastToggleTime) > DEBOUNCE_TIME) {
      speedReductionToggle = !speedReductionToggle;
      lastToggleTime = millis();
      Serial.print("Speed reduction ");
      Serial.println(speedReductionToggle ? "ENABLED" : "DISABLED");
    }
    prevBrakeValue = currentBrake;

    // Apply deadzone and normalize values (-1.0 to 1.0)
    float x = 0, y = 0, z = 0;
    
    if (abs(axisLX) > deadzone) 
      x = constrain(axisLX / 512.0f, -1.0f, 1.0f);
    if (abs(axisLY) > deadzone) 
      y = -constrain(axisLY / 512.0f, -1.0f, 1.0f); // Invert Y axis
    if (abs(axisRX) > deadzone)
      z = constrain(axisRX / 512.0f, -1.0f, 1.0f);

    // Apply speed multiplier
    float speedMultiplier = speedReductionToggle ? 0.40f : 1.0f;
    x *= speedMultiplier;
    y *= speedMultiplier;
    z *= speedMultiplier;

    // Mecanum wheel kinematic equations
    float frontLeft = y + x + z;
    float frontRight = y - x - z;
    float backLeft = y - x + z;
    float backRight = y + x - z;

    // Find maximum possible speed
    float maxVal = fmaxf(fabsf(frontLeft), fmaxf(fabsf(frontRight), 
                   fmaxf(fabsf(backLeft), fabsf(backRight))));
    if (maxVal > 1.0f) {
      frontLeft /= maxVal;
      frontRight /= maxVal;
      backLeft /= maxVal;
      backRight /= maxVal;
    }

    // Convert to motor speeds (0-255)
    int m1Speed = abs(frontLeft) * maxSpeed;
    int m2Speed = abs(frontRight) * maxSpeed;
    int m3Speed = abs(backLeft) * maxSpeed;
    int m4Speed = abs(backRight) * maxSpeed;

    // Set motor directions based on sign
    setM1(m1Speed, frontLeft > 0 ? FWD : REV);
    setM2(m2Speed, frontRight > 0 ? FWD : REV);
    setM3(m3Speed, backLeft > 0 ? FWD : REV);
    setM4(m4Speed, backRight > 0 ? FWD : REV);

    // Servo controls (keep existing)
    if (ctl->a()) {
      armClockwise();
    } else if (ctl->b()) {
      armCounterClockwise();
    } else {
      stopArm();
    }

    if(ctl->x()) {
      magnetClockwise();
    } else if(ctl->y()) {
      magnetCounterClockwise();
    } else {
      stopMagnet();
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Motor Control Initialization
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
  const uint8_t tmp = 0;
  SR.setAll(&tmp);

  // Bluetooth Initialization
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableNewBluetoothConnections(true);
  BP32.forgetBluetoothKeys();


  armControl.attach(S1, 500, 2500);
  magnetControl.attach(S2, 500, 2500);
  stopArm();
  stopMagnet();
  Serial.println("Servo initialized");
  
  Serial.println("System Ready - Waiting for controller connection...");
}

void loop() {
  BP32.update();
  
  for (auto controller : myControllers) {
    if (controller && controller->isConnected()) {
      processController(controller);
    }

  }
  
  delay(2);
  vTaskDelay(1);
}