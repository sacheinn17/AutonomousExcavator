#include <AFMotor.h> // Include the Adafruit Motor Shield library

// Initialize motors
AF_DCMotor leftMotor1(1);  // Left wheel motor 1
AF_DCMotor leftMotor2(2);  // Left wheel motor 2
AF_DCMotor rightMotor1(3); // Right wheel motor 1
AF_DCMotor rightMotor2(4); // Right wheel motor 2

const int encoder1PinA = 5; // Encoder 1 Channel A
const int encoder1PinB = 6; // Encoder 1 Channel B
const int encoder2PinA = 7; // Encoder 2 Channel A
const int encoder2PinB = 8; // Encoder 2 Channel B

volatile long encoder1Count = 0; // Counter for Encoder 1
volatile long encoder2Count = 0; // Counter for Encoder 2

void encoder1ISR() {
    Serial.println("change");
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Count++;
  } else {
    encoder1Count--;
  }
}

// Interrupt service routines for Encoder 2
void encoder2ISR() {
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
    encoder2Count++;
  } else {
    encoder2Count--;
  }
}


// Constants
const float maxVelocity = 100.0; // Maximum velocity (rad/s) expected
const int maxPWM = 255;          // Maximum PWM value for motor control

//sudo chmod a+rw /dev/ttyACM0

void setup() {
  Serial.begin(9600); // Start serial communication
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, RISING);
}

void loop() {
  Serial.println("["+String(encoder1Count)+","+String(encoder2Count)+"]");
  if (Serial.available() > 0) {
    // Read incoming data until newline
    String receivedData = Serial.readStringUntil('\n');

    // Check format and parse values
    if (receivedData.startsWith("[") && receivedData.endsWith("]")) {
      receivedData = receivedData.substring(1, receivedData.length() - 1); // Remove brackets
      int commaIndex = receivedData.indexOf(',');
      if (commaIndex != -1) {
        float leftVelocity = receivedData.substring(0, commaIndex).toFloat();
        float rightVelocity = receivedData.substring(commaIndex + 1).toFloat();

        // Calculate PWM values from velocities
        int leftPWM = map(abs(leftVelocity), 0, maxVelocity, 0, maxPWM);
        leftPWM = constrain(leftPWM, 0, maxPWM);

        int rightPWM = map(abs(rightVelocity), 0, maxVelocity, 0, maxPWM);
        rightPWM = constrain(rightPWM, 0, maxPWM);

        // Set left motors speed and direction
        setMotor(leftMotor1, leftPWM, leftVelocity);
        setMotor(leftMotor2, leftPWM, leftVelocity);

        // Set right motors speed and direction
        setMotor(rightMotor1, rightPWM, rightVelocity);
        setMotor(rightMotor2, rightPWM, rightVelocity);
      }
    }
  }
}

// Function to set motor speed and direction
void setMotor(AF_DCMotor &motor, int pwmValue, float velocity) {
  if (velocity > 0) {
    motor.run(FORWARD);
  } else if (velocity < 0) {
    motor.run(BACKWARD);
  } else {
    motor.run(RELEASE);
  }
  motor.setSpeed(pwmValue);
}
