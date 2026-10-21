
#include <Arduino.h>
#include <math.h>

// RUMBA ATmega2560 — Test all 6 stepper motors
// 200 full steps/rev × 16 microsteps = 3200 pulses

// ===== MOTOR PINS =====
// Motor order: X, Y, Z, E0, E1, E2

const byte STEP_PINS[6]   = {17, 54, 57, 23, 26, 29};
const byte DIR_PINS[6]    = {16, 47, 56, 22, 25, 28};
const byte ENABLE_PINS[6] = {48, 55, 62, 24, 27, 39};

const char* MOTOR_NAMES[6] = {
  "X", "Y", "Z", "E0", "E1", "E2"
};

// ===== MOTOR SETTINGS =====
const float ROTATION_DEGREES = 20.0;
const int PULSES_PER_REVOLUTION = 3200;

const unsigned int STEP_DELAY_MS = 1;
const unsigned int PULSE_WIDTH_US = 10;

// Direction for each motor
// Change LOW to HIGH to reverse individual motors
const byte POSITIVE_DIRECTIONS[6] = {
  LOW, HIGH, LOW, HIGH, LOW, HIGH
};

// Convert degrees to pulses
const long STEPS_TO_MOVE = lroundf(
  fabsf(ROTATION_DEGREES) *
  PULSES_PER_REVOLUTION / 360.0f
);

// ===== SETUP =====
void setup() {

  // Configure and disable all motor drivers
  for (byte i = 0; i < 6; i++) {

    digitalWrite(ENABLE_PINS[i], HIGH);
    pinMode(ENABLE_PINS[i], OUTPUT);

    digitalWrite(STEP_PINS[i], LOW);
    pinMode(STEP_PINS[i], OUTPUT);

    digitalWrite(DIR_PINS[i], LOW);
    pinMode(DIR_PINS[i], OUTPUT);
  }

  Serial.begin(115200);

  Serial.println("RUMBA Motor Test Ready");
  Serial.println("1=X, 2=Y, 3=Z, 4=E0, 5=E1, 6=E2");
  Serial.println("Send motor number to rotate.");
}

// ===== ROTATE SELECTED MOTOR =====
void rotateMotor(byte motor) {

  Serial.print("Rotating motor ");
  Serial.print(MOTOR_NAMES[motor]);
  Serial.print(": ");
  Serial.print(ROTATION_DEGREES);
  Serial.println(" degrees");

  // Determine direction
  byte direction = ROTATION_DEGREES >= 0
    ? POSITIVE_DIRECTIONS[motor]
    : (POSITIVE_DIRECTIONS[motor] == LOW ? HIGH : LOW);

  digitalWrite(DIR_PINS[motor], direction);

  // Enable selected motor
  digitalWrite(ENABLE_PINS[motor], LOW);
  delay(100);

  // Rotate motor
  for (long step = 0; step < STEPS_TO_MOVE; step++) {

    digitalWrite(STEP_PINS[motor], HIGH);
    delayMicroseconds(PULSE_WIDTH_US);

    digitalWrite(STEP_PINS[motor], LOW);
    delay(STEP_DELAY_MS);
  }

  delay(200);

  // Disable selected motor
  digitalWrite(ENABLE_PINS[motor], HIGH);

  Serial.print("Motor ");
  Serial.print(MOTOR_NAMES[motor]);
  Serial.println(" done.");
}

// ===== MAIN LOOP =====
void loop() {

  if (Serial.available() > 0) {

    char command = Serial.read();

    // Select motor using '1' to '6'
    if (command >= '1' && command <= '6') {

      byte motor = command - '1';

      rotateMotor(motor);
    }

    // Keep old 'g' command for X motor
    else if (command == 'g' || command == 'G') {
      rotateMotor(0);
    }

    // Clear commands received during movement
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}
