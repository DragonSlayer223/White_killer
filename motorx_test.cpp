// RUMBA X motor: one revolution per 'g' command.
// 200 full steps/revolution × 16 microsteps = 3200 pulses.
// Driver microstepping must be configured physically.

const byte STEP_PIN   = 17;
const byte DIR_PIN    = 16;
const byte ENABLE_PIN = 48;

// X, Y, Z, E0, E1, E2 — active LOW.
const byte ENABLE_PINS[] = {48, 55, 62, 24, 27, 39};

const int PULSES_PER_REVOLUTION = 3200;
const unsigned int STEP_DELAY_MS = 1;
const unsigned int PULSE_WIDTH_US = 10;

void setup() {
  // Disable all six motor drivers.
  for (byte i = 0; i < 6; i++) {
    digitalWrite(ENABLE_PINS[i], HIGH);
    pinMode(ENABLE_PINS[i], OUTPUT);
  }

  digitalWrite(STEP_PIN, LOW);
  pinMode(STEP_PIN, OUTPUT);

  digitalWrite(DIR_PIN, HIGH);  // Change to LOW to reverse.
  pinMode(DIR_PIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("Ready. Send g for one revolution.");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'g' || command == 'G') {
      Serial.println("Moving X: 3200 pulses...");

      digitalWrite(ENABLE_PIN, LOW);
      delay(100);

      for (int step = 0; step < PULSES_PER_REVOLUTION; step++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(PULSE_WIDTH_US);

        digitalWrite(STEP_PIN, LOW);
        delay(STEP_DELAY_MS);
      }

      delay(200);
      digitalWrite(ENABLE_PIN, HIGH);  // Release holding torque.

      // Discard commands received during movement.
      while (Serial.available() > 0) {
        Serial.read();
      }

      Serial.println("Done. Send g to repeat.");
    }
  }
}
