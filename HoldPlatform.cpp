#include <Arduino.h>

const byte ENABLE_PINS[6] = {48, 55, 62, 24, 27, 39};
const byte STEP_PINS[6]   = {17, 54, 57, 23, 26, 29};
const byte DIR_PINS[6]    = {16, 47, 56, 22, 25, 28};

void setup(){
    // Configure all motor pins
    for (byte i=0; i<6; i++){
        // Initially disable driver
        digitalWrite(ENABLE_PINS[i], LOW);
        pinMode(ENABLE_PINS[i], OUTPUT);

        digitalWrite(STEP_PINS[i], LOW);
        pinMode(STEP_PINS[i], OUTPUT);

        digitalWrite(DIR_PINS[i], LOW);
        pinMode(DIR_PINS[i], OUTPUT);
    }

    Serial.begin(115200);

    // Enable all six motors
    for (byte i=0; i<6; i++){
        digitalWrite(ENABLE_PINS[i], LOW);
    }

    Serial.println("All six motors enabled - holding position");
}

void loop(){
}