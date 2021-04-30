/*
    @Purpose: testing the spi control of the power supply
    @Author: Edward Thomson
*/

#include <inttypes.h>

elapsedMillis printTimer;
elapsedMillis blinkTimer;

#define LED_PIN 13

void setup() {
    Serial.begin(115200);
    
    while (!Serial) {
    ; // wait for serial port to connect.
    } 

    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    
    if (printTimer >= 1000) {
        printTimer = 0;
        Serial.println("Hello World!");

    }

    if (printTimer <= 500) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
}

