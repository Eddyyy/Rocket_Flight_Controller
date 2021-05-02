/*
    @Purpose: Simple barometer reading
    @Author: Edward Thomson
*/

#include <inttypes.h>
#include <Wire.h>


elapsedMillis baroReadTimer;

#define LED_PIN 13

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    while (!Serial) {
    ; // wait for serial port to connect.
    } 

    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    
    if (baroReadTimer >= 1000) {
        baroReadTimer = 0;
        Serial.println("Presure, Temperature");

        Wire.beginTransmission(0x76);
        Wire.write(0xF4);
        Wire.write(0x00 | 0b01 | 0b001<<2 | 0b001<<5);
        Wire.endTransmission();

        Wire.beginTransmission(0x76);
        Wire.write(0xF7);
        Wire.endTransmission();
        Wire.requestFrom(0x76, 6);
        
        // Preasure val
        uint16_t preasure = Wire.read();
        preasure <<= 8;
        preasure |= Wire.read();
        Wire.read();

        // Temperature
        uint16_t temp = Wire.read();
        temp <<= 8;
        temp |= Wire.read();
        Wire.read();

        Serial.print(preasure);
        Serial.print("\t");
        Serial.print(temp);
        Serial.println();

    }

    if (baroReadTimer <= 500) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
}
