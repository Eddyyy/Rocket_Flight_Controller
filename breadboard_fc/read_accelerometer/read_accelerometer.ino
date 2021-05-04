//Uses https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library
/*
    @Purpose: Simple Accelerometer Reading
    @Author: Kevin Hu
*/

#include <Wire.h>

#include "MPU9250.h"
#include "quaternionFilters.h"

elapsedMillis accelReadTimer;

#define LED_PIN 13

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    while (!Serial) {
    ; // wait for serial port to connect.
    } 

    pinMode(LED_PIN, OUTPUT);
}
