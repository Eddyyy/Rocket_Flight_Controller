#include "TinyGPS++.h"
#include <SoftwareSerial.h>

// The TinyGPS++ object
TinyGPSPlus gps;

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600);
    while(!Serial){
    ;   // wait for serial port to connect.
    }
}

void loop() {
    while (Serial2.available()) {
        gps.encode(Serial2.read());
    }
    if (gps.location.isUpdated()) {
        Serial.print(F("Lat ="));
        Serial.print(gps.location.lat(), 6);
        Serial.print(F("Long ="));
        Serial.println(gps.location.lng(), 6);
    } else if (gps.speed.isUpdated()) {
        Serial.print(F("Speed (m/s) ="));
        Serial.print(gps.speed.mps());
    } else if (gps.course.isUpdated()) {
        Serial.print(F("Course (Deg) ="));
        Serial.println(gps.course.deg());
    } else if (gps.altitude.isUpdated()) {
        Serial.print(F("Altitude (m) ="));
        Serial.print(gps.altitude.meters());
    } else if (gps.satellites.isUpdated()) {
        Serial.print(F("Satellites ="));
        Serial.println(gps.satellites.value());
    } else if (gps.hdop.isUpdated()) {
        Serial.print(F("HDOP ="));
        Serial.println(gps.hdop.hdop());
    }
}
