#include "TinyGPS++.h"
#include <SoftwareSerial.h>

// The TinyGPS++ object
TinyGPSPlus gps;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600);
  while(!Serial){
      ;
  }
}

void loop()
{
  // Dispatch incoming characters
  while (Serial2.available())
    Serial.print(Serial2.readStringUntil("\n"));
}

//speed
//alt
//location
//course
//hdop
//satalite
