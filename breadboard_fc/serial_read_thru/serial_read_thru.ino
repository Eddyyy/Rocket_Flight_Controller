void setup()
{
  Serial.begin(2000000);
  Serial1.begin(1000000);
  Serial1.attachRts(2);
  while(!Serial){
      ;
  }

}

void loop()
{
  // Dispatch incoming characters
    while (Serial1.available()) {
        String message = Serial1.readStringUntil("\n");
        Serial1.print(message);
        Serial.print(message);
    }

    while (Serial.available()) {
        String message = Serial.readStringUntil("\n");
        Serial1.print(message);
        Serial.print(message);
    }

}