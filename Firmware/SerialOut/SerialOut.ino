#include <ArduinoJson.h>

int currChar = 0;
long prevTimeMicros= 0;
long m = 0;
long final = 0;

String constantlySend = "";

void setup() {
  Serial.begin(115200);
  while (!Serial) continue;
  Serial.setTimeout(100);
 
}

void blink(int millis) {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(millis/2);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(millis/2); 
}

void loop() {
 
  char frontDelimiter = '<';
  char endDelimiter = '>';

  char jsonData[255];

  if(Serial.available() > 0) {
    currChar = Serial.read();

    if(currChar == frontDelimiter) {
      prevTimeMicros = micros();
      
     byte length = Serial.readBytesUntil(endDelimiter, jsonData, sizeof(jsonData));
      m = micros();
      
      char sub[length+1];
      memcpy( sub, &jsonData[0], length);
      sub[length] = '\0';
    
      StaticJsonBuffer<255> jsonBuffer;
    
      JsonObject& root = jsonBuffer.parseObject(jsonData);
      if (!root.success()) {
        return;
      }

      double FL = root["motorValues"]["FL"];
      double FR = root["motorValues"]["FR"];
      double FV = root["motorValues"]["FV"];
      double BL = root["motorValues"]["BL"];
      double BR = root["motorValues"]["BR"];
      double BV = root["motorValues"]["BV"];

      final = micros();
      
      Serial.print("Time elapsed in microseconds: ");
      Serial.println(final  - prevTimeMicros);

      
      Serial.print("Time for read: ");
      Serial.println( m - prevTimeMicros);

      
      
      Serial.print("Time for json: ");
      Serial.println( final - m);

      for(int i = 0; i < FL*10; i++) {
        blink(1000);
      }
    }
  }
 }

