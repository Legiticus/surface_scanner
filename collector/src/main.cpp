#include <Arduino.h>


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1000);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    if (data == "HELLO") {
      Serial.println("WORLD");
    }
  }
}
