#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(9600);
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

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}