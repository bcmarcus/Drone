#include <Arduino.h>

const int batteryPins[] = {20, 21, 22, 23};
const int numBatteries = 4;
const float maxVoltage = 2.08;
const float minVoltage = 1.7;
const float voltageRange = maxVoltage - minVoltage;
volatile bool interruptTriggered = false;

void checkBattery() {
  for (int i = 0; i < numBatteries; i++) {
    float voltage = (analogRead(batteryPins[i]) / 1023.0) * 3.3;
    int percentage = (voltage - minVoltage) / voltageRange * 100;
    if (percentage < 20) {
      interruptTriggered = true;
      Serial.print("Battery ");
      Serial.print(i + 1);
      Serial.println(" is below 20%.");
    }
  }
}

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < numBatteries; i++) {
    pinMode(batteryPins[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(2), checkBattery, FALLING);
}

void loop() {
  if (!interruptTriggered) {
    for (int i = 0; i < numBatteries; i++) {
      float voltage = (analogRead(batteryPins[i]) / 1023.0) * 3.3;
      int percentage = (voltage - minVoltage) / voltageRange * 100;
      Serial.print("Battery ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(percentage);
      Serial.println("%");
    }
    delay(1000);
  }
}
