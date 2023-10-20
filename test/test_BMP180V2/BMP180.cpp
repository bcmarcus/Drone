#include <BMP180.h>
#include <Wire.h>

BMP180 bmp180(Wire1, 3);
int counter = 0;
boolean print = true;
boolean flag = false;
uint32_t timer = 0;

void setup (){ 
  Serial.begin(115200);
  bmp180.begin();
  timer = millis();
}

void loop (){
  switch (bmp180.getState()) {
    case BMP180::Bmp180State::IDLE:
      bmp180.stateIdle();
      break;
    case BMP180::Bmp180State::REQUESTED_TEMP:
      bmp180.stateTemperature();
      break;
    case BMP180::Bmp180State::REQUESTED_PRESSURE:
      bmp180.statePressure();
      if (bmp180.hasNewData()) {
        counter++;
        flag = true;
        if (print) {
          Serial.print("Temperature: "); Serial.print (bmp180.getTemperature()); Serial.print (" // ");
          Serial.print("Pressure: "); Serial.print (bmp180.getPressure()); Serial.print (" // ");
          Serial.print("Altitude: "); Serial.println(bmp180.getAltitude());
        }
        else {
          bmp180.getTemperature();
          bmp180.getPressure();
          bmp180.getAltitude();
        }
      }
      break;
  }

  if (counter % 100 == 0 && flag) {
    Serial.print(counter);
    Serial.print(" :: ");
    Serial.print(millis() / 1000.0);
    Serial.print(" :: ");
    Serial.println((millis() - timer) / 1000.0);
    timer = millis();
    flag = false;
  }
}