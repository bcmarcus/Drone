/*
* David Dold
* JDDL Design, LLC
*
* HC-06 differs from an HC-05. HC-06 are slave only devices, whereas HC-05 can be master or slave.
* By default, no BT connection, an HC-06 device in AT mode @ 9600 baud between
*
* An HC-06 does not respond to "AT"
*
* The response from an HC-06 device for "AT+NAMEwhatever" is OKsetname
*
* LEDs flashing on the HC-06 indicate command mode, steady indicate a BT connection.
*
* NOTE: If you maintain a BT connection, via a terminal session or mobile app, then the AT commands
* will simply transmit to the connected device.
*
* The HC-06 spoc sheet calls for divided resistors and RX, I've never done it...
*
* TROUBLESHOOTING
* If you are unable to get this to work, confirm your rx/tx pins are correct. Confirm your wiring with a Serial BT Terminal.
* On Android there are many to choose from. Pair your HC-06 with your phone, then connect to the device via the BT terminal app,
* LEDs on HC-06 go solid. Run this sketch. Characters sent from the send textbox via the serial monitor or sent from the app,
* echo on the opposing device. If your characters do not appear, re-check your wiring (hint is tx/rx reversed?).
*
* If you are using seperate power supplies for your Arduino device -and- the HC-06, the grounds must be bonded.
*
* Setting baud rate is for the hardware connection between
* processor pid and the HC-06 device. BT connection is negotiated
* between radios.

*/

#include <SoftwareSerial.h>
#include <Arduino.h>

#define BLUETOOTH_RX 15
#define BLUETOOTH_TX 14
#define BAUDRATE 9600

SoftwareSerial bluetooth(BLUETOOTH_RX, BLUETOOTH_TX);

void setup()
{
  Serial.begin(BAUDRATE);
  Serial.println("ready");

  pinMode(BLUETOOTH_RX, INPUT);
  pinMode(BLUETOOTH_TX, OUTPUT);

  //setup bluetooth
  bluetooth.begin(BAUDRATE);
  delay(1000);
  // bluetooth.write("AT+NAMEDrone");
  // bluetooth.write("AT+VERSION");
}

void loop()
{
  //optionally, send characters via the SerialMonitor, this can be commented out and the bluetooth.write in setup() works.
  //outbound to HC-06
  if(Serial.available())
  {
    Serial.println("writing to bt");
    while (Serial.available())
    bluetooth.write(char(Serial.read()));
  }

  //inbound from HC-O6
  if(bluetooth.available())
    {
    Serial.println("BT Response");
    while (bluetooth.available())
    {
      char c = (char)bluetooth.read();
      delay(3);
      Serial.print(c);
      bluetooth.print(c);
    }
    bluetooth.println();
    Serial.println();
  }
}