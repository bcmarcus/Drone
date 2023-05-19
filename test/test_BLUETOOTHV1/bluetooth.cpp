
// TO USE :::: START BY PLUGGING EVERYTHING IN, AND REMOVING DEVICE FROM BLUETOOTH BY FORGETTING IT.
// TO USE :::: PLUG IN ALL OF THE CORDS, AND THEN PUSH THE PROGRAM
// TO USE :::: IF TEENSY IS OFF, AND PROGRAM CANNOT PUSH, UNPLUG AND PLUG IN TEENSY CORD AGAIN
// TO USE :::: RUN THIS COMMAND TO FIND THE TTY PORT: ls /dev/tty.*
// TO USE :::: CONNECT TO BLUETOOTH, AND THEN OPEN THE SERIAL WINDOW WITH 'platformio device monitor --port /dev/tty.HC-06'
// TO USE :::: DETAILS SHOULD BE SHOWN HERE


#include <SoftwareSerial.h>
#include <Arduino.h>

// Define the RX and TX pins for the Bluetooth module
const int rxPin = 7;
const int txPin = 8;


// Initialize the SoftwareSerial library for communication with the Bluetooth module
SoftwareSerial bluetooth(rxPin, txPin);

String s = "";
char prev = ' ';

void setup() {
  // Set the baud rate for the serial monitor and the Bluetooth module
  Serial.begin(9600);
  bluetooth.begin(9600);
}


void loop() {
  // Forward data from the Bluetooth module to the serial monitor
  if (bluetooth.available()) {
    while (bluetooth.available()) {
      prev = (char) bluetooth.read();
      s += prev;
    }

    // end of command
    if (prev == '\n'){
      Serial.print("Recieved: ");
      Serial.print(s);
      Serial.print("Sending: ");
      Serial.print(s);
      bluetooth.print(s);
      Serial.print("Sent!");
      Serial.print("\n\n");
      s = "";
      prev = ' ';
    }
  }
}
