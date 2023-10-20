
// TO USE :::: START BY PLUGGING EVERYTHING IN, AND REMOVING DEVICE FROM BLUETOOTH BY FORGETTING IT.
// TO USE :::: PLUG IN ALL OF THE CORDS, AND THEN PUSH THE PROGRAM
// TO USE :::: IF TEENSY IS OFF, AND PROGRAM CANNOT PUSH, UNPLUG AND PLUG IN TEENSY CORD AGAIN
// TO USE :::: RUN THIS COMMAND TO FIND THE TTY PORT: ls /dev/tty.*
// TO USE :::: CONNECT TO BLUETOOTH, AND THEN OPEN THE SERIAL WINDOW WITH 'platformio device monitor --port /dev/tty.HC-06'
// TO USE :::: DETAILS SHOULD BE SHOWN HERE



// CRUCIAL :: WHEN SENDING AT COMMANDS, SEND THEM TWICE FOR SOME REASON



// BaudRates: Start with "AT+BAUD". Ex: "AT+BAUD4"
// 1---1200
// 2---2400
// 3---4800
// 4---9600
// 5---19200
// 6---38400
// 7---57600
// 8---115200
// 9---230400
// A---460800
// B---921600
// C---1382400 

#define CURRENTBAUD 9600
#define SETBAUD 9600
#define SET "AT+BAUD4"

#define DELAY 1000

#include <SoftwareSerial.h>
#include <Arduino.h>

const int rxPin = 15;
const int txPin = 14;

// Initialize the SoftwareSerial library for communication with the Bluetooth module
SoftwareSerial bluetooth(rxPin, txPin);

void sendBluetooth(const char *command) {
  Serial.print ("Sending Command: ");
  Serial.println(command);

  bluetooth.write(command);
  bluetooth.flush();
  delay (DELAY);

  if(bluetooth.available())
    {
    Serial.print("BT Response: ");
    while (bluetooth.available())
    {
      delay(3);
      Serial.print((char)bluetooth.read());
    }
    Serial.println("\n");
  }
}

void setup() {
  Serial.begin(9600);
  delay(DELAY);

  const char* commands[] = {
    "AT+BAUD1",
    "AT+BAUD2",
    "AT+BAUD3",
    "AT+BAUD4",
    "AT+BAUD5",
    "AT+BAUD6",
    "AT+BAUD7",
    "AT+BAUD8",
    "AT+BAUD9",
    "AT+BAUDA",
    "AT+BAUDB",
  };

  bluetooth.begin(SETBAUD);

  delay (DELAY);
  while (bluetooth.available()) {
    char c = bluetooth.read();  // Read a character from the serial port
    Serial.print(c);          // Print the character to the serial monitor
  }

  for (int i = 0; i < 3; i++) { 

    Serial.println(i);

    sendBluetooth("AT");
    // delay (500);
  }
  sendBluetooth("AT+VERSION");
  delay (500);

  sendBluetooth("AT+NAMEDrone");
  delay (500);

  sendBluetooth("AT+VERSION");
  delay (500);

  while (1);


  int baud = 921600;
  // for (int i = 10; i >= 6; i--) {
  //   // Start the hardware serial port for the Bluetooth module at 9600 (default baud rate for HC-06)
  //   bluetooth.begin(baud);

  //   // Wait for serial port to connect
  //   delay(DELAY);

  //   sendBluetooth(commands[i]);
  //   Serial.println(("Baud rate has been set to " + String(baud)));
  //   sendBluetooth("AT");

  //   baud = baud >> 1;
  // }

  // baud = 38400;
  baud = 9600;

  for (int i = 3; i >= 0; i--) {   
    // Start the hardware serial port for the Bluetooth module at 9600 (default baud rate for HC-06)
    bluetooth.begin(baud);

    delay(DELAY);

    sendBluetooth(commands[i]);

    delay(500);

    sendBluetooth("AT");
    delay (500);

    baud = baud >> 1;
  }



  

  //Setting the final value

  delay(DELAY);

  // change this to whatever you want
  bluetooth.begin (CURRENTBAUD);

  // bluetooth.write("AT+VERSION");
  delay(DELAY);
  while (bluetooth.available()) {
    char c = bluetooth.read();  // Read a character from the serial port
    Serial.print(c);          // Print the character to the serial monitor
  }
  Serial.println(SET);
  bluetooth.write("AT+NAMEDrone");
  delay(DELAY);
  while (bluetooth.available()) {
    char c = bluetooth.read();  // Read a character from the serial port
    Serial.print(c);          // Print the character to the serial monitor
  }

  delay(DELAY);
  bluetooth.write("AT+VERSION");
  delay(DELAY);

  while (bluetooth.available()) {
    char c = bluetooth.read();  // Read a character from the serial port
    Serial.print(c);          // Print the character to the serial monitor
  }

  // bluetooth.end();
  // delay (DELAY);
  // pinMode(15, INPUT);
  // pinMode(14, OUTPUT);

  bluetooth.begin(SETBAUD);

  delay (DELAY);
  while (bluetooth.available()) {
    char c = bluetooth.read();  // Read a character from the serial port
    Serial.print(c);          // Print the character to the serial monitor
  }
  // Serial.println("hello world");
}

void loop() {
  Serial.println("Loop");
  // bluetooth.write("hello world");
  // bluetooth.flush();
  // Serial.println("hello world");
  // while (bluetooth.available()) {
    // char c = bluetooth.read();  // Read a character from the serial port
    // Serial.print(c);          // Print the character to the serial monitor
  // }
  // bluetooth.flush();

  sendBluetooth("AT");

  // bluetooth.write("AT");
  // delay (DELAY);
  // while (bluetooth.available()) {
  //   char c = bluetooth.read();  // Read a character from the serial port
  //   Serial.print(c);          // Print the character to the serial monitor
  // }
  delay (500);
}