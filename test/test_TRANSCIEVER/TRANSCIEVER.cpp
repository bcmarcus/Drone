// RFM69HCW Example Sketch
// Send serial input characters from one RFM69 node to another
// Based on RFM69 library sample code by Felix Rusu
// http://LowPowerLab.com/contact
// Modified for RFM69HCW by Mike Grusin, 4/16

// This sketch will show you the basics of using an
// RFM69HCW radio module. SparkFun's part numbers are:
// 915MHz: https://www.sparkfun.com/products/12775
// 434MHz: https://www.sparkfun.com/products/12823

// See the hook-up guide for wiring instructions:
// https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide

// Uses the RFM69 library by Felix Rusu, LowPowerLab.com
// Original library: https://www.github.com/lowpowerlab/rfm69
// SparkFun repository: https://github.com/sparkfun/RFM69HCW_Breakout

// Include the RFM69 and SPI libraries:
// #include "Arduino.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>

#include <Arduino.h>
#include <RH_RF69.h>
#include <SPI.h>
#include <RHHardwareSPI.h>
#include <RHHardwareSPI1.h>

// Addresses for this node. CHANGE THESE FOR EACH NODE!

#define NETWORKID     0   // Must be the same for all nodes
#define MYNODEID      1   // My node ID
#define TONODEID      2   // Destination node ID
#define MAX_MESSAGE_LEN 10

// RFM69 frequency, uncomment the frequency of your module:

#define FREQUENCY   RF69_433MHZ
// #define FREQUENCY     RF69_915MHZ

// AES encryption (or not):

#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):

#define USEACK        false // Request ACKs or not

// Packet sent/received indicator LED (optional):

#define LED           9 // LED positive pin
#define GND           8 // LED ground pin
// Create a library object for our RFM69HCW module:

RH_RF69 driver(29, 31, hardware_spi1);

void setup()
{
  // Open a serial port so we can send keystrokes to the module:

  Serial.begin(9600);
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");  

  // Set up the indicator LED (optional):

  // pinMode(LED,OUTPUT);
  // digitalWrite(LED,LOW);
  // pinMode(GND,OUTPUT);
  // digitalWrite(GND,LOW);

  // Initialize the RFM69HCW:
  // radio.setCS(10);  //uncomment this if using Pro Micro
  if (!driver.init()) {
    Serial.println ("init failed");
  } else {
    Serial.println ("init success");
  }
  driver.setTxPower(15, true); // Always use this for RFM69HCW

  // Turn on encryption if desired:

  if (ENCRYPT)
    driver.setEncryptionKey((uint8_t *) ENCRYPTKEY);

}

void loop()
{
  // Set up a "buffer" for characters that we'll send:

  static char sendbuffer[MAX_MESSAGE_LEN];
  static int sendlength = 0;

  // SENDING

  // In this section, we'll gather serial characters and
  // send them to the other node if we (1) get a carriage return,
  // or (2) the buffer is full (61 characters).

  // If there is any serial input, add it to the buffer:
  // Serial.println ("loop");
  if (Serial.available() > 0)
  {
    Serial.println ("here1");
    char input = Serial.read();

    if (input != '\r') // not a carriage return
    {
      sendbuffer[sendlength] = input;
      sendlength++;
    }

    // If the input is a carriage return, or the buffer is full:

    if ((input == '\r') || (sendlength == MAX_MESSAGE_LEN - 1)) // CR or buffer full
    {
      // Send the packet!


      Serial.print("sending to node ");
      Serial.print(TONODEID, DEC);
      Serial.print(", message [");
      for (byte i = 0; i < sendlength; i++)
        Serial.print(sendbuffer[i]);
      Serial.println("]");

      // There are two ways to send packets. If you want
      // acknowledgements, use sendWithRetry():

      // if (USEACK)
      // {
      //   if (driver.send((uint8_t *) sendbuffer, sendlength))
      //     Serial.println("ACK received!");
      //   else
      //     Serial.println("no ACK received");
      // }

      // If you don't need acknowledgements, just use send():

      // else // don't use ACK
      // {
        driver.send((uint8_t *)sendbuffer, sendlength);
      // }

      sendlength = 0; // reset the packet
      // Blink(LED,10);
    }
  }

  // RECEIVING

  // In this section, we'll check with the RFM69HCW to see
  // if it has received any packets:
  uint8_t buf[MAX_MESSAGE_LEN];

  uint8_t len = MAX_MESSAGE_LEN;
  // uint8_t from;
  if (driver.recv(buf, &len)) // Got one!
  {
    // Print out the information:

    Serial.print("received from node ");
    Serial.print(", message [");

    for (byte i = 0; i < len; i++)
      Serial.print((char)buf[i]);

    // RSSI is the "Receive Signal Strength Indicator",
    // smaller numbers mean higher power.

    Serial.print("], RSSI ");
    Serial.println(driver.rssiRead());

    // Send an ACK if requested.
    // (You don't need this code if you're not using ACKs.)

    // if (radio.ACKRequested())
    // {
    //   radio.sendACK();
    //   Serial.println("ACK sent");
    // }
    // Blink(LED,10);
  }
}

void Blink(byte PIN, int DELAY_MS)
// Blink an LED for a given number of ms
{
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}