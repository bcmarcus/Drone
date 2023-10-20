#include <Arduino.h>
#include <GY87.h>
#include <Wire.h>
#include <sstream>
#include <cstring>

const float start = 0.01;
const float end = 0.07;
const float step = 0.0005;
const int size = (end - start) / (step) + 2; // Number of steps
float beta_values[size];
float beta_results[size];
float min_diff = 9999; // Init with a large value
int closest_index; // Index of beta with result closest to zero

long count = 10000;
// long count = 100000;

TwoWire wire = Wire1;
GY87 gy87(wire);

const std::string formatFloat(float val, boolean normalize = false) {
  static char out[8];  // Increase the size of the out array to accommodate null terminator
  if (normalize) {
    val *= 1000;
  }
  std::memset(out, ' ', sizeof(out));  // Initialize the array with spaces
  dtostrf(val, 6, 5, out);  // Use 6 instead of 7 to leave space for the potential negative sign
  out[7] = '\0';  // Add null terminator at the end
  return out;
}

void customPrint (const char* data, boolean overridePrint = false) {
  if (Serial) {
    Serial.println (data);
  }
}


// Placeholder function for the test you want to perform.
float testBeta(float beta) {
  gy87.setBeta(beta);
  gy87.reset();

  // gy87.calibrate(true, false);
  // gy87.stabilize(true, count);
  gy87.stabilize(true, count);

  std::stringstream payloadStream;
  payloadStream 
    << "Beta: " << formatFloat(gy87.getBeta()) << " ||| "
    << "Acceleration (m/s^2) => X: " << formatFloat(gy87.getAccX()) << ", Y: " << formatFloat(gy87.getAccY()) << ", Z: " << formatFloat(gy87.getAccZ()) << " ||| "
    << "Gyro (deg/s^2) => X: " << formatFloat(gy87.getGyroX()) << ", Y: " << formatFloat(gy87.getGyroY()) << ", Z: " << formatFloat(gy87.getGyroZ()) << " ||| "
    << "Madgwick (deg) => X: " << formatFloat (gy87.getRoll()) << "  " << "Y: " << formatFloat (gy87.getPitch()) << "  " << "Z: " << formatFloat (gy87.getAbsoluteYaw());

  customPrint(payloadStream.str().c_str());
  return gy87.getAbsoluteYaw();
}

void calibrateBeta() {
  gy87.begin(1, 0, 3, true);
  
  int counter = 0;
  for(float i = start; i < end; i += step) {
    float beta = i; // Calculate beta
    beta_values[counter] = beta; // Store beta
    float result = testBeta(beta);
    beta_results[counter] = result; // Store result

    // Update minimum difference and corresponding beta
    if (abs(result) < min_diff) { 
      min_diff = abs(result);
      closest_index = counter;
    }
    counter++;
  }

  // Print the beta value with result closest to zero
  Serial.println("Beta with result closest to zero: ");
  Serial.print("Beta = ");
  Serial.println(beta_values[closest_index], 5); // print with 2 decimal places
  Serial.print("Result = ");
  Serial.println(beta_results[closest_index], 7); // print with 2 decimal places

  // Print the beta values and results directly below and above it
  if (closest_index > 0) { // Check if not the first element
    Serial.println("Beta and result directly below: ");
    Serial.print("Beta = ");
    Serial.println(beta_values[closest_index - 1], 5);
    Serial.print("Result = ");
    Serial.println(beta_results[closest_index - 1], 7);
  }

  if (closest_index < size - 1) { // Check if not the last element
    Serial.println("Beta and result directly above: ");
    Serial.print("Beta = ");
    Serial.println(beta_values[closest_index + 1], 5);
    Serial.print("Result = ");
    Serial.println(beta_results[closest_index + 1], 7);
  }
}

void calibrateMPU() {
  gy87.calibrate(true, false);
}

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 bits per second.
  
  gy87.begin(1, 0, 3, true);
  
  int counter = 0;
  for(float i = start; i < end; i += step) {
    float beta = i; // Calculate beta
    beta_values[counter] = beta; // Store beta
    float result = testBeta(beta);
    beta_results[counter] = result; // Store result

    // Update minimum difference and corresponding beta
    if (abs(result) < min_diff) { 
      min_diff = abs(result);
      closest_index = counter;
    }
    counter++;
  }

  // Print the beta value with result closest to zero
  Serial.println("Beta with result closest to zero: ");
  Serial.print("Beta = ");
  Serial.println(beta_values[closest_index], 5); // print with 2 decimal places
  Serial.print("Result = ");
  Serial.println(beta_results[closest_index], 7); // print with 2 decimal places

  // Print the beta values and results directly below and above it
  if (closest_index > 0) { // Check if not the first element
    Serial.println("Beta and result directly below: ");
    Serial.print("Beta = ");
    Serial.println(beta_values[closest_index - 1], 5);
    Serial.print("Result = ");
    Serial.println(beta_results[closest_index - 1], 7);
  }

  if (closest_index < size - 1) { // Check if not the last element
    Serial.println("Beta and result directly above: ");
    Serial.print("Beta = ");
    Serial.println(beta_values[closest_index + 1], 5);
    Serial.print("Result = ");
    Serial.println(beta_results[closest_index + 1], 7);
  }
}

void loop() {
  Serial.println("Complete");
  delay(100000);
}
