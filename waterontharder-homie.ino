#include "waterontharder-homie.h"   // Check this file for setup and secrets !!!!!


#define measurementInterval 5000 // measurement interval in milliseconds
#define minimumDistance 20 // smaller values will be capped to the minimum
#define maximumDistance 100 // maximum, this means the tank is empty
#define maximumCap 150 // larger values will be ignored

// global variables
int currentDistance; // last measured distance
int currentPercentage; // percentage based on last measurement

// ----------------------------------------------------------------------------------
//   Logging
// ----------------------------------------------------------------------------------

#define SERIAL_BAUD_RATE 9600 // for serial monitor

// Function to set up logging
void setupLogging() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("Logging initialized");
}

// Function to write log messages to the serial console, eg.:
// logMessage("Written to config: setpoint %d, hw-setpoint: %d", setpoint, hwSetpoint);
void logMessage(const char *format, ...) {
  char logBuffer[128]; // Adjust the buffer size as needed
  va_list args;
  va_start(args, format);
  vsnprintf(logBuffer, sizeof(logBuffer), format, args); // Format the log message
  va_end(args);

  Serial.println(logBuffer);
}

// ----------------------------------------------------------------------------------
//   Metering the distance
// ----------------------------------------------------------------------------------

#define trigPin 4 // pin to Trig of JSN-SR04T/AJ-SR04M
#define echoPin 5 // pin to Echo of JSN-SR04T/AJ-SR04M
#define echoTriggerDuration 20 // trigger length in microseconds

unsigned long previousMillis = 0; // tracks last measurement time

// Variables for running average
const int numReadings = 5; // number of readings in our running average
int readings[numReadings]; // Array to store readings for the running average
int readIndex = 0;  // Index for the circular buffer, where to add the next reading
int readTotal = 0;  // Running total of all recorded values

void setupMetering() {
  currentDistance = 0;
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);  //

  // prepare array for running average
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }

  delayMicroseconds(2);
}

// returns true when the measured distance changed
bool stepMetering() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < measurementInterval) {
    return false; // not up yet for a new measurement
  }

  // store current time as last time we measured
  previousMillis = currentMillis;

  // trigger a measurment
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(echoTriggerDuration);
  digitalWrite(trigPin, LOW);
  // Read the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH); // the duration of sound wave travel
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  // validate maximum boundaries
  if (distance > maximumCap) {
    return false;
  }

  // Calculate running average
  readTotal -= readings[readIndex];
  readTotal += distance;
  readings[readIndex] = distance;
  readIndex = (readIndex + 1) % numReadings;
  distance = readTotal / numReadings;

  // validate minimum boundary
  if (distance < minimumDistance) {
    distance = minimumDistance;
  }

  // did it change?
  if (distance == currentDistance) {
    return false; // no change
  }
  // changed!
  currentDistance = distance;
  currentPercentage = 100 - (distance * 100) / maximumDistance;
  if (currentPercentage < 0) {
    currentPercentage = 0;
  } else if (currentPercentage > 100) {
    currentPercentage = 100;
  }
  return true;
}


void setup() {
  setupLogging();
  setupMetering();
  logMessage("Initialization complete");
}

void loop() {
  if (stepMetering()) {
    // value was updated, so we should report it
    logMessage("Distance: %d cm @ %d %%", currentDistance, currentPercentage);
  }
  delay(100);
}
