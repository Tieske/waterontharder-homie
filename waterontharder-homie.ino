#include "waterontharder-homie.h"   // Check this file for setup and secrets !!!!!

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h> // library "PubSubClient" by Nick O'Leary version 2.8, https://pubsubclient.knolleary.net/
#include "FS.h"

// Sanity checks; cap values due to sensor capabilities to get them in
// a sensible range
const int minimumMeasurementCap = 20;  // smaller values will be capped to the minimum
const int maximumMeasurementCap = 150; // larger values will be ignored as invalid

// global variables
int currentDistance; // last measured distance
int currentPercentage; // percentage based on last measurement
unsigned long maxTankDepth = defaultMaxTankDepth; // maximum, this measured distance means the tank is empty
unsigned long measurementInterval = defaultMeasurementInterval;

WiFiClient espClient; // The WiFi client instance
PubSubClient client(espClient); // The MQTT client instance
String deviceId = TIESKE_DEVICE_ID;

// ----------------------------------------------------------------------------------
//   Logging
// ----------------------------------------------------------------------------------


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
//   WiFi connection
// ----------------------------------------------------------------------------------


void wifiSetup()
{
  logMessage("Connecting to WiFi %s", TIESKE_WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(TIESKE_WIFI_SSID, TIESKE_WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  logMessage("WiFi connected. IP address: %s", WiFi.localIP().toString());
  delay(1500);
}


// ----------------------------------------------------------------------------------
//   Read and write configuration data
// ----------------------------------------------------------------------------------


const char* filePath = "/config.txt"; // File path to store the values

void storeConfig() {
  File file = SPIFFS.open(filePath, "w");
  if (!file) {
    logMessage("Failed to open config file for writing");
    return;
  }

  // Write the values to the file
  file.write((uint8_t*)&measurementInterval, sizeof(uint32_t));
  file.write((uint8_t*)&maxTankDepth, sizeof(uint32_t));
  file.close();
  logMessage("Written to config: measurementInterval %d, maxTankDepth: %d", measurementInterval, maxTankDepth);
  return;
}

void retrieveConfig() {
  File file = SPIFFS.open(filePath, "r");
  if (!file) {
    logMessage("Failed to open config file for reading");
    return;
  }

  // Read the values from the file
  file.readBytes((char*)&measurementInterval, sizeof(uint32_t));
  file.readBytes((char*)&maxTankDepth, sizeof(uint32_t));
  file.close();
  logMessage("Read from config: measurementInterval %d, hw-setpoint: %d", measurementInterval, maxTankDepth);
  return;
}

void storageSetup() {
  if (SPIFFS.begin()) {
    logMessage("SPIFFS mounted successfully");
  } else {
    logMessage("SPIFFS mount failed");
  }
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
  if (distance > maximumMeasurementCap) {
    return false;
  }

  // Calculate running average
  readTotal -= readings[readIndex];
  readTotal += distance;
  readings[readIndex] = distance;
  readIndex = (readIndex + 1) % numReadings;
  distance = readTotal / numReadings;

  // validate minimum boundary
  if (distance < minimumMeasurementCap) {
    distance = minimumMeasurementCap;
  }

  // did it change?
  if (distance == currentDistance) {
    return false; // no change
  }
  // changed!
  currentDistance = distance;
  currentPercentage = 100 - (distance * 100) / maxTankDepth;
  if (currentPercentage < 0) {
    currentPercentage = 0;
  } else if (currentPercentage > 100) {
    currentPercentage = 100;
  }
  return true;
}


// ----------------------------------------------------------------------------------
//   Homie device setup
// ----------------------------------------------------------------------------------


String intervalPropertyTopic = "homie/" + deviceId + "/config/interval";
String intervalPropertySetTopic = "homie/" + deviceId + "/config/interval/set";
String storageDepthPropertyTopic = "homie/" + deviceId + "/config/depth";
String storageDepthPropertySetTopic = "homie/" + deviceId + "/config/depth/set";
String storageStatus = "homie/" + deviceId + "/status/remainder";
String storageDepth = "homie/" + deviceId + "/status/depth";

// Check if a received payload is a float
bool isFloat(const String& value) {
  char* endPtr;
  float result = strtof(value.c_str(), &endPtr);
  return *endPtr == '\0';
}

// Callback handling subscribed topic values received
void callback(char* topic, byte* payload, unsigned int length) {
  //logMessage("received MQTT data");

  if (length <= 100) { // we do not expect payloads that big, so ignore if > 100
  
    payload[length] = '\0'; // Null-terminate the payload
    String value = String((char*)payload);
    //logMessage("received MQTT data: %s", value);
  
    if (String(topic) == intervalPropertySetTopic ||
        String(topic) == storageDepthPropertySetTopic) {
      if (isFloat(value) && value.toFloat() > 0) {

        // valid topic as well as valid value
        if (String(topic) == intervalPropertySetTopic) {
          // new meaurement interval (in seconds)
          measurementInterval = 1000 * (long)floor(value.toFloat()); //convert to millisecs
          client.publish(intervalPropertyTopic.c_str(), String(floor(measurementInterval/1000)).c_str(), true);
          storeConfig();  // store updated settings in SPIFFS
          currentDistance = currentDistance + 100; // force a "change" and hence an update on next measurement
          
        } else if (String(topic) == storageDepthPropertySetTopic) {
          // new maximum depth (in cm)
          maxTankDepth = (int)floor(value.toFloat());
          client.publish(storageDepthPropertyTopic.c_str(), String(maxTankDepth).c_str(), true);
          storeConfig();  // store updated settings in SPIFFS
          currentDistance = currentDistance + 100; // force a "change" and hence an update on next measurement
        } 

      } else {
        logMessage("'%s' received bad value: '%s'", topic, value);
      }
    } else {
      logMessage("received value on unknown topic: '%s'", String(topic));
    }
  }
}

void writeTopic(String topic, String value, bool retain) {
  client.publish(topic.c_str(), value.c_str(), retain);
}

// Write the Homie description to the MQTT topics and subscribe to topics
void publishHomieDeviceDescription() {
  writeTopic("homie/" + deviceId + "/$state",                "init", true);
  writeTopic("homie/" + deviceId + "/$extensions",           "none", true);
  writeTopic("homie/" + deviceId + "/$homie",                "4.0.0", true);
  writeTopic("homie/" + deviceId + "/$implementation",       "https://github.com/Tieske/waterontharder-homie", true);
  writeTopic("homie/" + deviceId + "/$name",                 "Waterontharder zout meter", true);
  writeTopic("homie/" + deviceId + "/$nodes",                "config,status", true);

  writeTopic("homie/" + deviceId + "/config/$name",          "Configuratie voor zout voorraad meting", true);
  writeTopic("homie/" + deviceId + "/config/$properties",    "interval,depth", true);
  writeTopic("homie/" + deviceId + "/config/$type",          "parameters", true);
  
  writeTopic("homie/" + deviceId + "/config/interval",            String(floor(measurementInterval/1000)), true);
  writeTopic("homie/" + deviceId + "/config/interval/$datatype",  "integer", true);
  writeTopic("homie/" + deviceId + "/config/interval/$name",      "Measurement interval", true);
  writeTopic("homie/" + deviceId + "/config/interval/$retained",  "true", true);
  writeTopic("homie/" + deviceId + "/config/interval/$settable",  "true", true);
  writeTopic("homie/" + deviceId + "/config/interval/$unit",      "s", true);

  writeTopic("homie/" + deviceId + "/config/depth",               String(maxTankDepth), true);
  writeTopic("homie/" + deviceId + "/config/depth/$datatype",     "integer", true);
  writeTopic("homie/" + deviceId + "/config/depth/$name",         "tank depth", true);
  writeTopic("homie/" + deviceId + "/config/depth/$retained",     "true", true);
  writeTopic("homie/" + deviceId + "/config/depth/$settable",     "true", true);
  writeTopic("homie/" + deviceId + "/config/depth/$unit",         "cm", true);

  writeTopic("homie/" + deviceId + "/status/$name",          "status", true);
  writeTopic("homie/" + deviceId + "/status/$properties",    "remainder,depth", true);
  writeTopic("homie/" + deviceId + "/status/$type",          "sensor", true);

  writeTopic("homie/" + deviceId + "/status/remainder",            "100", true);
  writeTopic("homie/" + deviceId + "/status/remainder/$datatype",  "integer", true);
  writeTopic("homie/" + deviceId + "/status/remainder/$name",      "remainder in tank", true);
  writeTopic("homie/" + deviceId + "/status/remainder/$retained",  "true", true);
  writeTopic("homie/" + deviceId + "/status/remainder/$settable",  "false", true);
  writeTopic("homie/" + deviceId + "/status/remainder/$unit",      "%", true);

  writeTopic("homie/" + deviceId + "/status/depth",                String(maxTankDepth), true);
  writeTopic("homie/" + deviceId + "/status/depth/$datatype",      "integer", true);
  writeTopic("homie/" + deviceId + "/status/depth/$name",          "measured distance", true);
  writeTopic("homie/" + deviceId + "/status/depth/$retained",      "true", true);
  writeTopic("homie/" + deviceId + "/status/depth/$settable",      "false", true);
  writeTopic("homie/" + deviceId + "/status/depth/$unit",          "cm", true);

  // subscribe to setter topic
  client.subscribe(intervalPropertySetTopic.c_str(), 1);  // QoS level 1; at least once
  client.subscribe(storageDepthPropertySetTopic.c_str(), 1);  // QoS level 1; at least once

  // Mark device as ready
  writeTopic("homie/" + deviceId + "/$state", "ready", true);
}


// ----------------------------------------------------------------------------------
//   MQTT setup
// ----------------------------------------------------------------------------------


void setupMQTT() {
  client.setServer(TIESKE_MQTT_SERVER, TIESKE_MQTT_PORT);
  client.setCallback(callback);  
}

void reconnectMQTT() {
  while (!client.connected()) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost, reconnecting...");
      wifiSetup();  // will not return until connected
    }

    if (!client.connect(
          TIESKE_DEVICE_ID, 
          TIESKE_MQTT_USER, 
          TIESKE_MQTT_PASSWORD, 
          ("homie/" + deviceId + "/$state").c_str(),  // LWT: topic
          1,                                          // LWT: QoS
          true,                                       // LWT: retain
          "lost",                                     // LWT: value
          false))                                     // no clean session
      {  
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }

  Serial.println("Connected to MQTT broker");
  publishHomieDeviceDescription();
}


// ----------------------------------------------------------------------------------
//   Main application setup and loop
// ----------------------------------------------------------------------------------


void setup() {
  setupLogging();
  // SPIFFS.format();  // uncomment to format the config data, and run once. Make sure to comment out again afterwards!!!!
  storageSetup();
  retrieveConfig();
  setupMetering();
  wifiSetup();   // set up and connect
  setupMQTT();   // only set up, connect is automatic in the main loop
  logMessage("Initialization complete");
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT(); 
  }
  client.loop();

  if (stepMetering()) {
    // value was updated, so we should report it
    logMessage("Distance: %d cm @ %d %%", currentDistance, currentPercentage);
    writeTopic(storageStatus, String(currentPercentage), true);
    writeTopic(storageDepth, String(currentDistance), true);
  }
  delay(50);
}
