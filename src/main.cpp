/******************************************************************************

Temperature and humidity gateway for:
- Common 433 MHz weather station sensors (Nexus-TH, WS2910, etc.)
- Local measurements using a BMP280 sensor
- Reports from a WS2910 weather station via HTTP POST requests

It forwards all received data to a dashboard via HTTP POST requests and supports 
logging over both Serial and Telnet.

More information at:
https://www.onetransistor.eu/2024/01/receive-lpd433-weather-unit-nexus.html

******************************************************************************/
#include <Arduino.h>
// OTA updates
#include <ArduinoOTA.h>
// Logging
#include "Logger.h"
#include "Channels/SerialChannel.h"
#include "Channels/TelnetChannel.h"
// WiFi and networking
#include "net.h"
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <uri/UriBraces.h>
// 433 MHz receiver and packet processing
#include "THReciever.h"
// JSON serialization for dashboard communication
#include <ArduinoJson.h>

#include "config.h"
#include "secrets.h"
#include "manage_eeprom.h"
#include "manage_local_th.h"
#include "manage_webserver.h"
#include "manage_thingspeak.h"


/******************************************************************************
 *                                                                            *
 * 433Mhz receiver variables, macros and functions                            *
 *                                                                            *
 *****************************************************************************/

// Setting up the receiver and devices
THReceiver receiver = THReceiver();

#define LOOP_RECEIVER() {\
  if (receiver.isAvailable()) {\
    THPacket packet = receiver.getLastReceived();\
    processPacket(packet);\
  }\
}

/******************************************************************************
 *                                                                            *
 * Sensor handling functions                                                  *
 *                                                                            *
 *****************************************************************************/

/**
 * @brief Finds the index of the THDevice that best fits the incoming packet.
 * 
 * @param packet The incoming THPacket to search for.
 * @return The index of the device if found, otherwise -1.
 */
int findDevice(THPacket packet) {
  int foundIndex = -1;
  for (int n = 0; n < SENSOR_COUNT; n++) {
    THSensor *d = sensors[n];
    int compare = d->compareTo(packet.deviceID, packet.channelNo);
    if (compare == 1) {
      // Perfect match: deviceID and channel number both match. We can stop searching immediately.
      return n;
    } else if (compare == 0) {
      // Partial match: deviceID matches but channel number differs. We keep looking for a better match, but remember this one in case we don't find a perfect match.
      foundIndex = n;
    }
  }
  return foundIndex;
}

/**
 * @brief Processes an incoming THPacket.
 */
void processPacket(THPacket packet) {
    char sentence[64];
    sprintf(sentence, "0x%02X|%1d|%s|%5.1f|%3d", 
      packet.deviceID, packet.channelNo, packet.batteryState? "N" : "L", packet.temperature, packet.humidity
    );
    Log.trace(sentence);
    int i = findDevice(packet);
    if (i >= 0) {
      THSensor *d = sensors[i];
      d->process(packet);
    } else {
      Log.warning(String("Device not found for: ") + String(sentence));
    }
}


/**
 * @brief The setup function runs once when the device is powered on or reset. 
 * 
 */
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  #if SERIAL_DEBUGGING_ENABLED > 0
    SETUP_SERIAL_LOGGING();
  #endif
  // Start the EEPROM and load the configuration
  Log.info("Loading configuration from EEPROM...");
  BEGIN_EEPROM();
  config = loadConfig(); // Load config with version control, initialize if version mismatch
  loadSensorConfigs(sensorConfigs); // Load sensor configs from EEPROM
  // Connect to WiFi and set up telnet logging if enabled
  connectWifi(SECRET_SSID, SECRET_PASS);
  #if TELNET_DEBUGGING_ENABLED > 0
    SETUP_TELNET_LOGGING();
    telnetChannel.setLogLevel(config.telnetLogLevel);
  #endif

  Log.info("Starting OTA service");
  ArduinoOTA.begin();
  BEGIN_WEB_SERVER();
  BEGIN_THINGSPEAK();
  
  Log.info("Initializing devicemanagement...");
  for (int i=0; i<SENSOR_COUNT; i++) {
    Log.trace("Initializing device " + String(i) + " " + String(sensorConfigs[i].code));
    sensors[i] = new THSensor(&sensorConfigs[i]);
  }
  // Initialize the local sensor and 433MHz receiver 
  Log.info("Initializing local BMP280 sensor..."); 
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  SETUP_BMP280();
  Log.info("Starting 433 MHz receiver on interrupt pin " + String(THRECEIVER_PIN));
  receiver.begin(THRECEIVER_PIN);
  Log.info("Setup completed successfully. System is ready.");
  delay(500); // Short delay to ensure all setup logs are sent before starting normal operation
}

/**
 * @brief The loop function runs continuously after setup() has completed.
 * 
 */
void loop() {
  ArduinoOTA.handle();
  // Check for telnet clients
  handleTelnetConnections();
  // Handle incoming HTTP requests
  server.handleClient();
  // Check for incoming packets from the 433 MHz receiver
  LOOP_RECEIVER();
  // Periodically take local measurements from the BMP280 sensor
  LOOP_BMP280();
  // Sensor timeout watchdog
  for (int n = 0; n < SENSOR_COUNT; n++) {
    THSensor *d = sensors[n];
    d->checkTimeout();
  }
  // Periodically update ThingSpeak with the latest measurements and status updates
  handleThingSpeak();
  // Give the cpu some rest
  delay(100);
}
