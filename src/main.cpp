/******************************************************************************

433.92 MHz Weather station receiver for Nexus-TH compatible units
More information at:
https://www.onetransistor.eu/2024/01/receive-lpd433-weather-unit-nexus.html

This sample code requires ESP8266 or AVR Arduino with 433.92 MHz receiver, 
an 128x64 pixel I2C OLED display and BMP280

******************************************************************************/
#include <Arduino.h>
#include <ArduinoOTA.h>

#include "THDisplay.h"
#include "Logger.h"
#include "channels/SerialChannel.h"
#include "channels/TelnetChannel.h"

#include "THSensor.h"

#define TELNET_DEBUGGING_ENABLED

// #define SERIAL_DEBUGGING_ENABLED

// Max 8 devices for now, can be extended in the future
#define SENSOR_COUNT 8

#define CONFIG_VERSION 1

struct Config {
  int version = 0; // Version control
  uint8_t logLevel = 2; // 0 = NONE, 5 = TRACE, ... 50 = CRITICAL
} config;

THSensorConfig sensorConfigs[SENSOR_COUNT];

THSensor **sensors = new THSensor*[SENSOR_COUNT];

/******************************************************************************
 *                                                                            *
 * EEPROM config handling                                                     *
 *                                                                            *
 *****************************************************************************/
#include <EEPROM.h>

#define CONFIG_EEPROM_SIZE sizeof(Config) + sizeof(sensorConfigs)
#define CONFIG_EEPROM_ADDRESS 0
#define CONFIG_SENSORS_ADDRESS CONFIG_EEPROM_ADDRESS + sizeof(Config)

void setSensorConfig(uint8_t index, uint8_t deviceID, const char* name, bool hasHumidity, uint8_t channel, int8_t correction, uint8_t maxValidDelta, uint8_t maxValidInterval) {
  sensorConfigs[index].deviceID = deviceID;
  strlcpy(sensorConfigs[index].name, name, sizeof(sensorConfigs[index].name));
  if (hasHumidity) {
    sensorConfigs[index].settings |= 0b00000001; // Set bit 0
  } else {
    sensorConfigs[index].settings &= ~0b00000001; // Clear bit 0
  }
  // Set the channel bits (bits 1-2) in settings
  sensorConfigs[index].settings &= ~0b00000110; // Clear bits 1-2
  sensorConfigs[index].settings |= ((channel & 0b00000011) << 1) & 0b00000110; // Set bits 1-2 based on channel value
  sensorConfigs[index].correction = correction;
  sensorConfigs[index].maxValidDelta = maxValidDelta;
  sensorConfigs[index].maxValidInterval = maxValidInterval;
}

void saveConfig() {
  Log.trace("Saving configuration...");
  EEPROM.begin(CONFIG_EEPROM_SIZE);
  EEPROM.put(CONFIG_EEPROM_ADDRESS, config);
  EEPROM.put(CONFIG_SENSORS_ADDRESS, sensorConfigs);
  EEPROM.commit();
}

void initConfig() {
  Log.trace("Initializing configuration...");
  setSensorConfig(0, 0xA0, "BT-S", false, 1,   0, 7, 15);
  setSensorConfig(1, 0xE5, "Garg", true,  2, -13, 7, 15);
  setSensorConfig(2, 0x22, "Kkn ", true,  3,   0, 7, 15);
  setSensorConfig(3, 0xD7, "Slkr", false, 1,   0, 7, 15);
  setSensorConfig(4, 0x53, "Kldr", true,  2,   0, 7, 15);
  setSensorConfig(5, 0x00, "Kntr", true,  0,   0, 7, 15);
  setSensorConfig(6, 0x16, "BT-G", true,  1,   0, 7, 15);
  setSensorConfig(7, 0xFF, "----", false, 0,   0, 7, 15);
  config.version = CONFIG_VERSION;
  config.logLevel = Logger::INFO;
  saveConfig();
}

void loadConfig(int version) {
  EEPROM.begin(CONFIG_EEPROM_SIZE);
  EEPROM.get(CONFIG_EEPROM_ADDRESS, config);
  if (config.version != version) {
    initConfig();
  } else {
    EEPROM.get(CONFIG_SENSORS_ADDRESS, sensorConfigs);
    Log.trace("Loaded config: version=" + String(config.version) + ", logLevel=" + String(config.logLevel));
  }
}

THSensorConfig loadSensorConfig(int index) {
  if (index < 0 || index >= SENSOR_COUNT) {
    Log.error("Invalid sensor index: " + String(index));
    return THSensorConfig(); // Return default config for invalid index
  }
  THSensorConfig config;
  EEPROM.begin(CONFIG_EEPROM_SIZE);
  EEPROM.get(CONFIG_SENSORS_ADDRESS + index * sizeof(THSensorConfig), config);
  return config;
} 


/******************************************************************************
 *                                                                            *
 * Wifi connection functions                                                  *
 *                                                                            *
 *****************************************************************************/
#include <WiFiServer.h>
#include <WiFiClient.h>
#include "net.h"

#include "secrets.h"

const char *ssid = SECRET_SSID; // your network SSID (name)
const char *pass = SECRET_PASS; // your network password

int status = WL_IDLE_STATUS;     // the WiFi radio's status

WiFiClient client;

/**
 * @brief (Re)connects to the WiFi network using the global SSID and 
 * password.
 */
void connectWifi() {
  if (status == WL_CONNECTED) {
    return;
  }
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_SHIELD) {
    Log.critical("No WiFi module! System HALT");
    // don't continue
    while (1) { // Don't proceed, loop forever. Everything else depends upon it
      delay(100);
    }
  }
  // Connect or reconnect to WiFi
  bool initializing = status == WL_IDLE_STATUS || WiFi.SSID() != ssid;
  if (initializing) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Init Wifi: %s", ssid);
    Log.info(buffer);
    WiFi.mode(WIFI_STA);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Log.trace(initializing ? "Initializing WiFi..." : "Connecting to WiFi...");
    status = WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
    delay(1000); // Wait for the wifi module to initialize and attempt connection
    while(WiFi.status() != WL_CONNECTED) {
      Log.warning("Conn: " + String(wl_status_to_string(WiFi.status())));
      delay(5000);     
    }
    Display.updateIPAdress(WiFi.localIP().toString());
  }
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "IP: %s", WiFi.localIP().toString().c_str());
  Log.info("WiFi connection successful");
}

// Setting up Telnet debugging (optional, can impact performance)
#ifdef TELNET_DEBUGGING_ENABLED
  #include "channels/TelnetChannel.h"
  WiFiServer telnetServer(23);
  WiFiClient telnetClient;
  TelnetChannel telnetChannel(&telnetClient, Logger::DEBUG);
#endif

// Setting up Serial debugging (optional, can impact performance)
#ifdef SERIAL_DEBUGGING_ENABLED
  #define SERIAL_DEBUGGING_LEVEL Logger::TRACE
  #pragma message("Serial debugging is ENABLED. This may impact performance. Set SERIAL_DEBUGGING_ENABLED to 0 to disable.")
  #include "channels/SerialChannel.h"
#endif

#include "THReciever.h"
// Setting up the receiver and devices
#define THRECEIVER_PIN 3
THReceiver receiver = THReceiver();

/**
 * @brief Finds the index of the THDevice with the specified device ID.
 * 
 * @param deviceID The device ID to search for.
 * @return The index of the device if found, otherwise -1.
 */
int findDevice(uint8_t deviceID) {
  for (int n = 0; n < SENSOR_COUNT; n++) {
    THSensor *d = sensors[n];
    if (d->hasID(deviceID))
      return n;
  }
  return -1;
}

/**
 * @brief Processes an incoming THPacket.
 */
void processPacket(THPacket packet) {
    char sentence[32];
    sprintf(sentence, "0x%02X|%1d|%s|%4.1f|%3d", 
      packet.deviceID, packet.channelNo, packet.batteryState? "N" : "L", packet.temperature, packet.humidity
    );
    Log.trace(sentence);
    int i = findDevice(packet.deviceID);
    if (i >= 0) {
      THSensor *d = sensors[i];
      d->process(packet);
      Display.updateDeviceInfo(d->displayID, d->getLastStatus());
    } else {
      sprintf(sentence, "UNKN: 0x%02X %4.1f", packet.deviceID, packet.temperature);
      Log.warning(sentence);
    }
}

/******************************************************************************
 *                                                                            *
 * ThingSpeak functions                                                       *
 *                                                                            *
 *****************************************************************************/
#include "ThingSpeakLogChannel.h"
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

#define THINGSPEAK_UPDATE_INTERVAL 5 * 60 * 1000 // 5 minutes
unsigned long nextThingSpeakUpdate = 0;

ThingSpeakLogChannel thingSpeakLogChannel(Logger::DEBUG);

/**
 * @brief Updates the ThingSpeak channel with the latest measurements from the 
 * devices and any pending status messages from the ThingSpeakLogChannel.
 */
void updateThingSpeak() {
  Log.trace("Updating ThingSpeak channel...");
  Display.updateThingSpeakStatus(0);
  bool hasUpdates = false;
  for (int n = 0; n < SENSOR_COUNT; n++) {
    THSensor *d = sensors[n];
    if (d->hasUpdates()) {
      THPacket m = d->getLastRecieved();
      ThingSpeak.setField(n + 1, m.temperature);
      hasUpdates = true;
    }
    if (thingSpeakLogChannel.hasStatusToSend()) {
      ThingSpeak.setStatus(thingSpeakLogChannel.getStatusForSending());
      hasUpdates = true;
    }
  }
  if (hasUpdates) {
    // reconnect if needed
    connectWifi();
    //write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    Display.updateThingSpeakStatus(x);
    if(x == 200) {
      Log.info("ThingSpeak update successful.");
    }
    else {
      Log.error("THINGSPEAK" + String("ThingSpeak error " + String(x)));
    }
  } else {
    Log.trace("No updates to send to ThingSpeak.");
    Display.updateThingSpeakStatus(100);
  }
}

/******************************************************************************
 *                                                                            *
 * Webserver request handler functions                                        *
 *                                                                            *
 *****************************************************************************/
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <uri/UriBraces.h>

ESP8266WebServer server(80);

/**
 * Handles GET requests to the root (/) endpoint.
 * 
 * It returns a JSON response containing an array of sensors with their actual
 * data.
 */
void handleRoot() {
  Log.debug("Received GET / request");
  JsonDocument json;
  JsonArray sensorsJson = json.to<JsonArray>();
  for (int i = 0; i < SENSOR_COUNT; i++) {
    JsonObject sensor = sensorsJson.add<JsonObject>();
    sensor["id"] = i;
    sensor["displayID"] = sensorConfigs[i].displayID;
    sensor["deviceID"] = sensorConfigs[i].deviceID;
    sensor["name"] = sensors[i]->getName();
    sensor["temperature"] = sensors[i]->getLastTemp();
    if (sensors[i]->hasHumidity()) {
      sensor["humidity"] = sensors[i]->getLastHumidity();
    }
    sensor["batteryState"] = sensors[i]->getLastBatteryState();
    sensor["age"] = sensors[i]->getLastAge();
  }
  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);
}

/**
 * Handles GET requests to the /config endpoint. 
 * 
 * It returns a JSON response containing the global config.
 */
void handleGetConfig() {
  Log.debug("Received GET /config request");
  JsonDocument json;
  json["log-level"] = config.logLevel;
  json["version"] = config.version;
  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);
}

/**
 * Handles PATCH request to the /config endpoint.
 * 
 * It updates the global config
 */
void handlePatchConfig() {
  Log.debug("Received PATCH /config request");
  if (server.hasArg("plain") == false) {
    server.send(400, "application/json", "{\"error\":\"Geen body\"}");
    return;
  }
  String body = server.arg("plain");
  JsonDocument json;
  DeserializationError error = deserializeJson(json, body);

  if (error) {
    server.send(400, "application/json", "{\"error\":\"Ongeldige JSON\"}");
    return;
  }

  if (json["log-level"].is<uint8_t>()) {
    config.logLevel = json["log-level"].as<uint8_t>();
    #ifdef TELNET_DEBUGGING_ENABLED
    telnetChannel.setLogLevel(config.logLevel);
    #endif
  }

  saveConfig(); // Sla bijgewerkte configuratie op

  JsonDocument response;
  response["log-level"] = config.logLevel;
  response["version"] = config.version;
  String output;
  serializeJson(response, output);
  server.send(200, "application/json", output);
} 

/**
 * Handles reset config requests
 */
void handleResetConfig() {
  Log.debug("Received POST /config/reset request");
  config.version = 0; // Reset version to trigger reinitialization on next load
  saveConfig();
  String response = "{\"status\":\"ok\",\"message\":\"Configuratie gereset!\"}";
  server.send(200, "application/json", response);
}

/**
 * Handles GET requests to the /config/sensors endpoint.
 */
void handleGetConfigSensors() {
  Log.debug("Received GET /config/sensors request");
  JsonDocument json;
  JsonArray sensors = json["sensors"].to<JsonArray>();
  for (int i = 0; i < SENSOR_COUNT; i++) {
    JsonObject sensor = sensors.add<JsonObject>();
    sensor["id"] = i;
    sensor["deviceID"] = sensorConfigs[i].deviceID;
    sensor["displayID"] = sensorConfigs[i].displayID;
    sensor["name"] = sensorConfigs[i].name;
    sensor["hasHumidity"] = (sensorConfigs[i].settings & 0b00000001) != 0;
    sensor["channel"] = (sensorConfigs[i].settings & 0b00000110) >> 1;
    sensor["correction"] = sensorConfigs[i].correction;
    sensor["maxValidDelta"] = sensorConfigs[i].maxValidDelta;
    sensor["maxValidInterval"] = sensorConfigs[i].maxValidInterval;
  }
  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);
}

/**
 * Handles PATCH requests to the /config/sensors endpoint.
 */
void handlePatchConfigSensors() {
  int id = server.pathArg(0).toInt();
  Log.debug("Received PATCH /config/sensors request for sensor ID: " + String(id));

  if (id < 0 || id >= SENSOR_COUNT) {
    server.send(404, "application/json", "{\"error\":\"Ongeldige sensor ID\"}");
    return;
  }

  if (server.hasArg("plain") == false) {
    server.send(400, "application/json", "{\"error\":\"Geen body\"}");
    return;
  }

  String body = server.arg("plain");
  JsonDocument json;
  DeserializationError error = deserializeJson(json, body);

  if (error) {
    server.send(422, "application/json", "{\"error\":\"Ongeldige JSON\"}");
    return;
  }

  if (json["deviceID"].is<uint8_t>()) {
    sensorConfigs[id].deviceID = json["deviceID"];
  }

  if (json["displayID"].is<uint8_t>()) {
    sensorConfigs[id].displayID = json["displayID"];
  }

  if (json["name"].is<const char*>()) {
    strlcpy(sensorConfigs[id].name, json["name"], sizeof(sensorConfigs[id].name));
  }

  if (json["settings"].is<uint8_t>()) {
    sensorConfigs[id].settings = json["settings"];
  }

  // Set the hasHumidity bit (bit 0) in settings
  if (json["hasHumidity"]) {
    sensorConfigs[id].settings |= 0b00000001; // Set bit 0
  } else {
    sensorConfigs[id].settings &= ~0b00000001; // Clear bit 0
  }

  if (json["channel"].is<uint8_t>()) {
    // Set the channel bits (bits 1-2) in settings
    sensorConfigs[id].settings &= ~0b00000110; // Clear bits 1-2
    uint8_t channel = json["channel"]; // Get only the last 2 bits for channel
    sensorConfigs[id].settings |= ((channel & 0b00000011) << 1) & 0b00000110; // Set bits 1-2 based on channel value
  }

  if (json["correction"].is<int8_t>()) {
    sensorConfigs[id].correction = json["correction"];
  }

  if (json["maxValidDelta"].is<uint8_t>()) {
    sensorConfigs[id].maxValidDelta = json["maxValidDelta"];
  }

  if (json["maxValidInterval"].is<uint8_t>()) {
    sensorConfigs[id].maxValidInterval = json["maxValidInterval"];
  }

  saveConfig(); // Sla bijgewerkte configuratie op
  sensors[id]->setConfig(sensorConfigs[id]); // Update the corresponding THDevice instance with the new config
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleWheatherStationReport() {
  Log.debug("Received POST /weather-station-report request");
  String body = server.arg("plain");  
  // The body is in form-data. The keys needed are: "tempinf", "humidityin"
  // We will extract these values and log them
  float tempf = body.substring(body.indexOf("tempinf=") + 8, body.indexOf("&", body.indexOf("tempinf="))).toFloat();
  // Convert tempf to Celsius
  float tempc = (tempf - 32) * 5.0 / 9.0;
  float humidity = body.substring(body.indexOf("humidityin=") + 11).toFloat();
  THPacket packet = {};
  packet.deviceID = 1;
  packet.channelNo = 0;
  packet.batteryState = 1;
  packet.temperature = tempc;
  packet.humidity = humidity;
  packet.timestamp = millis();
  processPacket(packet);
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

/**
 * Handles unhandlesd routes
 */
void handleNotFound() {
  Log.debug("Received 404 request" + server.uri());
  String message = "File Not Found\n\n";
  server.send(404, "text/plain", message);
}

/******************************************************************************
 *                                                                            *
 * BMP280 sensor functions                                                    *
 *                                                                            *
 *****************************************************************************/
#include "SparkFunBME280.h"

bool bmp280Initialized = false;
// Setting up the BMP280 sensor for local measurements
#define MY_BMP280_ADDRESS 0x76
BME280 bmp280; //Uses default I2C address 0x76

#define LOCAL_MEASUREMENT_INTERVAL 60000 // 1 minute
unsigned long nextLocalMeasurement = 0;


#include "DisplayLogChannel.h"
// Additional log channels
DisplayLogChannel displayLogChannel(Display, Logger::INFO);

#ifdef TELNET_DEBUGGING_ENABLED
/**
 * @brief Checks for incoming telnet connections and accepts them. This allows 
 * for remote logging over telnet. 
 */
bool handleTelnetConnections() {
  // Check for telnet clients
  if (telnetServer.hasClient()) {
      telnetClient = telnetServer.accept();
      Log.trace("Telnet log client accepted");
      return true;
  }
  return false;
}
#endif

/**
 * @brief Takes a local measurement from the BMP280 sensor and returns it as a 
 * THPacket.
 */
THPacket getLocalMeasurement() {
  THPacket packet = {};
  packet.deviceID = 0;
  packet.channelNo = 9;
  packet.batteryState = 1;
  packet.temperature = static_cast<float>(static_cast<int>(bmp280.readTempC() * 10.)) / 10.;
  packet.humidity = bmp280.readFloatHumidity();
  packet.timestamp = millis();
  return packet;
}

/**
 * @brief Enters an infinite loop to handle OTA updates when a critical error 
 * occurs.
 */
void startInifiniteErrorLoop() {
  Log.critical("Entering infinite OTA error loop");
  connectWifi();
  ArduinoOTA.begin();
  while (1) { // Don't proceed, loop forever. Everything else depends upon it
    ArduinoOTA.handle();
    delay(100);
  }
}

/**
 * @brief The setup function runs once when the device is powered on or reset. 
 * It initializes the system, connects to WiFi, sets up the display, sensors, 
 * and logging channels, and prepares the device for operation. If any critical 
 * initialization step fails (like WiFi connection or sensor setup), it logs the 
 * error and enters an infinite loop to prevent further execution.
 */
void setup() {
  #ifdef SERIAL_DEBUGGING_ENABLED
    // Setting up serial logging
    Serial.begin(115200);
    delay(5000); // Wait for system to stabilize and allow time to connect serial monitor after reset
    Log.addChannel(new SerialChannel(SERIAL_DEBUGGING_LEVEL));
    Log.trace("Serial logging initialized");
  #endif
  loadConfig(CONFIG_VERSION);
  Wire.begin(0, 2);  // set I2C pins (SDA = GPIO0, SCL = GPIO2), default clock is 100kHz
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!Display.begin()) {
    Log.error("SSD1306 allocation failed");
  } else {
    Log.addChannel(&displayLogChannel);
    Log.trace("SSD1306 allocation successful");
  }
  connectWifi();
  #ifdef TELNET_DEBUGGING_ENABLED
    Log.info("Starting telnet logging, waiting for client to connect...");
    telnetServer.begin();
    telnetChannel.setLogLevel(config.logLevel);
    Log.addChannel(&telnetChannel);
    long unsigned int timeout = millis() + 20000; // Wait up to 20 seconds for a telnet client to connect before proceeding without telnet logging
    bool connected = false;
    while (!connected && (millis() < timeout))
    {
      connected = handleTelnetConnections();
      delay(100);
    }
  #endif

  Log.info("Starting OTA service");
  ArduinoOTA.begin();

  bmp280.setI2CAddress(MY_BMP280_ADDRESS);
  //The I2C address must be set before .begin() otherwise the cal values will fail to load.
  if(bmp280.beginI2C() == false) {
    Log.error("BMP280 init failed");
  } else {
    bmp280Initialized = true;
    Log.info("BMP280 is ready to use");
  } 

  Log.info("Starting HTTP server on port 80");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/config", HTTP_GET, handleGetConfig);
  server.on("/config/reset", HTTP_POST, handleResetConfig);
  server.on("/config", HTTP_PATCH, handlePatchConfig);
  server.on("/config/sensors", HTTP_GET, handleGetConfigSensors);
  server.on("/data/report/", HTTP_POST, handleWheatherStationReport);
  server.on(UriBraces("/config/sensors/{}"), HTTP_PATCH, handlePatchConfigSensors);
  server.onNotFound(handleNotFound);
  server.begin();

  Log.info("Starting ThingSpeak service");
  ThingSpeak.begin(client);

  Log.info("Initializing devices");
  for (int i=0; i<SENSOR_COUNT; i++) {
    Log.trace("Initializing device " + String(i) + " " + String(sensorConfigs[i].name));
    sensors[i] = new THSensor(sensorConfigs[i]);
    Display.updateDeviceInfo(sensors[i]->displayID, sensors[i]->getLastStatus());
  }

  Log.info("Starting 433 MHz receiver on interrupt pin " + String(THRECEIVER_PIN));
  receiver.begin(THRECEIVER_PIN);

  Log.info("Setup completed successfully. System is ready.");
  delay(1000); // Short delay to ensure all setup logs are sent before starting normal operation
  Display.startNormal();
  displayLogChannel.setLogLevel(Logger::WARNING); // Only show warnings and above on the display to avoid cluttering it with too much information
}

/**
 * @brief The loop function runs continuously after setup() has completed. It
 * handles OTA updates, processes incoming packets from the 433 MHz receiver, 
 * takes periodic measurements from the BMP280 sensor, checks for device timeouts, 
 * and updates ThingSpeak with the latest data. The loop also includes a watchdog 
 * mechanism to ensure the system remains responsive, and it gives the CPU some 
 * rest by introducing a small delay when appropriate. 
 */
void loop() {
  unsigned long now = millis();
  ArduinoOTA.handle();
  // Handle incoming HTTP requests
  server.handleClient();
  // Check for telnet clients
  if (telnetServer.hasClient()) {
      telnetClient = telnetServer.accept();
      Log.info("New telnet client connected");
  }
  // Check for incoming packets from the 433 MHz receiver
  if (receiver.isAvailable()) {
    THPacket packet = receiver.getLastReceived();
    processPacket(packet);
  }
  // Periodically take local measurements from the BMP280 sensor
  if (now >= nextLocalMeasurement) {
    if (!bmp280Initialized) {
      Log.warning("BMP280 sensor not initialized, skipping local measurement");
    } else {
      THPacket localPacket = getLocalMeasurement();
      processPacket(localPacket);
    }
    nextLocalMeasurement = now + LOCAL_MEASUREMENT_INTERVAL;
  }
  // Update WiFi status on the display
  Display.updateWifiStatus(WiFi.status() == WL_CONNECTED, WiFi.RSSI());
  // Timeout watchdog
  for (int n = 0; n < SENSOR_COUNT; n++) {
    THSensor *d = sensors[n];
    d->checkTimeout();
  }
  // Periodically update ThingSpeak with the latest measurements and status updates
  if (now >= nextThingSpeakUpdate) {
    updateThingSpeak();
    nextThingSpeakUpdate = now + THINGSPEAK_UPDATE_INTERVAL;
  } else {
    // Give the cpu some rest
    delay(100);
  }
}
