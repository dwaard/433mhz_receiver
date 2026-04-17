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
#include "SparkFunBME280.h"
// JSON serialization for dashboard communication
#include <ArduinoJson.h>

#include "secrets.h"

/******************************************************************************
 *                                                                            *
 * Soft- and hardware configuration                                           *
 *                                                                            *
 *****************************************************************************/

// Setting up Serial debugging (optional, can impact performance)
#define SERIAL_DEBUGGING_ENABLED 1

#if SERIAL_DEBUGGING_ENABLED > 0
  #include "Channels/SerialChannel.h"
  #define SERIAL_DEBUGGING_LEVEL Logger::INFO
#endif

// Setting up Telnet debugging (optional, can impact performance)
#define TELNET_DEBUGGING_ENABLED 1

#if TELNET_DEBUGGING_ENABLED > 0
  #include <WiFiServer.h>
  #include <WiFiClient.h>
  #include "Channels/TelnetChannel.h"
  #define TELNET_SERVER_PORT 23
  #define TELNET_DEBUGGING_LEVEL Logger::DEBUG
#endif

// Pin for the 433 MHz receiver. 
// This should be an interrupt-capable pin for best performance.
#define THRECEIVER_PIN 3

// I2C configuration for BMP280 sensor
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 2

// Local measurement (BMP280) configuration
#define BMP280_I2C_ADDRESS 0x76
#define BMP280_DEVICE_ID 0x00
#define BMP280_DEVICE_CHANNEL 0

#define LOCAL_MEASUREMENT_INTERVAL 60000 // 1 minute

// Weather station (WS2910) configuration macros
#define WS2910_IN_DEVICE_ID 0x01
#define WS2910_IN_DEVICE_CHANNEL 0
#define WS2910_OUT_DEVICE_ID 0x02
#define WS2910_OUT_DEVICE_CHANNEL 0

// Dashboard configuration
#define DASHBOARD_HOST "192.168.8.5"
#define DASHBOARD_PORT 80
#define DASHBOARD_PATH "/data/packet"
#define DASHBOARD_POST_TIMEOUT 2000 // ms

/******************************************************************************
 *                                                                            *
 * Logging functions                                                          *
 *                                                                            *
 *****************************************************************************/
#if SERIAL_DEBUGGING_ENABLED > 0
  #define SETUP_SERIAL_LOGGING() {\
     Log.info("Starting serial logging...");\
     delay(5000); /* Wait for system to stabilize and allow time to connect serial monitor after reset */\
     Log.addOwnedChannel(new SerialChannel(SERIAL_DEBUGGING_LEVEL));\
     Log.trace("Serial logging initialized");\
  }
#endif

#if TELNET_DEBUGGING_ENABLED > 0
  WiFiServer telnetServer(TELNET_SERVER_PORT);
  WiFiClient telnetClient;
  TelnetChannel telnetChannel(&telnetClient, TELNET_DEBUGGING_LEVEL);

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

  #define SETUP_TELNET_LOGGING() {\
     Log.info("Starting telnet logging, waiting for client to connect...");\
     telnetServer.begin();\
     Log.addChannel(&telnetChannel);\
     long unsigned int timeout = millis() + 20000; /* Wait up to 20 seconds for a telnet client to connect before proceeding without telnet logging */\
     bool connected = false;\
     while (!connected && (millis() < timeout)) {\
       connected = handleTelnetConnections();\
       delay(100);\
     }\
  }
#endif

/******************************************************************************
 *                                                                            *
 * Wifi connection functions                                                  *
 *                                                                            *
 *****************************************************************************/

WiFiClient httpClient;

/**
 * @brief Sends a packet payload to the dashboard as an HTTP POST request.
 */
bool postPacketToDashboard(const THPacket &packet) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi(SECRET_SSID, SECRET_PASS);
  }

  if (!httpClient.connect(DASHBOARD_HOST, DASHBOARD_PORT)) {
    Log.warning("Dashboard POST failed: could not connect to " + String(DASHBOARD_HOST) + ":" + String(DASHBOARD_PORT));
    return false;
  }

  JsonDocument json;
  json["deviceID"] = packet.deviceID;
  json["channelNo"] = packet.channelNo;
  json["batteryState"] = packet.batteryState;
  json["temperature"] = packet.temperature;
  json["humidity"] = packet.humidity;

  String payload;
  serializeJson(json, payload);

  httpClient.println(String("POST ") + DASHBOARD_PATH + " HTTP/1.1");
  httpClient.println(String("Host: ") + DASHBOARD_HOST);
  httpClient.println("Content-Type: application/json");
  httpClient.println(String("Content-Length: ") + payload.length());
  httpClient.println("Connection: close");
  httpClient.println();
  httpClient.println(payload);

  unsigned long timeout = millis() + DASHBOARD_POST_TIMEOUT;
  while (!httpClient.available() && millis() < timeout) {
    delay(10);
  }

  if (!httpClient.available()) {
    Log.warning("Dashboard POST timed out waiting for response");
    httpClient.stop();
    return false;
  }

  String statusLine = httpClient.readStringUntil('\n');
  statusLine.trim();
  bool success = statusLine.startsWith("HTTP/1.1 2") || statusLine.startsWith("HTTP/1.0 2");

  while (httpClient.available()) {
    httpClient.read();
  }
  httpClient.stop();

  if (!success) {
    Log.warning("Dashboard POST failed. Response: " + statusLine);
    return false;
  }

  Log.trace("Dashboard POST successful");
  return true;
}

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

/**
 * @brief Processes an incoming THPacket.
 */
void processPacket(THPacket packet) {
    char sentence[32];
    sprintf(sentence, "0x%02X|%1d|%s|%5.1f|%3d", 
      packet.deviceID, packet.channelNo, packet.batteryState? "N" : "L", packet.temperature, packet.humidity
    );
    Log.info(sentence);
    postPacketToDashboard(packet);
}


/******************************************************************************
 *                                                                            *
 * Webserver request handler functions                                        *
 *                                                                            *
 *****************************************************************************/

ESP8266WebServer server(80);

#define SETUP_WEB_SERVER() {\
  server.on("/", HTTP_GET, handleRoot);\
  server.on("/data/report/", HTTP_POST, handleWheatherStationReport);\
  server.onNotFound(handleNotFound);\
  server.begin();\
  Log.info("Web server started");\
}

/**
 * Handles GET requests to the root (/) endpoint.
 * 
 * It returns a JSON response containing an array of sensors with their actual
 * data.
 */
void handleRoot() {
  Log.trace("Received GET / request");
  server.send(200, "application/json", "{\"message\":\"Welcome to the THGateway!\"}");
}

String getValueStr(const String& data, const char* key) {
  int keyIndex = data.indexOf(String(key) + "=");
  if (keyIndex == -1) {
    return "";
  }
  int valueStart = keyIndex + strlen(key) + 1; // +1 for the '=' character
  int valueEnd = data.indexOf("&", valueStart);
  if (valueEnd == -1) {
    valueEnd = data.length();
  }
  return data.substring(valueStart, valueEnd);
}

void handleWheatherStationReport() {
  Log.trace("Received POST /data/report request");
  String body = server.arg("plain");
  // The body is in form-data. The keys needed are: "tempinf", "humidityin"
  // We will extract these values and log them
  String value = getValueStr(body, "tempinf");
  if (value == "") {
    Log.warning("Received weather station report without tempinf");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing tempinf\"}");
    return;
  }
  float tempIn = (value.toFloat() - 32) * 5.0 / 9.0;
  value = getValueStr(body, "humidityin");
  if (value == "") {
    Log.warning("Received weather station report without humidityin");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing humidityin\"}");
    return;
  }
  float humidityIn = value.toFloat();
  value = getValueStr(body, "tempf");
  if (value == "") {
    Log.warning("Received weather station report without tempf");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing tempf\"}");
    return;
  }
  float tempOut = (value.toFloat() - 32) * 5.0 / 9.0;
  value = getValueStr(body, "humidity");
  if (value == "") {
    Log.warning("Received weather station report without humidity");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing humidity\"}");
    return;
  }
  float humidityOut = value.toFloat();
  value = getValueStr(body, "wh65batt");
  if (value == "") {
    Log.warning("Received weather station report without wh65batt");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing wh65batt\"}");
    return;
  }
  bool batteryState = value.toInt() == 0; // wh65batt=1 indicates low battery for the outdoor sensor array, so we invert it for our batteryState

  THPacket packet = {};
  packet.deviceID = WS2910_IN_DEVICE_ID;
  packet.channelNo = WS2910_IN_DEVICE_CHANNEL;
  packet.batteryState = true;
  packet.temperature = tempIn;
  packet.humidity = humidityIn;
  packet.timestamp = millis();
  processPacket(packet);
  packet.deviceID = WS2910_OUT_DEVICE_ID;
  packet.channelNo = WS2910_OUT_DEVICE_CHANNEL;
  packet.batteryState = batteryState; // The outdoor sensor array is battery powered
  packet.temperature = tempOut;
  packet.humidity = humidityOut;
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
bool bmp280Initialized = false;

BME280 bmp280; //Uses default I2C address 0x76

unsigned long nextLocalMeasurement = 0;

#define SETUP_BMP280() {\
  bmp280.setI2CAddress(BMP280_I2C_ADDRESS);\
  /* The I2C address must be set before .begin() otherwise the cal values will fail to load. */\
  if(bmp280.beginI2C() == false) {\
    Log.error("BMP280 init failed");\
  } else {\
    bmp280Initialized = true;\
    Log.info("BMP280 is ready to use");\
  } \
}

#define LOOP_BMP280() {\
  if (millis() >= nextLocalMeasurement) {\
    nextLocalMeasurement = millis() + LOCAL_MEASUREMENT_INTERVAL;\
    if (bmp280Initialized) {\
      THPacket packet = {};\
      packet.deviceID = BMP280_DEVICE_ID;\
      packet.channelNo = BMP280_DEVICE_CHANNEL;\
      packet.batteryState = 1;\
      packet.temperature = static_cast<float>(static_cast<int>(bmp280.readTempC() * 10.)) / 10.;\
      packet.humidity = bmp280.readFloatHumidity();\
      packet.timestamp = millis();\
      processPacket(packet);\
    } else {\
      Log.warning("Skipping local measurement, BMP280 failed to initialize");\
    }\
  }\
}

/**
 * @brief The setup function runs once when the device is powered on or reset. 
 * 
 */
void setup() {
  Serial.begin(115200);
  #if SERIAL_DEBUGGING_ENABLED > 0
    SETUP_SERIAL_LOGGING();
  #endif
  connectWifi(SECRET_SSID, SECRET_PASS);
  #if TELNET_DEBUGGING_ENABLED > 0
    SETUP_TELNET_LOGGING();
  #endif
  Log.info("Starting OTA service");
  ArduinoOTA.begin();
  
  SETUP_WEB_SERVER();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // set I2C pins (SDA = GPIO0, SCL = GPIO2), default clock is 100kHz
  SETUP_BMP280();
  Log.info("Starting 433 MHz receiver on interrupt pin " + String(THRECEIVER_PIN));
  receiver.begin(THRECEIVER_PIN);
  Log.info("Setup completed successfully. System is ready.");
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
}
