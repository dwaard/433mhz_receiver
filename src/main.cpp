/******************************************************************************

433.92 MHz Weather station receiver for Nexus-TH compatible units
More information at:
https://www.onetransistor.eu/2024/01/receive-lpd433-weather-unit-nexus.html

This sample code requires ESP8266 or AVR Arduino with 433.92 MHz receiver, 
an 128x64 pixel I2C OLED display and BMP280

******************************************************************************/
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "SparkFunBME280.h"

#include "Logger.h"
#include "channels/TelnetChannel.h"
#include "DisplayLogChannel.h"
#include "ThingSpeakLogChannel.h"

// Enable or disable serial debugging. Set to 1 to enable, 0 to disable. 
#define SERIAL_DEBUGGING_ENABLED 1
#ifdef SERIAL_DEBUGGING_ENABLED
  #define SERIAL_DEBUGGING_LEVEL Logger::TRACE
  #pragma message("Serial debugging is ENABLED. This may impact performance. Set SERIAL_DEBUGGING_ENABLED to 0 to disable.")
  #include "channels/SerialChannel.h"

#endif

#include "THReciever.h"
#include "THDevice.h"
#include "secrets.h"

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#pragma message("Building for ESP8266 platform...")

#elif defined(ARDUINO_ARCH_AVR)
#include <WiFi.h>
#pragma message("Building for Arduino AVR platform...")

#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#pragma message("Building for ESP32 platform...")
#endif

#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const char* ssid = SECRET_SSID;   // your network SSID (name) 
const char* pass = SECRET_PASS;   // your network password

int status = WL_IDLE_STATUS;     // the WiFi radio's status
WiFiClient  client;

#define TELNET_DEBUGGING_ENABLED 0
#ifdef TELNET_DEBUGGING_ENABLED
  WiFiServer telnetServer(23);
  WiFiClient telnetClient;
  TelnetChannel telnetChannel(&telnetClient, Logger::DEBUG);
#endif

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

String lines[] = {String(""), String(""), String(""), String(""), String("")};
unsigned int lineptr = 0;

#define MY_BMP280_ADDRESS 0x76
BME280 bmp280; //Uses default I2C address 0x76

#define THRECEIVER_PIN 3
THReceiver receiver = THReceiver();

unsigned long last_sent = 0;

THDevice **devices;

const int DEVICE_COUNT = 7;

DisplayLogChannel displayLogChannel(Logger::DEBUG);
ThingSpeakLogChannel thingSpeakLogChannel(Logger::DEBUG);

void render() {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 1);
  display.clearDisplay();
  // Display the lines
  for (unsigned int index = 0; index < 5; index++) {
    display.println(lines[(lineptr + index) % 8]);
  }
  for (unsigned int index = 0; index < 3; index++) {
    display.println(displayLogChannel.getRecent(index));
  }
  display.display(); 
}


void println(String line) {
  lines[lineptr] = line;
  lineptr = (lineptr + 1) % 8;
  render();
}

void initDevices() {
  Log.trace("Creating THDevice instances for each known device");
  devices = new THDevice*[DEVICE_COUNT];
  devices[0] = new THDevice(0xA0, 1, "BT-S",  0  , THDevice::DISABLE_HUMIDITY);
  devices[1] = new THDevice(0xE5, 2, "Garg"    , -1.3);
  devices[2] = new THDevice(0x22, 3, "Kkn"    ,  0  );
  devices[3] = new THDevice(0xD7, 1, "Slkr",  0  , THDevice::DISABLE_HUMIDITY);
  devices[4] = new THDevice(0x53, 2, "Kldr"    ,  0  );
  devices[5] = new THDevice(0x00, 9, "Kntr"    ,  0  );
  devices[6] = new THDevice(0x16, 1, "BT-G"    ,  0  );
}

int findDevice(uint8_t deviceID) {
  for (int n = 0; n < DEVICE_COUNT; n++) {
    THDevice *d = devices[n];
    if (d->hasID(deviceID))
      return n;
  }
  return -1;
}

const char* wl_status_to_string(wl_status_t status) {
  switch (status) {               //"This string has exactly 32 chars"
    case WL_NO_SHIELD:       return "WiFi radio is not available";
    case WL_IDLE_STATUS:     return "WiFi Radio in in idle";
    case WL_NO_SSID_AVAIL:   return "No SSIDs available";
    case WL_SCAN_COMPLETED:  return "WiFi scan completed";
    case WL_CONNECTED:       return "Connected to WiFi";
    case WL_CONNECT_FAILED:  return "Connection to WiFi failed";
    case WL_CONNECTION_LOST: return "WiFi connection lost";
    case WL_WRONG_PASSWORD:  return "Wrong password";
    case WL_DISCONNECTED:    return "Disconnected from WiFi";
  }
  // Create a static buffer to hold the status value
  static char buffer[32];
  snprintf(buffer, sizeof(buffer), "UNKNOWN STATUS: %d", status);
  return buffer;
}

void connectWifi(const char* ssid, const char* pass) {
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
  if (status == WL_IDLE_STATUS) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Init Wifi: %s", ssid);
    Log.info(buffer);
    WiFi.mode(WIFI_STA);
  }
  status = WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
  while(WiFi.status() != WL_CONNECTED) {
    Log.warning("Conn: " + String(wl_status_to_string(WiFi.status())));
    delay(5000);     
  }
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "IP: %s", WiFi.localIP().toString().c_str());
  Log.info("WiFi connection successful");
}

void updateThingSpeak() {
  Log.trace("Updating ThingSpeak channel...");
  bool hasUpdates = false;

  for (int n = 0; n < DEVICE_COUNT; n++) {
    THDevice *d = devices[n];
    if (d->hasUpdates()) {
      THPacket m = d->getLastRecieved();
      ThingSpeak.setField(n + 1, m.temperature);
      hasUpdates = true;
    }
    // TODO add updates from the ThingSpeakChannel
    if (thingSpeakLogChannel.hasStatusToSend()) {
      ThingSpeak.setStatus(thingSpeakLogChannel.getStatusForSending());
      hasUpdates = true;
    }
  }
  
  if (hasUpdates) {
    // reconnect if needed
    connectWifi(ssid, pass);
    //write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if(x == 200) {
      Log.info("ThingSpeak update successful.");
    }
    else {
      Log.error("THINGSPEAK" + String("ThingSpeak error " + String(x)));
    }
  } else {
    Log.trace("No updates to send to ThingSpeak.");
    println("Nothing to update.");
  }
}

void startInifiniteErrorLoop() {
  Log.critical("Entering infinite OTA error loop");
  connectWifi(ssid, pass);
  ArduinoOTA.begin();
  while (1) { // Don't proceed, loop forever. Everything else depends upon it
    ArduinoOTA.handle();
    delay(100);
  }
}

#ifdef TELNET_DEBUGGING_ENABLED
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
    Log.addChannel(new SerialChannel(SERIAL_DEBUGGING_LEVEL));
  #endif

  Wire.begin(0, 2);  // set I2C pins (SDA = GPIO0, SCL = GPIO2), default clock is 100kHz
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Log.error("SSD1306 allocation failed");
  } else {
    display.setRotation(2);
    Log.addChannel(&displayLogChannel);
    Log.trace("SSD1306 allocation successful");
  }

  connectWifi(ssid, pass);
  #ifdef TELNET_DEBUGGING_ENABLED
    // Setting up telnet logging
    telnetServer.begin();
    Log.addChannel(&telnetChannel);
    int timeout = 0; bool connected = false;
    while (!connected && timeout < 100)
    {
      connected = handleTelnetConnections();
      timeout++;
      delay(100);
    }
  #endif

  Log.trace("Starting OTA service...");
  ArduinoOTA.begin();

  Log.trace("Initializing receiver and sensors...");
  bmp280.setI2CAddress(MY_BMP280_ADDRESS);
  //The I2C address must be set before .begin() otherwise the cal values will fail to load.

  if(bmp280.beginI2C() == false) {
    Log.error("BMP280 initialization failed");
  } else {
    Log.trace("BMP280 is ready to use");
  } 

  Log.trace("Starting ThingSpeak service...");
  println("Start ThingSpeak");
  ThingSpeak.begin(client);

  Log.trace("Initializing receiver...");
  println("Start receiver");
  initDevices();
  receiver.begin(THRECEIVER_PIN);

  Log.info("Setup completed successfully. System is ready.");
  println("Ready.");
}


unsigned long prevLocalMeasurement = 0;

void processPacket(THPacket packet) {
    Log.trace("Received packet - Device ID: 0x" + String(packet.deviceID, HEX) + ", Channel: " + String(packet.channelNo) + ", Battery: " + (packet.batteryState ? "OK" : "LOW") + ", Temperature: " + String(packet.temperature) + "°C, Humidity: " + String(packet.humidity) + "%");
    int i = findDevice(packet.deviceID);
    if (i >= 0) {
      THDevice *d = devices[i];
      d->process(packet);
    } else {
      char sentence[32];
      sprintf(sentence, "UNKNOWN: 0x%02X %4.1f", packet.deviceID, packet.temperature);
      Log.warning(sentence);
    }
}

void loop() {
  ArduinoOTA.handle();
  if (receiver.isAvailable()) {
    THPacket packet = receiver.getLastReceived();
    processPacket(packet);
  }
  unsigned long now = millis();
  if (prevLocalMeasurement == 0 || now - prevLocalMeasurement > 60000) {
    THPacket packet = {};
    packet.deviceID = 0;
    packet.channelNo = 9;
    packet.batteryState = 1;
    packet.temperature = static_cast<float>(static_cast<int>(bmp280.readTempC() * 10.)) / 10.;
    packet.humidity = bmp280.readFloatHumidity();
    packet.timestamp = now;
    processPacket(packet);
    prevLocalMeasurement = now;
  }

  // Timeout watchdog
  for (int n = 0; n < DEVICE_COUNT; n++) {
    THDevice *d = devices[n];
    d->checkTimeout();
  }

  unsigned long current = millis();
  if ((current - last_sent) > 60000 || last_sent == 0) {
    updateThingSpeak();
    last_sent = current;
  } else {
    // Give the cpu some rest
    delay(100);
  }
}
