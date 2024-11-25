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
// #include "WiFiUtils.h"
#include "THReciever.h"
#include "THDevice.h"
#include "secrets.h"

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#pragma "Building for ESP8266 platform..."

#elif defined(ARDUINO_ARCH_AVR)
#include <WiFi.h>
#pragma "Building for Arduino AVR platform..."

#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#pragma "Building for ESP32 platform..."
#endif

#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const char* ssid = SECRET_SSID;   // your network SSID (name) 
const char* pass = SECRET_PASS;   // your network password

WiFiClient  client;


unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

String lines[] = {String(""), String(""), String(""), String(""), String(""), String(""), String(""), String("")};
unsigned int lineptr = 0;

#define MY_BMP280_ADDRESS 0x76
BME280 bmp280; //Uses default I2C address 0x76

#define THRECEIVER_PIN 3
THReceiver receiver = THReceiver();

unsigned long last_sent = 0;

const int MAX_STATUS_SIZE = 2000;
char* status = new char[MAX_STATUS_SIZE];
String statusString;

THDevice **devices;

const int DEVICE_COUNT = 6;


void render() {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 1);
  display.clearDisplay();
  // Display the lines
  for (unsigned int index = 0; index < 8; index++) {
    display.println(lines[(lineptr + index) % 8]);
  }
  display.display(); 
}


void println(String line) {
  lines[lineptr] = line;
  lineptr = (lineptr + 1) % 8;
  render();
}

void initDevices() {
  devices = new THDevice*[DEVICE_COUNT];
  devices[0] = new THDevice(0x99, 1, "Buiten slk",  0  );
  devices[1] = new THDevice(0xF6, 1, "Garage"    , -1.3);
  devices[2] = new THDevice(0x22, 3, "Keuken"    ,  0  );
  devices[3] = new THDevice(0xD7, 1, "Slaapkamer",  0  , THDevice::DISABLE_HUMIDITY);
  devices[4] = new THDevice(0xE5, 2, "Kelder"    ,  0  );
  devices[5] = new THDevice(0x00, 9, "BMP280"    ,  0  );
}

void resetStatus() {
  statusString = String("");
}

void addStatus(String msg) {
  if (statusString.length() == 0) {
    statusString = String(msg);
  } else {
    statusString.concat(" | " + msg);
  }
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
  switch (status) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_WRONG_PASSWORD: return "WL_WRONG_PASSWORD";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
  }
  // Create a static buffer to hold the status value
  static char buffer[32];
  snprintf(buffer, sizeof(buffer), "UNKNOWN STATUS: %d", status);
  return buffer;
}

void connectWifi(const char* ssid, const char* pass) {
  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED) {
    println("SSID: " + String(ssid));
    WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
    while(WiFi.status() != WL_CONNECTED){
      println(wl_status_to_string(WiFi.status()));
      delay(5000);     
    } 
    println("IP: " + WiFi.localIP().toString());
  }
}

void initWifi(const char* ssid, const char* pass) {
  println("Initializing WiFi...");
  WiFi.mode(WIFI_STA);
  connectWifi(ssid, pass);
}

void updateThingSpeak() {
  println("Updating ThingSpeak.");
  bool hasUpdates = false;

  for (int n = 0; n < DEVICE_COUNT; n++) {
    THDevice *d = devices[n];
    if (d->hasUpdates()) {
      THPacket m = d->getLastRecieved();
      ThingSpeak.setField(n + 1, m.temperature);
      hasUpdates = true;
    }
    if (d->hasStatusupdates()) {
      addStatus(d->getStatusupdates());
      d->resetStatus();
    }
  }

  if (statusString.length() > 0) {
    println("Sent status");
    ThingSpeak.setStatus(statusString);
    resetStatus();
    hasUpdates = true;
  }
  
  if (hasUpdates) {
    // reconnect if needed
    connectWifi(ssid, pass);
    //write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if(x == 200) {
      println("ThingSpeak update OK.");
    }
    else {
      String msg = String("ThingSpeak error " + String(x));
      println(msg);
      addStatus(msg);
    }
  } else {
    println("Nothing to update.");
  }
}


void scan() {
  byte error, address;
  int nDevices;

  println("Scanning I2C adresses");

  nDevices = 0;
  String result = String("");
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    String deviceId = String("0x" + String(address<16 ? "0" : "") + String(address, HEX));
    if (error == 0)
    {
      result.concat(String(deviceId + " "));
      nDevices++;
    }
    else if (error==4)
    {
      println("Unknown error at address " + deviceId);
    }
  }
  println(String("Found " + String(nDevices) +" devices."));
  if (nDevices > 0)
    println(result);
}

void startInifiniteErrorLoop() {
  initWifi(ssid, pass);
  ArduinoOTA.begin();
  while (1) { // Don't proceed, loop forever. Everything else depends upon it
    ArduinoOTA.handle();
    delay(100);
  }
}

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(115200);
  resetStatus();
  addStatus("Receiver opgestart");

  Wire.begin(0, 2);  // set I2C pins (SDA = GPIO0, SCL = GPIO2), default clock is 100kHz
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    addStatus("Display connect fail!");
  } else {
    display.setRotation(2);
    println("Display connected.");
  }

  // scan();

  bmp280.setI2CAddress(MY_BMP280_ADDRESS);
  //The I2C address must be set before .begin() otherwise the cal values will fail to load.

  if(bmp280.beginI2C() == false) {
    addStatus("BMP280 connect fail!");
    println("BMP280 connect fail!");
  } else {
    println("BMP280 connected.");
  } 
  initWifi(ssid, pass);

  println("Start OTA");
  ArduinoOTA.begin();

  println("Start ThingSpeak");
  ThingSpeak.begin(client);

  println("Start receiver");
  initDevices();
  receiver.begin(THRECEIVER_PIN);

  println("Ready.");
}

void printData(THPacket packet) {
    char sentence[32];
    sprintf(sentence, "%02X %i %s %5.1f  %3i", packet.deviceID, packet.channelNo, packet.batteryState ? " " : "L", packet.temperature, packet.humidity);
    println(sentence);
}

unsigned long prevLocalMeasurement = 0;

void processPacket(THPacket packet) {
    printData(packet);
    int i = findDevice(packet.deviceID);
    if (i >= 0) {
      THDevice *d = devices[i];
      d->process(packet);
    } else {
      char sentence[32];
      sprintf(sentence, "UNKNOWN: 0x%02X %4.1f", packet.deviceID, packet.temperature);
      println(String(sentence));
      // Add a status about an unknown device
      sprintf(sentence, "%02X;%i;%s;%.1f;%i", packet.deviceID, packet.channelNo, packet.batteryState ? "N" : "L", packet.temperature, packet.humidity);
      addStatus(String("Onbekend device: " + String(sentence)));
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
    packet.temperature = bmp280.readTempC();
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
