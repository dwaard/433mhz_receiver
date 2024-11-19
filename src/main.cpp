/******************************************************************************

433.92 MHz Weather station receiver for Nexus-TH compatible units
More information at:
https://www.onetransistor.eu/2024/01/receive-lpd433-weather-unit-nexus.html

This sample code requires ESP8266 or AVR Arduino with 433.92 MHz receiver.
Optional for ESP8266: 0.96" I2C OLED and MQTT functionality

******************************************************************************/
#include <Arduino.h>
#include "secrets.h"
#include "THReciever.h"
#include "THDevice.h"

/* Platform specific stuff */
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

#define RECEIVER_PIN 4

const char* ssid = SECRET_SSID;   // your network SSID (name) 
const char* pass = SECRET_PASS;   // your network password

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

WiFiClient  client;

THReceiver receiver = THReceiver();

unsigned long last_sent = 0;


const int MAX_STATUS_SIZE = 2000;
char* status = new char[MAX_STATUS_SIZE];

THDevice **devices;

const int DEVICE_COUNT = 5;

void initDevices() {
  devices = new THDevice*[DEVICE_COUNT];
  devices[0] = new THDevice(0xD6, 1, "Buiten slk",  0  );
  devices[1] = new THDevice(0xF6, 1, "Garage"    , -1.3);
  devices[2] = new THDevice(0x59, 1, "Keuken"    ,  0  );
  devices[3] = new THDevice(0x80, 3, "Kantoor"   ,  0  );
  devices[4] = new THDevice(0xE5, 2, "Kelder"    ,  0  );
}

void resetStatus() {
  strcpy(status, "");
}

void addStatus( const char *msg) {
  if (strlen(status) == 0) {
    strcpy(status, msg);
  } else {
    strlcat(status, " | ", MAX_STATUS_SIZE);
    strlcat(status, msg, MAX_STATUS_SIZE);
  }
}

void connectWifi() {
  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(WiFi.status());
      delay(5000);     
    } 
    Serial.println("\nConnected.");
    addStatus("Device: (re)connected");
  }
}

void initWifi() {
  Serial.println("Initializing WiFi...");
  WiFi.mode(WIFI_STA);
  connectWifi();
}

int findDevice(uint8_t deviceID) {
  for (int n = 0; n < DEVICE_COUNT; n++) {
    THDevice *d = devices[n];
    if (d->hasID(deviceID))
      return n;
  }
  return -1;
}

void updateThingSpeak() {
  Serial.println("Trying to update ThingSpeak.");
  bool hasUpdates = false;

  for (int n = 0; n < DEVICE_COUNT; n++) {
    THDevice *d = devices[n];
    if (d->hasUpdates()) {
      THPacket m = d->getLastRecieved();
      ThingSpeak.setField(n + 1, m.temperature);
      char buffer[40];
      d->printName(buffer);
      Serial.print("  ");
      Serial.print(buffer);
      Serial.println(" has updates.");
      hasUpdates = true;
    }
    if (d->hasStatusupdates()) {
      addStatus(d->getStatusupdates());
      d->resetStatus();
    }
  }

  if (strlen(status) > 0) {
    Serial.println("  There are statusupdates");
    ThingSpeak.setStatus(status);
    resetStatus();
    hasUpdates = true;
  }
  
  if (hasUpdates) {
    // reconnect if needed
    connectWifi();
    //write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if(x == 200){
      Serial.println("Channel update successful.");
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(x));
      char msg[35];
      sprintf(msg, "ThingSpeak: Previous update was not succesful (%d)", x);
      addStatus(msg);
    }
  } else {
    Serial.println("Nothing to update.");
  }
}

void setup() {
  Serial.begin(115200);
  resetStatus();
  initWifi();
  Serial.println("Initializing ThingSpeak...");
  ThingSpeak.begin(client);
  Serial.println("Initializing the rest...");
  receiver.begin(RECEIVER_PIN);
  initDevices();
  Serial.println("Start receiving packets...");
}

void loop() {
  if (receiver.isAvailable()) {
    THPacket packet = receiver.getLastReceived();
    char sentence[20];
    sprintf(sentence, "%X;%s;%i;%.1f;%i", packet.deviceID, packet.batteryState ? "N" : "L", packet.channelNo, packet.temperature, packet.humidity);
    Serial.println(sentence);

    int i = findDevice(packet.deviceID);
    if (i >= 0) {
      THDevice *d = devices[i];
      d->process(packet);
    } else {
      // Add a status about an unknown device
      char buffer[40];
      sprintf(buffer, "Onbekend device: %s", sentence);
      addStatus(buffer);
    }
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

