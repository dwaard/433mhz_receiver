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

const THDevice devices[] = 
{
  THDevice(0x1D, 1, "Buiten slk",  0  ),
  THDevice(0xF6, 1, "Garage"    , -1.3),
  THDevice(0x59, 1, "Keuken"    ,  0  ),
  THDevice(0x80, 1, "Kantoor"   ,  0  ),
  THDevice(0xE5, 1, "Kelder"    ,  0  ),
};

void initWifi() {
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
  }
}

int findDeviceIndex(THPacket p) {
  int size = sizeof(devices);
  for (int n=0; n<size; n++) {
    THDevice d = devices[n];
    if (d.hasID(p.deviceID))
      return n;
  }
  return -1;
}

void processDecodedData(THPacket m) {
  char sentence[20];
  sprintf(sentence, "%X;%s;%i;%.1f;%i", m.deviceID, m.batteryState ? "N" : "L", m.channelNo, m.temperature, m.humidity);
  Serial.println(sentence);

  unsigned int deviceIndex = findDeviceIndex(m);
  if (deviceIndex >= 0) {
    // Add the measurement to ThingSpeak
    THDevice d = devices[deviceIndex];
    ThingSpeak.setField(deviceIndex + 1, m.temperature);
    if (!m.batteryState) {
      char buffer[40];
      d.printName(buffer);
      char status[strlen(buffer) + 15];
      sprintf(status, "Batterij %s laag", buffer);
      ThingSpeak.setStatus(status);
    }
  } else {
    // Add a status about an unknown device
    char buffer[40];
    sprintf(buffer, "Unknown device: %s", sentence);
    ThingSpeak.setStatus(buffer);
  }

  unsigned long current = millis();
  if ((current - last_sent) > 60000 || last_sent == 0) {
    initWifi();

    //write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if(x == 200){
      Serial.println("Channel update successful.");
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
    last_sent = current;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing wifi...");
  WiFi.mode(WIFI_STA);
  initWifi();
  Serial.println("Initializing ThingSpeak...");
  ThingSpeak.begin(client);
  Serial.println("Initializing decoder...");
  receiver.begin(RECEIVER_PIN);
  Serial.println("Start receiving packages...");
}

void loop() {
  if (receiver.isAvailable()) {
    THPacket measurement = receiver.getLastReceived();
    processDecodedData(measurement);
  }
}

