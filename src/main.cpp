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

/* Platform specific stuff */
#if defined(ARDUINO_ARCH_ESP8266)
  #include <ESP8266WiFi.h>
  #pragma message("Building for ESP8266 platform...")
#else
  #include <WiFi.h>
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

void initWifi() {
  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);
    } 
    Serial.println("\nConnected.");
  }
}

void printMeasurement(Measurement m) {
  Serial.print(m.deviceID, HEX);
  Serial.print(";");
  Serial.print(m.batteryState ? "N" : "L");
  Serial.print(";");
  Serial.print(m.channelNo);
  Serial.print(";");
  Serial.print(m.temperature, 1);
  Serial.print(";");
  Serial.println(m.humidity, DEC);
}

void updateThingSpeak(Measurement m) {
  // set the fields with the values
  if (m.deviceID==0x1D) { // Buiten slaapkamer
    ThingSpeak.setField(1, m.temperature);
    if (!m.batteryState) {
      ThingSpeak.setStatus("Battery Buiten slk (1D) low");
    }
  }
  if (m.deviceID==0x45) { // Garage
    ThingSpeak.setField(2, m.temperature);
    if (!m.batteryState) {
      ThingSpeak.setStatus("Battery Garage (45) low");
    }
  }
  if (m.deviceID==0x5C) { // Keuken
    ThingSpeak.setField(3, m.temperature);
    if (!m.batteryState) {
      ThingSpeak.setStatus("Battery Keuken (5C) low");
    }
  }
  if (m.deviceID==0x70) { // Buiten kantoor
    ThingSpeak.setField(4, m.temperature);
    if (!m.batteryState) {
      ThingSpeak.setStatus("Battery Buiten kantoor (70) low");
    }
  }
  if (m.deviceID==0xE5) { // Kelder
    ThingSpeak.setField(5, m.temperature);
    if (!m.batteryState) {
      ThingSpeak.setStatus("Battery Kelder (E5) low");
    }
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
  Serial.println("Initializing wifi.");
  WiFi.mode(WIFI_STA);
  initWifi();
  ThingSpeak.begin(client);
  Serial.println("Initializing decoder.");
  receiver.begin(RECEIVER_PIN);
  Serial.println("Start receiving packages...");
}

void loop() {
  if (receiver.isAvailable()) {
    Measurement m = receiver.getLastMeasurement();

    printMeasurement(m);
    updateThingSpeak(m);
  }

}