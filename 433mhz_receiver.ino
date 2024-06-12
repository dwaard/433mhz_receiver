/******************************************************************************

433.92 MHz Weather station receiver for Nexus-TH compatible units
More information at:
https://www.onetransistor.eu/2024/01/receive-lpd433-weather-unit-nexus.html

This sample code requires ESP8266 or AVR Arduino with 433.92 MHz receiver.
Optional for ESP8266: 0.96" I2C OLED and MQTT functionality

******************************************************************************/

#include "secrets.h"

/* Platform specific stuff */
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#pragma "Building for ESP8266 platform..."
#define INTERRUPT_ROUTINE void ICACHE_RAM_ATTR
#define EXAMPLE_PIN D7

#elif defined(ARDUINO_ARCH_AVR)
#include <WiFi.h>
#pragma "Building for Arduino AVR platform..."
#define INTERRUPT_ROUTINE void
#define EXAMPLE_PIN 3

#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#pragma "Building for ESP32 platform..."
#define INTERRUPT_ROUTINE void IRAM_ATTR
#define EXAMPLE_PIN 4
#endif

/* Signal and bit stream characteristics */
#define BITS_PER_PACKET 36
#define PACKETS_PER_STREAM 10

#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros

const char* ssid = SECRET_SSID;   // your network SSID (name) 
const char* pass = SECRET_PASS;   // your network password

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

WiFiClient  client;

const int decPin = EXAMPLE_PIN;                    /* signal input pin; must be interrupt capable */
volatile unsigned long lastInt = 0;                /* the most recent interrupt time */
volatile unsigned long currInt;                    /* current interrupt time */
unsigned long currMicros;                          /* current time for main loop */
volatile unsigned long timeInt1 = 0, timeInt2 = 0; /* These variables store the last two pulse timings */
volatile bool keepWaiting = true;                  /* this is reset when the signal is idle and we can start processing sampled data */
//unsigned long lastDec = 0;                         /* last time data got decoded */
volatile unsigned long lastSync = 0; /* last time a sync frame got received */

bool rawBits[PACKETS_PER_STREAM][BITS_PER_PACKET + 1]; /* two dimension array for raw bitstream; number of rows and columns is set for expected signal */
                                                       /* there is one extra bit at the end which will be used as complete flag (i.e. if the row has been filled by exactly 36 bits) */
uint8_t i = 0, j = 0;                                  /* i is the column, j is the row */

uint8_t deviceID;
bool batteryState;
uint8_t channelNo;
float temperature = 0;
uint8_t humidity = 0;

unsigned long last_sent = 0;

INTERRUPT_ROUTINE mapPulseTimingsToBits() {
  currInt = micros();

  // Copy the second pulse timing to the first variable
  timeInt1 = timeInt2;

  // Update the second pulse timing with the last measured pulse
  timeInt2 = currInt - lastInt;

  // Update the most recent interrupt time
  lastInt = currInt;

  // if the first pulse timing fits the ON state
  if (timeInt1 < timeInt2 && timeInt1 > 400 && timeInt1 < 600) {
    // continue sampling the signal
    keepWaiting = true;

    // check if the pulse that followed matches bit 0
    if (timeInt2 > 900 && timeInt2 < 1100) {
      rawBits[j][i] = 0;

      if (i < BITS_PER_PACKET)
        i += 1;
      else rawBits[j][BITS_PER_PACKET] = 0;  // more than expected bits received; set error flag

      // DEBUG: Serial.print("0");
    }

    // otherwise if it maches bit 1
    else if (timeInt2 > 1900 && timeInt2 < 2100) {
      rawBits[j][i] = 1;

      if (i < BITS_PER_PACKET)
        i += 1;
      else rawBits[j][BITS_PER_PACKET] = 0;  // more than expected bits received; set error flag

      // DEBUG: Serial.print("1");
    }

    // if it is longer than that is probably a sync pulse
    else if (timeInt2 < 4500) {

      // if less than 36 bits received, set the flag to false
      if (i < (BITS_PER_PACKET - 1)) {
        rawBits[j][BITS_PER_PACKET] = 0;
      } else {
        rawBits[j][BITS_PER_PACKET] = 1;
      }

      i = 0;  // reset column

      // start filling the next row; ignore extra streams
      if (j < (PACKETS_PER_STREAM - 1))
        j += 1;

      lastSync = currInt;

      // DEBUG: Serial.println();
    }
  }
}

void processRawBitstream() {
  bool finalBits[BITS_PER_PACKET] = { 0 };
  int8_t bitLowCnt = 0, bitHighCnt = 0;

  for (int8_t b = 0; b < BITS_PER_PACKET; b++) {
    for (int8_t a = 0; a < PACKETS_PER_STREAM; a++) {
      // only if packet is complete
      if (rawBits[a][BITS_PER_PACKET] == 1) {

        // take each bit in the same position over all packets
        // in theory all should be identical
        // assuming poor signal strength, go with the majority
        // by counting packets with each of the two bit states
        if (rawBits[a][b] == 1) bitHighCnt += 1;
        else bitLowCnt += 1;

        if (bitHighCnt >= bitLowCnt) finalBits[b] = 1;
        else finalBits[b] = 0;

        bitLowCnt = 0;
        bitHighCnt = 0;
      }
    }
  }

  // clear raw bit stream
  for (int8_t i = 0; i < PACKETS_PER_STREAM; i++)
    for (int8_t j = 0; j < BITS_PER_PACKET; j++)
      rawBits[i][j] = 0;

  /*Serial.print("\nProcessed bitstream: ");
  for (int8_t i = 0; i < BITS_PER_PACKET; i++) {
    Serial.print(finalBits[i], BIN);
  }
  Serial.println();*/

  // another sanity check
  if (finalBits[24] && finalBits[25] && finalBits[26] && finalBits[27]) {

    // get device ID
    deviceID = 0;
    for (int8_t i = 0; i < 8; i++) {
      if (finalBits[i] == 1) deviceID |= 1 << (7 - i);
    }

    // get battery state
    batteryState = finalBits[8];

    // get channel number
    channelNo = (int8_t(finalBits[10]) << 1) | int8_t(finalBits[11]) + 1;

    // get temperature
    int16_t t12 = 0;
    for (int8_t i = 0; i < 12; i++) {
      if (finalBits[i + 12] == 1) t12 |= 1 << (11 - i);
    }
    if (t12 > 2048) t12 = t12 - 4096;
    temperature = t12 / 10.0;
    //temperature = (t12 >> 4) * 0.1f;

    // get humidity
    humidity = 0;
    for (int8_t i = 0; i < 8; i++) {
      if (finalBits[i + 28] == 1) humidity |= 1 << (7 - i);
    }

    // valid data received
    if (deviceID != 0) {
      // process received data
      processDecodedData();
    }
  }
}

void processDecodedData() {
  Serial.print(deviceID, HEX);
  Serial.print(";");
  Serial.print(batteryState ? "N" : "L");
  Serial.print(";");
  Serial.print(channelNo);
  Serial.print(";");
  Serial.print(temperature, 2);
  Serial.print(";");
  Serial.println(humidity, DEC);

  // set the fields with the values
  if (deviceID==0x1D) { // Buiten slaapkamer
    ThingSpeak.setField(1, temperature);
    if (!batteryState) {
      ThingSpeak.setStatus("Battery Buiten slk (1D) low");
    }
  }
  if (deviceID==0x45) { // Garage
    ThingSpeak.setField(2, temperature);
    if (!batteryState) {
      ThingSpeak.setStatus("Battery Garage (45) low");
    }
  }
  if (deviceID==0x5C) { // Keuken
    ThingSpeak.setField(3, temperature);
    if (!batteryState) {
      ThingSpeak.setStatus("Battery Keuken (5C) low");
    }
  }
  if (deviceID==0x70) { // Buiten kantoor
    ThingSpeak.setField(4, temperature);
    if (!batteryState) {
      ThingSpeak.setStatus("Battery Buiten kantoor (70) low");
    }
  }
  if (deviceID==0xE5) { // Kelder
    ThingSpeak.setField(5, temperature);
    if (!batteryState) {
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

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing wifi...");
  WiFi.mode(WIFI_STA);
  initWifi();
  ThingSpeak.begin(client);
  Serial.println("Initializing decoder...");
  pinMode(decPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(decPin), mapPulseTimingsToBits, CHANGE);
}

void loop() {
  currMicros = micros();

  // if no sync frame received within 100 ms (assuming a 36-bit packet of "1" will be sent in 95 ms)
  if ((currMicros > lastSync + 100000) && (keepWaiting == true)) {
    // disable interrrupt for a while
    detachInterrupt(digitalPinToInterrupt(decPin));

    yield();

    // disable sampling pulses and process existing data
    keepWaiting = false;

    // clear last timeframes
    timeInt1 = 0;
    timeInt2 = 0;

    // if at least one packet received
    if (j > 0) processRawBitstream();

    j = 0;
    i = 0;  // reset indexes

    // reattach interrupt
    attachInterrupt(digitalPinToInterrupt(decPin), mapPulseTimingsToBits, CHANGE);
  }
}