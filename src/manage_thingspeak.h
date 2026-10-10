/**
 * @file manage_thingspeak.h
 * @brief Functions for managing ThingSpeak interactions.
 * 
 * include this header in main.cpp to initialize ThingSpeak and handle periodic updates
 * Make sure it is included after other headers and custom macros, especially ThingSpeak.h
 */
#ifndef MANAGE_THINGSPEAK_H
#define MANAGE_THINGSPEAK_H

#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include "Channels/ThingSpeakLogChannel.h"

#include "config.h"
#include "secrets.h"
#include "net.h"

#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros

/******************************************************************************
 *                                                                            *
 * ThingSpeak functions                                                       *
 *                                                                            *
 *****************************************************************************/

WiFiClient thingSpeakClient;

ThingSpeakLogChannel thingSpeakLogChannel(Logger::WARNING);

unsigned long nextThingSpeakUpdate = 0;
unsigned long nextAllowedThingSpeakRequestAt = 0;

int indoorFields[] = {7, 5, 2, 3, 4}; // Map sensor indices for indoor sensor
int outdoorFields[] = {1, 8, 6, 0}; // Map sensor indices for outdoor sensor

unsigned long lastIndoorUpdates[]  = {0, 0, 0, 0, 0, 0, 0, 0}; // Track last update times for indoor sensor fields
unsigned long lastOutdoorUpdates[] = {0, 0, 0, 0, 0, 0, 0, 0}; // Track last update times for outdoor sensor fields

#define BEGIN_THINGSPEAK() \
    Log.info("Starting ThingSpeak service"); \
    ThingSpeak.begin(thingSpeakClient); \
    Log.addChannel(&thingSpeakLogChannel); \
    thingSpeakLogChannel.setLogLevel(Logger::INFO); \
    Log.info("ThingSpeak service is started."); \
    thingSpeakLogChannel.setLogLevel(Logger::WARNING);

/**
 * @brief Updates the ThingSpeak channel with the latest measurements from the 
 * devices and any pending status messages from the ThingSpeakLogChannel.
 */
void handleThingSpeak() {
  bool indoorHasUpdates = false;
  bool outdoorHasUpdates = false;
  bool statusHasUpdates = false;
  if ((long)(millis() - nextAllowedThingSpeakRequestAt) >= 0) { // Wrap-around safe check for next allowed request time
    // Check if there are pending status messages to send to ThingSpeak and if 
    // we're allowed to send a request at this time (to avoid hitting rate limits)
    if (thingSpeakLogChannel.hasStatusToSend()) {
      String* statusPtr = thingSpeakLogChannel.popStatusForSending();
      if (statusPtr != nullptr) {
        ThingSpeak.setStatus(*statusPtr);
        delete statusPtr;
        statusHasUpdates = true;
      }
    }
    if ((long)(millis() - nextThingSpeakUpdate) >= 0) {
      for (int n = 0; n < 5; n++) {
        int i = indoorFields[n];
        if (i < SENSOR_COUNT && sensors[i]->getState() == THSensor::STATE_ACTIVE 
          && sensors[i]->getLastReceived() > lastIndoorUpdates[i] ) {
          ThingSpeak.setField(n + 1, sensors[i]->getLastTemp());
          lastIndoorUpdates[i] = sensors[i]->getLastReceived();
          indoorHasUpdates = true;
        }
      }
    }
    if (indoorHasUpdates || statusHasUpdates) {
      Log.trace("Updating ThingSpeak indoor and/or status channel...");
      // reconnect if needed
      connectWifi(SECRET_SSID, SECRET_PASS);
      //write to the ThingSpeak channel
      int x = ThingSpeak.writeFields(TS_INDOOR_CH_ID, TS_INDOOR_WRITE_APIKEY);
      if(x == 200) {
        Log.info("ThingSpeak indoor and/or status channel update successful.");
      }
      else {
        Log.error(String("ThingSpeak indoor error " + String(x)));
      }
    }
    if ((long)(millis() - nextThingSpeakUpdate) >= 0) {
      nextThingSpeakUpdate = millis() + THINGSPEAK_UPDATE_INTERVAL;
      for (int n = 0; n < 4; n++) {
        int i = outdoorFields[n];
        if (i < SENSOR_COUNT && sensors[i]->getState() == THSensor::STATE_ACTIVE 
          && sensors[i]->getLastReceived() > lastOutdoorUpdates[i]) {
          ThingSpeak.setField(n + 1, sensors[i]->getLastTemp());
          lastOutdoorUpdates[i] = sensors[i]->getLastReceived();
          outdoorHasUpdates = true;
        }
      }
    }
    if (outdoorHasUpdates) {
      Log.trace("Updating ThingSpeak outdoor channel...");
      // reconnect if needed
      connectWifi(SECRET_SSID, SECRET_PASS);
      //write to the ThingSpeak channel
      int x = ThingSpeak.writeFields(TS_OUTDOOR_CH_ID, TS_OUTDOOR_WRITE_APIKEY);
      if(x == 200) {
        Log.info("ThingSpeak outdoor channel update successful.");
      }
      else {
        Log.error(String("ThingSpeak outdoor error " + String(x)));
      }
    }
  }
  if (indoorHasUpdates || outdoorHasUpdates || statusHasUpdates) {
    nextAllowedThingSpeakRequestAt = millis() + THINGSPEAK_STATUS_UPDATE_INTERVAL; // Set a short delay before the next allowed request to avoid hitting rate limits
  }
}


#endif // MANAGE_THINGSPEAK_H
