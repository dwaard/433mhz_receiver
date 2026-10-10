// Include guard. Prevents program errors if the library is accidentally included more than once in the sketch
#ifndef _THSensor_h 
  #define _THSensor_h
  #include "THReciever.h"
  #pragma once
  #include <Arduino.h>  
  #include <stdio.h>
  #include <cstdlib>
  #include "Logger.h"
  #include "THSensorConfig.h"

  /**
   * Class representing a temperature and humidity sensor. It stores the latest 
   * measurement and some metadata about the device. It also has some basic 
   * validation and status update features.
   */
  class THSensor {
    public:
      static const bool DISABLE_HUMIDITY = false;
      static const bool MAX_NAME_LENGTH = 5;
      
      static const uint8_t STATE_IDLE = 0;
      static const uint8_t STATE_BASELINE = 1;
      static const uint8_t STATE_ACTIVE = 2;
      static const uint8_t STATE_TIMEOUT = 3;
      static const uint8_t STATE_ERROR = 0xFF;

      THSensor(THSensorConfig *config) {
        this->config = config;
        _hasNewPacket = false;
        _last.timestamp = 0; // Considered as an empty THPacket
        _state = STATE_IDLE;
      }

      // Getters for the device configuration.
      THSensorConfig getConfig() { return *config; }
      uint8_t getState() { return _state; }

      // Methods for the main processing logic of this device.
      bool process(THPacket measurement);
      void checkTimeout();
      bool hasUpdates();

      // Getters for the latest measurement and status of this device.
      unsigned long getLastReceived() {
        if (_state != STATE_ACTIVE) {
          return 0;
        }
        return _last.timestamp;
      }

      float getLastTemp() {
        if (_state != STATE_ACTIVE) {
          return NAN;
        }
        return _last.temperature;
      }

      float getLastHumidity() {
        if (_state != STATE_ACTIVE || !config->hasHumidity()) {
          return NAN;
        }
        return _last.humidity;
      }

      bool getLastBatteryState() {
        if (_state != STATE_ACTIVE) {
          return false;
        }
        return _last.batteryState;
      }

      unsigned long getLastAge() {
        if (_state != STATE_ACTIVE) {
          return -1;
        }
        return (millis() - _last.timestamp);
      }

      unsigned int getBaselineSize() {
        return _validTempsCount;
      }

      /**
       * Compares this device with another device based on their IDs and channels.
       * 
       * @param deviceID the deviceID to compare
       * @param channelNo the channel number to compare
       * @return -1 if this deviceID is not equal to the specified ID
       *          0 if they are equal but channels differ
       *          1 if this deviceID and channel number are both equal to the specified values
       */
      int compareTo(uint8_t deviceID, uint8_t channelNo) {
        if (config->deviceID != deviceID) {
          return -1;
        }
        if (config->getChannel() != channelNo) {
          return 0;
        }
        return 1;
      }

      String printName();

    private:
      // Pointer to the config of this device.
      THSensorConfig *config;  

      THPacket _last;

      // State variables for processing and validating measurements
      uint8_t _state = STATE_IDLE;

      unsigned long _prevUpdateTime = 0;
      float _prevUpdateTemp;
      uint8_t _prevUpdateHum = -1;
      uint8_t _prevUpdateChannel = -1;
      bool _hasNewPacket;
      unsigned long _nextBatteryNotification = 0;

      const unsigned long BATTERY_NOTIFICATION_INTERVAL = 60 * 60 * 1000;
      const unsigned int UPDATE_TIMEOUT = 5 * 60 * 1000;
      const unsigned int BASELINE_TIMEOUT = 15 * 60 * 1000;
      const unsigned int BASELINE_SIZE = 3;
      const float BASELINE_TEMP_THRESHOLD = 1.0;
      float *_baselineTemps = new float[BASELINE_SIZE];
      unsigned int _validTempsCount = 0;
      unsigned int _latestTempBaselineIndex = 0;
      unsigned long _lastReceived = 0;

      bool isValid(THPacket packet);
      bool processBaseline(THPacket packet);
      
  };
#endif