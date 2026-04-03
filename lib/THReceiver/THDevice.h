// Include guard. Prevents program errors if the library is accidentally included more than once in the sketch
#ifndef _THDReciever_h 
  #define _THDReciever_h
  #include "THReciever.h"
  #pragma once
  #include <Arduino.h>  
  #include <stdio.h>
  #include <cstdlib>
  #include "Logger.h"

  /**
   * Struct representing the configuration of a device. This is used to easily 
   * load and save device configurations to and from EEPROM. It has the following 
   * attributes:
   * 
   * The `deviceID` is a unique identifier for each device. It is used to match 
   * incoming measurements to the correct device. Normally a number between 0 and 
   * 0xFF. This is selected at random each time a sensor is turned on.
   * 
   * A short name for this device. This is used for status updates and logging. 
   * It should be no longer than 4 characters to save space in the EEPROM.
   * 
   * The `settings` byte is used to store multiple boolean and small integer values 
   * in a compact way. Bit 0 (the least significant bit) is used to indicate whether 
   * the device has humidity measurements (1) or not (0). Bits 1 and 2 are used 
   * to store the channel number (1, 2, or 4). The remaining bits (3-7) are reserved 
   * for future use. This allows us to store both the humidity setting and the 
   * channel number in a single byte, saving space in the EEPROM.  
   * 
   * The `correction` value is a simple calibration feature. The correction value 
   * is added to the received temperature to correct for any systematic errors 
   * in the sensor. This can be used to calibrate the sensor against a known accurate 
   * thermometer. The correction value can be positive or negative and is stored 
   * in steps of 0.1°C.
   * 
   * The `maxValidDelta` is used in the baseline temperature validator. When a 
   * new temperature measurement is received, it is compared to the previous valid 
   * temperature. If the absolute difference between the two temperatures exceeds 
   * the max valid delta, the new measurement is considered invalid and is not 
   * added to the baseline. This helps to filter out outliers and sudden changes 
   * in temperature that are unlikely to be real. The max valid delta is stored 
   * in steps of 0.1°C and can be set to a value between 0 and 255 (which 
   * corresponds to a maximum valid temperature difference of 25.5°C).
   * 
   * The `maxValidInterval` is used to determine if a device has timed out. If the 
   * time since the last valid measurement exceeds this interval, the device is 
   * considered timed out. This is useful to detect when a sensor has stopped 
   * working or is out of range. The max valid interval is stored in minutes and 
   * can be set to a value between 0 and 255.
   */
  struct DeviceConfig {
    /**
     * Unique identifier for each device. 
     */
    uint8_t deviceID;

    /**
     * Location on the display. 
     */
    uint8_t displayID;

    /**
     * A short name for this device.
     */
    char name[5];

    /**
     * The settings byte
     */
    uint8_t settings; // 0=hasHumidity 1-2=channel 3=7=reserved

    /**
     * A simple calibration feature. 
     */
    int8_t correction;

    /**
     * The max valid delta is used in the baseline temperature validator. 
     */
    uint8_t maxValidDelta; // Max valid temp difference * 10C (default: 7)
    
    /**
     * The max valid interval is used to determine if a device has timed out. 
     */
    uint8_t maxValidInterval; // Max valid time interval in minutes (default: 15)

  };

  /**
   * Class representing a temperature and humidity sensor. It stores the latest 
   * measurement and some metadata about the device. It also has some basic 
   * validation and status update features.
   */
  class THDevice {
    public:
      static const bool DISABLE_HUMIDITY = false;
      static const bool MAX_NAME_LENGTH = 5;

      THDevice(DeviceConfig config);

      void setConfig(DeviceConfig config);

      bool operator == (const THDevice &other);

      bool hasID(uint8_t id);
      String printName();

      bool hasHumidity() { return _hasHumidity; }

      bool process(THPacket measurement);
      String getName() { return String(_name); }
      unsigned long getLastAge();
      String getLastTemp();
      String getLastHumidity();
      String getLastBatteryState();
      String getLastStatus();
      void checkTimeout();

      bool hasUpdates();
      THPacket getLastRecieved();

      uint8_t displayID;

    private:
      uint8_t _deviceID;
      uint8_t _channelNo;
      String _name;
      float _correction;
      bool _hasHumidity;
      float _maxValidDelta;
      unsigned long _maxValidInterval;

      THPacket _last;
      unsigned long _prevUpdateTime = 0;
      float _prevUpdateTemp;
      uint8_t _prevUpdateHum = -1;
      uint8_t _prevUpdateChannel = -1;
      bool _hasNewPacket;
      unsigned long _lastBatteryNotification = 0;
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
      
  };
#endif