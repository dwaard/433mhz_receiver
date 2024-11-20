// Include guard. Prevents program errors if the library is accidentally included more than once in the sketch
#ifndef _THDReciever_h 
  #define _THDReciever_h
  #include "THReciever.h"
  #pragma once
  #include <Arduino.h>  
  #include <stdio.h>
  #include <cstdlib>

  class THDevice {
    public:
      THDevice(uint8_t deviceID, uint8_t channelNo, const char *name, float correction);

      bool operator == (const THDevice &other);

      bool hasID(uint8_t id);
      String printName();

      bool process(THPacket measurement);
      void checkTimeout();

      bool hasUpdates();
      THPacket getLastRecieved();
      bool hasStatusupdates();
      String getStatusupdates();
      void resetStatus();

    private:
      uint8_t _deviceID;
      uint8_t _channelNo;
      const char *_name;
      float _correction;

      THPacket _last;
      unsigned long _prevUpdateTime = 0;
      float _prevUpdateTemp;
      bool _hasNewPacket;

      String _status = String("");

      const int UPDATE_TIMEOUT = 5 * 60 * 1000;
      const int BASELINE_TIMEOUT = 10 * 60 * 1000;
      const int BASELINE_SIZE = 3;
      const float BASELINE_TEMP_THRESHOLD = 0.3;
      float *_baselineTemps = new float[BASELINE_SIZE];
      unsigned int _validTempsCount = 0;
      unsigned int _latestTempBaselineIndex = 0;
      unsigned long _lastReceived = 0;

      bool isValid(THPacket packet);

      void addStatus(String msg);

      void addFormattedStatus(const char *format, ...);
      
  };
#endif