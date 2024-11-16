// Include guard. Prevents program errors if the library is accidentally included more than once in the sketch
#ifndef _THDReciever_h 
  #define _THDReciever_h
  #include "THReciever.h"
  #include <stdio.h>
  #include <cstdlib>

  class THDevice {
    public:
      THDevice(uint8_t deviceID, uint8_t channelNo, const char *name, float correction);

      bool operator == (const THDevice &other);
      
      bool hasID(uint8_t id);
      
      void printName(char *buf);
      
      bool process(THPacket measurement);
      
      bool hasUpdates();
      
      THPacket getLastMeasurement();

    private:
      uint8_t _deviceID;
      uint8_t _channelNo;
      const char *_name;
      bool _batteryState;
      float _correction;
      bool _hasUpdates;

      THPacket _last;
  };
#endif