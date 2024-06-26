
// Include guard. Prevents program errors if the library is accidentally included more than once in the sketch
#ifndef _THDeviceManager_h 
  #define _THDeviceManager_h
  #include "THReciever.h"

  class THDeviceManager {
    public:
      void add(THDevice *device);
      void process(Measurement m);
      bool deviceExists(uint8_t deviceID);
      THDevice *getDevice(uint8_t deviceID);
    private:
    
  };

  class THDevice {
    public:
      THDevice(uint8_t deviceID, uint8_t channelNo, String name, float correction);
      String getName();
      void process(Measurement measurement);
      bool hasUpdates();
      Measurement getLastMeasurement();

    private:
      uint8_t _deviceID;
      uint8_t _channelNo;
      String _name;
      bool _batteryState;
      float _correction;
      bool _hasUpdates;

      Measurement _last;
  };

#endif
