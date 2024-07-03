
// Include guard. Prevents program errors if the library is accidentally included more than once in the sketch
#ifndef _THDeviceManager_h 
  #define _THDeviceManager_h
  #include "THReciever.h"

  class THDevice {
    public:
      THDevice(uint8_t deviceID, uint8_t channelNo, String name, float correction);
      bool operator == (const THDevice &other);
      bool hasID(uint8_t id);
      String getName();
      bool process(Measurement measurement);
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


  class THDeviceManager {
    public:
      THDeviceManager(uint8_t maxDevices);
      void add(THDevice device);
      bool process(Measurement m);
      bool hasDevice(uint8_t deviceID);
      bool hasDevice(THDevice device);
      THDevice *getDevice(uint8_t deviceID);
    private:
      THDevice* devices;
      uint8_t maxSize;
      uint8_t size;
  };
#endif
