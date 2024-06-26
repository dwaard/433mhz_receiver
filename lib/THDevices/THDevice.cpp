#include "THDeviceManager.h"

THDevice::THDevice(uint8_t deviceID, uint8_t channelNo, String name, float correction) {
  _deviceID = deviceID;
  _channelNo = channelNo;
  _name = name;
  _correction = correction;
}

String THDevice::getName() {
  char buffer [3];
  sprintf(buffer, "%x", + _deviceID);
  return _name + " (" + buffer + ")";
}

void THDevice::process(Measurement measurement) {
  _last = measurement;
  _hasUpdates = true;
}

bool THDevice::hasUpdates() {
  return _hasUpdates;
}

Measurement THDevice::getLastMeasurement() {
  _hasUpdates = false;
  return _last;
}
