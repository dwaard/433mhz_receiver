#include "THDeviceManager.h"

/**
 * Constructor for this class
 * 
 * @param deviceID the ID of the device. Normally a number between 0 and 0xFF
 * @param channelNo should be 1, 2 or 4
 * @param name human readable name. Basically which temperature and humidity this
 *              device monitors
 * @param correction temperature correction. A simple calibration feature
 */
THDevice::THDevice(uint8_t deviceID, uint8_t channelNo, String name, float correction) {
  _deviceID = deviceID;
  _channelNo = channelNo;
  _name = name;
  _correction = correction;
}

/**
 * Overloads the `==` operator. Checks if a this deviceID equals the given devices'.
 */
bool THDevice::operator == (const THDevice &other) {
  return _deviceID == other._deviceID;
}

/**
 * Returns `true` if this device has the specified ID. Otherwise it returns `false`
 * 
 * @param id the deviceID to check
 */
bool THDevice::hasID(uint8_t id) {
  return _deviceID == id;
}

/**
 * Returns a human readable name of this device. 
 */
String THDevice::getName() {
  char buffer [3];
  sprintf(buffer, "%x", + _deviceID);
  return _name + " (" + buffer + ")";
}

/**
 * Process a new measurement for this device. Returns `true` if and only
 * if this method results into a state change of this device. Otherwise 
 * `false`.
 * 
 * @param measurement the measurement to process
 */
bool THDevice::process(Measurement measurement) {
  if (measurement.deviceID != _deviceID) {
    return false;
  }
  _last = measurement;
  _hasUpdates = true;
  return true;
}

/**
 * Returns `true` if this device has something to update to ThingSpeak.
 * Otherwise `false`.
 */
bool THDevice::hasUpdates() {
  return _hasUpdates;
}

/**
 * Returns the last received Measurement.
 */
Measurement THDevice::getLastMeasurement() {
  _hasUpdates = false;
  return _last;
}
