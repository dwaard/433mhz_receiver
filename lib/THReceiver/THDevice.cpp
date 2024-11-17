#include "THDevice.h"

/**
 * Constructor for this class
 * 
 * @param deviceID the ID of the device. Normally a number between 0 and 0xFF
 * @param channelNo should be 1, 2 or 4
 * @param name human readable name. Basically which temperature and humidity this
 *              device monitors
 * @param correction temperature correction. A simple calibration feature
 */
THDevice::THDevice(uint8_t deviceID, uint8_t channelNo, const char *name, float correction) {
  _deviceID = deviceID;
  _channelNo = channelNo;
  _name = name;
  _correction = correction;
  _hasUpdates = false;
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
 * Prints a human readable name of this device to the specified buffer.
 * 
 * @param buf the buffer to print to 
 */
 void THDevice::printName(char *buf) {
  // building a char array like "_name (0xid)"
  sprintf(buf, "%s (0x%X)", _name, _deviceID);
}

bool THDevice::isValid(THPacket packet) {
  if (packet.deviceID != _deviceID) {
    return false;
  }
  // Disabled because of 1 sensor that seems to have a broken humidity sensor
  // if (packet.humidity > 100) {
  //   return false;
  // }
  if (packet.temperature < -50 || packet.temperature > 50) {
    return false;
  }
  return true;
}

/**
 * Process a new measurement for this device. Returns `true` if and only
 * if this method results into a state change of this device. Otherwise 
 * `false`.
 * 
 * @param measurement the measurement to process
 */
bool THDevice::process(THPacket packet) {
  if (!isValid(packet)) {
    return false;
  }
  _last = packet;
  _last.temperature += _correction;
  _hasUpdates = true;
  return true;
}

/**
 * Returns `true` if this device has something to update to ThingSpeak.
 * Otherwise `false`.
 */
bool THDevice::hasUpdates() {
  return _hasUpdates == true;
}

/**
 * Returns the last received Measurement.
 */
THPacket THDevice::getLastRecieved() {
  _hasUpdates = false;
  return _last;
}