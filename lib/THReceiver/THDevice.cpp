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
  _last.timestamp = 0; // Considered as an empty THPacket
  resetStatus();
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

/**
 * Check if the device is timed out, which means that too much time has elapsed
 * since this device has recieved a valid packet.
 */
void THDevice::checkTimeout() {
  if (_lastReceived != 0) {
    unsigned int tdiff = millis() - _lastReceived; // Rollover safe  time diff
    if (tdiff > BASELINE_TIMEOUT) {
      // Reset the baseline on first time or timeout
      char buf[35];
      printName(buf);
      Serial.print(buf);
      Serial.println(" timed out");
      addStatus("timeout. Baseline is reset");
      _validTempsCount = 0;
      _latestTempBaselineIndex = 0;
    }
  }
}

/**
 * Checks if the given packet is valid for this device.
 * 
 * @param packet the packet to validate
 * @return `true` if the packet is valid
 */
bool THDevice::isValid(THPacket packet) {
  if (packet.deviceID != _deviceID) {
    char msg[25];
    sprintf(msg, "ongeldig deviceID: 0x%X", packet.deviceID);
    addStatus(msg);
    return false;
  }
  // Disabled because of 1 sensor that seems to have a broken humidity sensor
  // if (packet.humidity > 100) {
  //   return false;
  // }
  if (packet.temperature < -50 || packet.temperature > 50) {
    char msg[25];
    sprintf(msg, "ongeldige temp.: %1f", packet.temperature);
    addStatus(msg);
    return false;
  }
  // The baseline temperature validator
  if (_validTempsCount > 0) {
    // Compute the diff between the latest and previous temp.
    float prevTemp = _baselineTemps[_latestTempBaselineIndex];
    float diff = abs(prevTemp - packet.temperature);

    if (diff > BASELINE_TEMP_THRESHOLD) {
      char msg[35];
      sprintf(msg, "temp.verandering te groot: %1f", diff);
      addStatus(msg);
      if (_validTempsCount >= BASELINE_SIZE) {
        // return false when the baseline is filled
        return false;
      }
      // when the baseline is still filling, reset the baseline
      _validTempsCount = 0;
    }

    // Finally, increase the latest index before a new temp is added.
    _latestTempBaselineIndex = (_latestTempBaselineIndex + 1) % BASELINE_SIZE ;
  }
  // Now, the recieved temp is either the first or the "same" as the previous
  // Add it to the baseline.
  _baselineTemps[_latestTempBaselineIndex] = packet.temperature;
  _lastReceived = packet.timestamp;
  _validTempsCount++;
  if (_validTempsCount < BASELINE_SIZE) {
    // The baseline is not filled yet
    return false;
  }
  if (_validTempsCount == BASELINE_SIZE) {
    addStatus("baseline klaar");
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
  if (!packet.batteryState) {
    addStatus("batterij laag");
  }
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

void THDevice::addStatus( const char *msg) {
  if (strlen(_status) == 0) {
    printName(_status);
    strlcat(_status, ": ", MAX_STATUS_SIZE);
  } else {
    strlcat(_status, "; ", MAX_STATUS_SIZE);
  }
  strlcat(_status, msg, MAX_STATUS_SIZE);
}

bool THDevice::hasStatusupdates() {
  return strlen(_status) > 0;
}

const char* THDevice::getStatusupdates() {
  return _status;
}

void THDevice::resetStatus() {
  strlcpy(_status, "", MAX_STATUS_SIZE);
}