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
  // The baseline temperature validator
  unsigned int tdiff = packet.timestamp - _lastReceived; // Rollover safe  time diff
  if (_validTempsCount == 0 || tdiff > BASELINE_TIMEOUT) {
    // Reset the baseline on first time or timeout
    Serial.println("  Reset the baseline on first time or timeout");
    addStatus("baseline reset");
    _validTempsCount = 0;
    _latestTempBaselineIndex = 0;
  } else {
    float prevTemp = _baselineTemps[_latestTempBaselineIndex];
    _latestTempBaselineIndex = (_latestTempBaselineIndex + 1) % BASELINE_SIZE ;
    Serial.print("  Calculating diff: |");
    Serial.print(prevTemp);
    Serial.print(" - ");
    Serial.print(packet.temperature);
    float diff = abs(prevTemp - packet.temperature);
    Serial.print("| = ");
    Serial.print(diff);
    if (diff > BASELINE_TEMP_THRESHOLD) {
      Serial.print(": too high!");
      if (_validTempsCount >= BASELINE_SIZE) {
        // return false when the baseline is filled
        Serial.println(" So, rejecting the packet");
        addStatus("packet reject");
        return false;
      }
      // when the baseline is still filling, reset the baseline
      Serial.print(" So, resetting the baseline");
      _validTempsCount = 0;
    }
    Serial.println();
  }
  // Now, the recieved temp is either the first or the "same" as the previous
  // Add it to the baseline.
  _baselineTemps[_latestTempBaselineIndex] = packet.temperature;
  _lastReceived = packet.timestamp;
  _validTempsCount++;
  Serial.print("  The baseline size is now: ");
  Serial.println(_validTempsCount);
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