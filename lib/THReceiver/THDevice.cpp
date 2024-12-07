#include "THDevice.h"

String formatString(const char *format, ...) {
    char buffer[128]; // Adjust size as needed
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    return String(buffer);
}


/**
 * Constructor for this class
 * 
 * Some sensors do not send proper humidity values. To avoid useless statusupdates, 
 * that value can be disabled.
 * 
 * @param deviceID the ID of the device. Normally a number between 0 and 0xFF
 * @param channelNo should be 1, 2 or 4
 * @param name human readable name. Basically which temperature and humidity this
 *              device monitors
 * @param correction temperature correction. A simple calibration feature
 * @param hasHumidity `false` to disable the humidity values (default `true`)
 */
THDevice::THDevice(uint8_t deviceID, uint8_t channelNo, const char *name, 
                    float correction, bool hasHumidity) {
  _deviceID = deviceID;
  _channelNo = channelNo;
  _name = name;
  _correction = correction;
  _hasHumidity = hasHumidity;
  _hasNewPacket = false;
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
 String THDevice::printName() {
  return String(String(_name) + " (0x" + String(_deviceID, HEX) + ")");
}

/**
 * Check if the device is timed out, which means that too much time has elapsed
 * since this device has recieved a valid packet.
 */
void THDevice::checkTimeout() {
  unsigned long now = millis();
  if (_lastReceived != 0) {
    unsigned int tdiff = now - _lastReceived; // Rollover safe  time diff
    if (tdiff > BASELINE_TIMEOUT && _validTempsCount > 0) {
      // Reset the baseline on first time or timeout
      Serial.print(printName());
      Serial.println(" timed out");
      addStatus("timeout");
      if (_validTempsCount > 1) {
        _validTempsCount = 0;
        _latestTempBaselineIndex = 0;
        addStatus("baseline is reset");
      }
      _lastReceived = now;
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
    addFormattedStatus("ongeldig deviceID: 0x%X", packet.deviceID);
    return false;
  }
  if (packet.channelNo > _channelNo) {
    if (packet.channelNo != _prevUpdateChannel) {
      addFormattedStatus("ongeldig channel no.: %i", packet.channelNo);
      _prevUpdateChannel = packet.channelNo;
    }
    // return false;
  }
  if (_hasHumidity && packet.humidity > 100) {
    if (packet.humidity != _prevUpdateHum) {
      addFormattedStatus("ongeldige humidity: %i", packet.humidity);
      _prevUpdateHum = packet.humidity;
    }
    // return false;
  }
  if (packet.temperature < -50 || packet.temperature > 50) {
    addFormattedStatus("ongeldige temp.: %.1f", packet.temperature);
    return false;
  }
  // The baseline temperature validator
  if (_validTempsCount == 0) {
    addStatus("start baseline");
  } else {
    // Compute the diff between the latest and previous temp.
    float prevTemp = _baselineTemps[_latestTempBaselineIndex];
    float diff = abs(prevTemp - packet.temperature);
    // Serial.println("  abs(" + String(prevTemp) + " - " + String(packet.temperature) + ") = " + String(diff));
    if (diff > BASELINE_TEMP_THRESHOLD) {
      addFormattedStatus("temp.verandering te groot: %.1f", diff);
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
    unsigned long now = millis();
    unsigned long diff = now - _lastBatteryNotification;
    if (diff > BATTERY_NOTIFICATION_INTERVAL) {
      addStatus("batterij laag");
      _lastBatteryNotification = now;
    }
  }
  _hasNewPacket = true;
  return true;
}

/**
 * Returns `true` if this device has something to update to ThingSpeak.
 * Otherwise `false`.
 */
bool THDevice::hasUpdates() {
  if (!_hasNewPacket) {
    return false;
  }
  return _prevUpdateTime == 0 
    || (_last.temperature != _prevUpdateTemp) 
    || (millis() - _prevUpdateTime) > UPDATE_TIMEOUT;
}

/**
 * Returns the last received Measurement.
 */
THPacket THDevice::getLastRecieved() {
  _prevUpdateTemp = _last.temperature;
  _prevUpdateTime = _last.timestamp;
  _hasNewPacket = false;
  return _last;
}

void THDevice::addStatus(String msg) {
    if (_status.length() == 0) {
    _status = String(printName() + ": " + msg);
  } else {
    _status.concat("; " + msg);
  }
  Serial.print("  ");
  Serial.println(msg);
}

void THDevice::addFormattedStatus(const char *format, ...) {
    char buffer[128]; // Adjust size as needed
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    addStatus(buffer);
}

bool THDevice::hasStatusupdates() {
  return _status.length() > 0;
}

String THDevice::getStatusupdates() {
  return _status;
}

void THDevice::resetStatus() {
  _status = String("");
}
