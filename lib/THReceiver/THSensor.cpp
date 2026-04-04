#include "THSensor.h"

String formatString(const char *format, ...) {
    char buffer[128]; // Adjust size as needed
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    return String(buffer);
}

String formatTemperature(float temperature) {
    return formatString("%5.1f", temperature);
}

String formatHumidity(float humidity) {
    return formatString("%3.0f", humidity);
}

THSensor::THSensor(THSensorConfig config) {
  setConfig(config);
  _hasNewPacket = false;
  _last.timestamp = 0; // Considered as an empty THPacket
}

void THSensor::setConfig(THSensorConfig config) {
  _deviceID = config.deviceID;
  displayID = config.displayID;
  _channelNo = (config.settings >> 1) & 0b11;
  _name = String(config.name);
  _correction = config.correction / 10.0;
  _hasHumidity = (config.settings & 0b1) == 1;
  _maxValidDelta = config.maxValidDelta / 10.0;
  _maxValidInterval = config.maxValidInterval * 60 * 1000; // Convert minutes to milliseconds
}

/**
 * Overloads the `==` operator. Checks if a this deviceID equals the given devices'.
 */
bool THSensor::operator == (const THSensor &other) {
  return _deviceID == other._deviceID;
}

/**
 * Returns `true` if this device has the specified ID. Otherwise it returns `false`
 * 
 * @param id the deviceID to check
 */
bool THSensor::hasID(uint8_t id) {
  return _deviceID == id;
}

/**
 * Prints a human readable name of this device to the specified buffer.
 * 
 * @param buf the buffer to print to 
 */
 String THSensor::printName() {
  return String(String(_name) 
    + " (0x" + (_deviceID < 0x10 ? "0" : "") 
    + String(_deviceID, HEX) + ")");
}

/**
 * Prints a display ready status for this device.
 * 
 * @param buf the buffer to print to 
 */
 String THSensor::getLastStatus() {
  String name = formatString("%4s", _name);
  String batt = " ";
  if (_validTempsCount >= BASELINE_SIZE && !_last.batteryState) {
    batt = "X";
  }
  String value = String("  ___");
  if (_validTempsCount >= BASELINE_SIZE) {
    value = formatString("%5.1f", _last.temperature);
  } else {
    for (unsigned int i = 0; i < _validTempsCount; i++) {
      value.setCharAt(i + 2, '-');
    }
  }
  return String(name + batt + getLastTemp());
}

/**
 * Prints a display ready status for this device.
 * 
 * @param buf the buffer to print to 
 */
 unsigned long THSensor::getLastAge() {
  if (_validTempsCount >= BASELINE_SIZE) {
    return (millis() - _last.timestamp) / 1000;
  }
  return -1;
}

/**
 * Prints a display ready status for this device.
 * 
 * @param buf the buffer to print to 
 */
 String THSensor::getLastBatteryState() {
  if (_validTempsCount >= BASELINE_SIZE) {
    return _last.batteryState ? "OK" : "LO";
  }
  return String("NaN");
}

/**
 * Prints a display ready status for this device.
 * 
 * @param buf the buffer to print to 
 */
 String THSensor::getLastTemp() {
  if (_validTempsCount >= BASELINE_SIZE) {
    return formatTemperature(_last.temperature);
  }
  String value = String("___");
  for (unsigned int i = 0; i < _validTempsCount; i++) {
    value.setCharAt(i + 2, '-');
  }
  return String(value);
}

/**
 * Prints a display ready status for this device.
 * 
 * @param buf the buffer to print to 
 */
 String THSensor::getLastHumidity() {
  String value = String("  ___");
  if (_validTempsCount >= BASELINE_SIZE) {
    value = formatHumidity(_last.humidity);
  } else {
    for (unsigned int i = 0; i < _validTempsCount; i++) {
      value.setCharAt(i + 2, '-');
    }
  }
  return String(value);
}

/**
 * Check if the device is timed out, which means that too much time has elapsed
 * since this device has recieved a valid packet.
 */
void THSensor::checkTimeout() {
  unsigned long now = millis();
  if (_lastReceived != 0) {
    unsigned int tdiff = now - _lastReceived; // Rollover safe  time diff
    if (tdiff > _maxValidInterval && _validTempsCount > 0) {
      // Reset the baseline on first time or timeout
      Log.warning(printName() + "Timed out after " + String(tdiff) + " ms");
      if (_validTempsCount > 1) {
        _validTempsCount = 0;
        _latestTempBaselineIndex = 0;
        Log.info(printName() +  ": baseline reset.");
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
bool THSensor::isValid(THPacket packet) {
  if (packet.deviceID != _deviceID) {
    Log.error(printName() + ": invalid deviceID. Exp:0x" + String(_deviceID, HEX) + ", got: 0x" + String(packet.deviceID, HEX));
    return false;
  }
  if (packet.channelNo > _channelNo) {
    if (packet.channelNo != _prevUpdateChannel) {
      Log.debug(printName() + ": invalid channel no.: " + String(packet.channelNo));
      _prevUpdateChannel = packet.channelNo;
    }
    // return false;
  }
  if (_hasHumidity && packet.humidity > 100) {
    if (packet.humidity != _prevUpdateHum) {
      Log.debug(printName() + ": invalid humidity. Humidity: " + String(packet.humidity) + "%");
      _prevUpdateHum = packet.humidity;
    }
    // return false;
  }
  float tempWithCorrection = packet.temperature + _correction;
  if (tempWithCorrection < -50 || tempWithCorrection > 50) {
    Log.debug(printName() + ": Invalid temperature. Temperature: " + formatTemperature(tempWithCorrection) + "°C");
    return false;
  }
  // The baseline temperature validator
  if (_validTempsCount == 0) {
    Log.debug(printName() + ": No baseline yet. Accepting first temp: " + formatTemperature(tempWithCorrection) + "°C");
  } else {
    // Compute the diff between the latest and previous temp.
    float prevTemp = _baselineTemps[_latestTempBaselineIndex];
    float diff = abs(prevTemp - tempWithCorrection);
    // Serial.println("  abs(" + String(prevTemp) + " - " + String(tempWithCorrection) + ") = " + String(diff));
    if (diff > _maxValidDelta) {
      Log.debug(printName() + ": Temp diff out of range:" + String(diff) + ". Latest:" + formatTemperature(tempWithCorrection) + ", prev:" + formatTemperature(prevTemp));
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
  _baselineTemps[_latestTempBaselineIndex] = tempWithCorrection;
  _lastReceived = packet.timestamp;
  _validTempsCount++;
  if (_validTempsCount < BASELINE_SIZE) {
    // The baseline is not filled yet
    return false;
  }
  if (_validTempsCount == BASELINE_SIZE) {
    Log.info(printName() + ": Baseline ready:" 
      + formatTemperature(_baselineTemps[0]) + "," + formatTemperature(_baselineTemps[1]) + "," + formatTemperature(_baselineTemps[2]));
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
bool THSensor::process(THPacket packet) {
  if (!isValid(packet)) {
    return false;
  }
  _last = packet;
  _last.temperature += _correction;
  if (!packet.batteryState) {
    unsigned long now = millis();
    unsigned long diff = now - _lastBatteryNotification;
    if (diff > BATTERY_NOTIFICATION_INTERVAL) {
      Log.warning(printName() + ": Low battery. Last notification was " + String(diff) + " ms ago.");
      _lastBatteryNotification = now;
    }
  }
  _hasNewPacket = true;
  Log.debug(printName() + ":" +
    + "Ch:" + String(packet.channelNo) 
    + ", Bat:" + (getLastBatteryState() ? "OK" : "LO") 
    + ", T:" + getLastTemp() + "°C"
    + ( _hasHumidity ? ", Rh:" + getLastHumidity() + "%" : "" )
  );
  return true;
}

/**
 * Returns `true` if this device has something to update to ThingSpeak.
 * Otherwise `false`.
 */
bool THSensor::hasUpdates() {
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
THPacket THSensor::getLastRecieved() {
  _prevUpdateTemp = _last.temperature;
  _prevUpdateTime = _last.timestamp;
  _hasNewPacket = false;
  return _last;
}
