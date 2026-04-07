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

// void THSensor::setConfig(THSensorConfig config) {
//   _deviceID = config.deviceID;
//   displayID = config.displayID;
//   _channelNo = (config.settings >> 1) & 0b11;
//   _name = String(config.name);
//   _correction = config.correction / 10.0;
//   _hasHumidity = (config.settings & 0b1) == 1;
//   _maxValidDelta = config.maxValidDelta / 10.0;
//   _maxValidInterval = config.maxValidInterval * 60 * 1000; // Convert minutes to milliseconds
// }



/**
 * Prints a human readable name of this device to the specified buffer.
 * 
 * @param buf the buffer to print to 
 */
 String THSensor::printName() {
  return String(String(getName()) 
    + " (0x" + (getDeviceID() < 0x10 ? "0" : "") 
    + String(getDeviceID(), HEX) + ")");
}

/**
 * Check if the device is timed out, which means that too much time has elapsed
 * since this device has recieved a valid packet.
 */
void THSensor::checkTimeout() {
  unsigned long now = millis();
  if (_lastReceived != 0) {
    unsigned int tdiff = now - _lastReceived; // Rollover safe  time diff
    if (tdiff > getMaxValidInterval() && _validTempsCount > 0) {
      // Reset the baseline on first time or timeout
      Log.warning(printName() + "Timed out after " + String(tdiff) + " ms"
    + (_validTempsCount > 1 ? ". Baseline reset." : ""));
      if (_validTempsCount > 1) {
        _validTempsCount = 0;
        _latestTempBaselineIndex = 0;
      }
      _state = STATE_TIMEOUT;
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
  if (packet.deviceID != getDeviceID()) {
    Log.error(printName() + ": invalid deviceID. Exp:0x" + String(getDeviceID(), HEX) + ", got: 0x" + String(packet.deviceID, HEX));
    return false; // Reject the packet. This must be a programming error!
  }
  if (packet.channelNo > getChannelNo()) {
    if (packet.channelNo != getChannelNo()) {
      Log.debug(printName() + ": invalid channel no.: " + String(packet.channelNo));
    }
    // return false; // Reject the packet. It might contain invalid data
  }
  if (hasHumidity() && packet.humidity > 100) {
    if (packet.humidity != _prevUpdateHum) {
      Log.debug(printName() + ": invalid humidity. Humidity: " + String(packet.humidity) + "%");
      _prevUpdateHum = packet.humidity;
    }
    // return false; // Reject the packet. It might contain invalid data
  }
  float tempWithCorrection = packet.temperature + getCorrection();
  if (tempWithCorrection < -50 || tempWithCorrection > 50) {
    Log.debug(printName() + ": Invalid temperature. Temperature: " + formatTemperature(tempWithCorrection) + "°C");
    return false; // Reject the packet. It contains invalid data
  }

  if (_state == STATE_BASELINE || _state == STATE_ACTIVE) {
    // In this state, we have a _last packet and can validate temp diff
    float prevTemp = _last.temperature;
    float diff = abs(prevTemp - tempWithCorrection);
    if (diff > getMaxValidDelta()) {
      Log.debug(printName() + ": Temp diff out of range:" + String(diff) + ". Latest:" + formatTemperature(tempWithCorrection) + ", prev:" + formatTemperature(prevTemp));
      return false; // Reject the packet. It contains invalid data
    }
    // TODO validate humidity diff. We need an extra config parameter for this, which is currently not available.
  }
  return true; // Accept the packet
}

bool THSensor::processBaseline(THPacket packet) {
  float tempWithCorrection = packet.temperature + getCorrection();
  // First, if we are not active yet, we will fill the baseline with the latest temps.
  if (_state != STATE_ACTIVE) {
    if (_validTempsCount == 0) {
      Log.debug(printName() + ": No baseline yet. Accepting first temp: " + formatTemperature(tempWithCorrection) + "°C");
      _state = STATE_BASELINE;
    } else if (_validTempsCount < BASELINE_SIZE) {
      Log.debug(printName() + ": Baseline not filled yet. Accepting temp #" + String(_validTempsCount + 1) + ": " + formatTemperature(tempWithCorrection) + "°C");
    }
  }
  // Then, increase the latest index before a new temp is added.
  _latestTempBaselineIndex = (_latestTempBaselineIndex + 1) % BASELINE_SIZE ;
  // Add it to the baseline.
  _baselineTemps[_latestTempBaselineIndex] = tempWithCorrection;
  // Lastereceived, for the timeout logic
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
  _state = STATE_ACTIVE;
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
  processBaseline(packet);
  if (_state == STATE_ACTIVE) {
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
      + ", T:" + formatTemperature(getLastTemp()) + "°C"
      + ( hasHumidity() ? ", Rh:" + formatHumidity(getLastHumidity()) + "%" : "" )
    );
  }
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
