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

/**
 * Check if the device is timed out, which means that too much time has elapsed
 * since this device has recieved a valid packet.
 */
void THSensor::checkTimeout() {
  unsigned long now = millis();
  if ((_state ==  STATE_ACTIVE || _state == STATE_BASELINE) && _lastReceived != 0) {
    unsigned int tdiff = now - _lastReceived; // Rollover safe  time diff
    if (tdiff > config->getMaxValidIntervalRaw() && _validTempsCount > 0) {
      // Reset the baseline on first time or timeout
      Log.warning(config->getLogName() + "Timed out after " + formatString("%3.0f", tdiff / 1000.0 / 60.0) + " minutes"
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
  if (packet.deviceID != config->deviceID) {
    Log.error(config->getLogName() + ": invalid deviceID. Exp:0x" + String(config->deviceID, HEX) + ", got: 0x" + String(packet.deviceID, HEX));
    return false; // Reject the packet. This must be a programming error!
  }
  if (packet.channelNo > config->getChannel()) {
    if (packet.channelNo != config->getChannel()) {
      Log.warning(config->getLogName() + ": invalid channel no.: " + String(packet.channelNo));
    }
    // return false; // Reject the packet. It might contain invalid data
  }
  if (config->hasHumidity() && packet.humidity > 100) {
    if (packet.humidity != _prevUpdateHum) {
      Log.warning(config->getLogName() + ": invalid humidity. Humidity: " + String(packet.humidity) + "%");
      _prevUpdateHum = packet.humidity;
    }
    // return false; // Reject the packet. It might contain invalid data
  }
  // float tempWithCorrection = packet.temperature + getCorrection();
  if (packet.temperature < -50 || packet.temperature > 50) {
    Log.warning(config->getLogName() + ": Invalid temperature. Temperature: " + formatTemperature(packet.temperature) + "°C");
    return false; // Reject the packet. It contains invalid data
  }

  if (_state == STATE_BASELINE || _state == STATE_ACTIVE) {
    // In this state, we have a _last packet and can validate temp diff
    float tempDiff = packet.temperature - _last.temperature;
    float timeDiffMs = (packet.timestamp - _last.timestamp);
    float diffPerMinute = abs(60 * 1000 * tempDiff / timeDiffMs);
    Log.debug("Calculated diff per minute (in °C/min): " + String(diffPerMinute));
    if (diffPerMinute > config->getMaxValidDelta()) {
      if (_state == STATE_BASELINE) {
        Log.warning(config->getLogName() + ": Temp diff out of range for baseline. Temp: " + formatTemperature(packet.temperature) + "°C, prev:" + formatTemperature(_last.temperature) + "°C. Resetting the baseline.");
        _state = STATE_IDLE; // Reset to idle, so that the next packet will be processed as if there is no baseline yet. This allows to recover from wrong measurements during baseline phase.
      } else {
        Log.warning(config->getLogName() + ": Temp diff out of range. Temp: " + formatTemperature(packet.temperature) + "°C, prev:" + formatTemperature(_last.temperature) + "°C");
      }
      return false; // Reject the packet. It contains invalid data
    }
    // TODO validate humidity diff. We need an extra config parameter for this, which is currently not available.
  }
  return true; // Accept the packet
}

bool THSensor::processBaseline(THPacket packet) {
  // float tempWithCorrection = packet.temperature + getCorrection();
  // First, if we are not active yet, we will fill the baseline with the latest temps.
  if (_state != STATE_ACTIVE) {
    if (_validTempsCount == 0) {
      Log.info(config->getLogName() + ": No baseline yet. Accepting first temp: " + formatTemperature(packet.temperature) + "°C");
      _state = STATE_BASELINE;
    } else if (_validTempsCount < BASELINE_SIZE) {
      Log.info(config->getLogName() + ": Baseline not filled yet. Accepting temp #" + String(_validTempsCount + 1) + ": " + formatTemperature(packet.temperature) + "°C");
    }
  }
  // Then, increase the latest index before a new temp is added.
  _latestTempBaselineIndex = (_latestTempBaselineIndex + 1) % BASELINE_SIZE ;
  // Add it to the baseline.
  _baselineTemps[_latestTempBaselineIndex] = packet.temperature;
  // Lastereceived, for the timeout logic
  _lastReceived = packet.timestamp;
  _validTempsCount++;
  if (_validTempsCount < BASELINE_SIZE) {
    // The baseline is not filled yet
    return false;
  }
  if (_validTempsCount == BASELINE_SIZE) {
    Log.debug(config->getLogName() + ": Baseline ready:" 
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
  packet.temperature += config->getCorrection();
  if (!isValid(packet)) {
    return false;
  }
  _last = packet;
  processBaseline(packet);
  if (_state == STATE_ACTIVE) {
    if (!packet.batteryState) {
      unsigned long now = millis();
      if (now >= _nextBatteryNotification) {
        Log.warning(config->getLogName() + ": Low battery.");
        _nextBatteryNotification = now + BATTERY_NOTIFICATION_INTERVAL;
      }
    } else {
      _nextBatteryNotification = 0;
    }
    _hasNewPacket = true;
    Log.info(config->getLogName() + 
      + ": Bat:" + (getLastBatteryState() ? "OK" : "LO") 
      + ", T:" + formatTemperature(getLastTemp()) + "°C"
      + ( config->hasHumidity() ? ", Rh:" + formatHumidity(getLastHumidity()) + "%" : "" )
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
    || (long)(millis() - _prevUpdateTime) > UPDATE_TIMEOUT;
}
