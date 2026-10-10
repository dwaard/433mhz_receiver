/**************************************************************************
* @file manage_eeprom.h
* @brief Functions for managing configuration data in EEPROM.
*
* This header defines functions to load and save both global configuration 
* and individual sensor configurations to EEPROM. It also includes an initialization
* function that sets default values when the stored configuration is missing or
* outdated.
*
* The global configuration includes settings such as log levels, while each
* sensor configuration entry contains details like device ID, display ID, code,
* name, and measurement settings.
*
**************************************************************************/
#ifndef MANAGE_EEPROM_H
#define MANAGE_EEPROM_H

#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
#include <ESP_EEPROM.h>
#define BEGIN_EEPROM() \
  Log.trace("Using ESP_EEPROM"); \
  EEPROM.begin(CONFIG_EEPROM_SIZE); \
  if (EEPROM.percentUsed() >= 0) { \
    Log.trace("EEPROM data is valid!"); \
  } else { \
    initConfig(config); \
  };
#elif defined(ARDUINO_ARCH_AVR)
  #include <EEPROM.h>
  #define BEGIN_EEPROM() \
    Log.trace("Using standard EEPROM"); \
    EEPROM.begin(CONFIG_EEPROM_SIZE);
#endif

#include "config.h"

/**
 * @brief Saves the global configuration to EEPROM.
 *
 * @param config Reference to the configuration object to persist.
 */
void saveConfig(Config &config) {
  Log.trace("Saving configuration...");
  EEPROM.put(CONFIG_EEPROM_ADDRESS, config);
  EEPROM.commit();
}

/**
 * @brief Saves one sensor configuration entry to EEPROM.
 *
 * @param index Zero-based sensor index.
 * @param sensorConfig Reference to the sensor configuration to persist.
 */
void saveSensorConfig(int index, THSensorConfig &sensorConfig) {
    if (index < 0 || index >= SENSOR_COUNT) {
        Log.error("Invalid sensor index: " + String(index));
        return;
    }
    EEPROM.put(CONFIG_SENSORS_ADDRESS + index * sizeof(THSensorConfig), sensorConfig);
    EEPROM.commit();
}   

/**
 * @brief Initializes configuration values in EEPROM.
 *
 * This function writes default sensor and global configuration values when the
 * stored configuration is missing or outdated.
 *
 * @param config Working copy of the configuration to initialize and persist.
 */
void initConfig(Config config) {
    if (CONFIG_VERSION == 0) {
        Log.trace("Initializing configuration");
    } else {
        Log.trace("Upgradinging configuration to version " + String(CONFIG_VERSION));
    }
    THSensorConfig defaultSensorConfig; // Default invalid config
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        saveSensorConfig(i, defaultSensorConfig); // Default invalid config
    }
    config.version = CONFIG_VERSION;
    config.telnetLogLevel = Logger::INFO;
    config.thingSpeakLogLevel = Logger::WARNING;
    saveConfig(config);
}

/**
 * @brief Loads global configuration from EEPROM.
 *
 * If the stored version differs from CONFIG_VERSION, defaults are written by
 * calling initConfig().
 *
 * @return Config The loaded (or reinitialized) global configuration.
 */
Config loadConfig() {
    Config config;
    EEPROM.get(CONFIG_EEPROM_ADDRESS, config);
    if (config.version != CONFIG_VERSION) {
        initConfig(config);
    }
    return config;
}

/**
 * @brief Loads one sensor configuration entry from EEPROM.
 *
 * @param index Zero-based sensor index.
 * @return THSensorConfig Loaded sensor config, or a default config if index is invalid.
 */
THSensorConfig loadSensorConfig(int index) {
    if (index < 0 || index >= SENSOR_COUNT) {
        Log.error("Invalid sensor index: " + String(index));
        return THSensorConfig(); // Return default config
    }
    THSensorConfig sensorConfig;
    EEPROM.get(CONFIG_SENSORS_ADDRESS + index * sizeof(THSensorConfig), sensorConfig);
    return sensorConfig;
}

/**
 * @brief Loads all sensor configurations from EEPROM into the provided array.
 *
 * @param sensorConfigs Pointer to the array where sensor configurations will be 
 *  stored.
 */
void loadSensorConfigs(THSensorConfig *sensorConfigs) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        sensorConfigs[i] = loadSensorConfig(i);
    }
}

#endif // MANAGE_EEPROM_H