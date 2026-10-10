#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "THSensor.h"
#include "THSensorConfig.h"

/******************************************************************************
 *                                                                            *
 * Serial logging configuration                                               *
 *                                                                            *
 *****************************************************************************/
#define SERIAL_BAUD_RATE 115200

#define SERIAL_DEBUGGING_ENABLED 0

#if SERIAL_DEBUGGING_ENABLED > 0
  #include "Channels/SerialChannel.h"
  #define SERIAL_DEBUGGING_LEVEL Logger::INFO
#endif

/******************************************************************************
 *                                                                            *
 * Telnet logging configuration                                               *
 *                                                                            *
 *****************************************************************************/
#define TELNET_DEBUGGING_ENABLED 1

#if TELNET_DEBUGGING_ENABLED > 0
  #include "Channels/TelnetChannel.h"
  #define TELNET_SERVER_PORT 23
  #define TELNET_DEBUGGING_LEVEL Logger::INFO
#endif


/******************************************************************************
 *                                                                            *
 * 433Mhz receiver configuration                                              *
 *                                                                            *
 *****************************************************************************/
// Pin for the 433 MHz receiver. 
// This should be an interrupt-capable pin for best performance.
#define THRECEIVER_PIN D5 // GPIO14 on ESP8266, supports interrupts


/******************************************************************************
 *                                                                            *
 * I2C configuration                                                          *
 *                                                                            *
 *****************************************************************************/
#define I2C_SDA_PIN D2 // GPIO4 on ESP8266, default SDA pin
#define I2C_SCL_PIN D1 // GPIO5 on ESP8266, default SCL pin


/******************************************************************************
 *                                                                            *
 * Local sensor configuration                                                 *
 *                                                                            *
 *****************************************************************************/
#define LOCAL_MEASUREMENT_INTERVAL 60000 // 1 minute

// Local measurement (BMP280) configuration
#define BMP280_I2C_ADDRESS 0x76 // DEFAULT I2C address for BMP280 sensor
#define BMP280_DEVICE_ID 0x00   // Selected deviceID for the local BMP280 sensor
#define BMP280_DEVICE_CHANNEL 0 // Selected channel for the local BMP280 sensor


/******************************************************************************
 *                                                                            *
 * Weather station (WS2910) configuration                                     *
 *                                                                            *
 *****************************************************************************/
#define WS2910_IN_DEVICE_ID 0x01    // Selected deviceID for the indoor WS2910 sensor
#define WS2910_IN_DEVICE_CHANNEL 0  // Selected channel for the indoor WS2910 sensor
#define WS2910_OUT_DEVICE_ID 0x02   // Selected deviceID for the outdoor WS2910 sensor
#define WS2910_OUT_DEVICE_CHANNEL 0 // Selected channel for the outdoor WS2910 sensor


/******************************************************************************
 *                                                                            *
 * ThingSpeak configuration                                                   *
 *                                                                            *
 *****************************************************************************/
#define THINGSPEAK_UPDATE_INTERVAL 5 * 60 * 1000    // 5 minutes
#define THINGSPEAK_STATUS_UPDATE_INTERVAL 25 * 1000 // 15 seconds


/******************************************************************************
 *                                                                            *
 * Sensor and global configuration and memory allocation                      *
 *                                                                            *
 *****************************************************************************/
#define SENSOR_COUNT 9
#define CONFIG_VERSION 1

struct Config {
  uint8_t version; // Version control
  uint8_t telnetLogLevel; // 0 = NONE, 5 = TRACE, ... 50 = CRITICAL
  uint8_t thingSpeakLogLevel; // 0 = NONE, 5 = TRACE, ... 50 = CRITICAL
} config;

THSensorConfig sensorConfigs[SENSOR_COUNT]; // Array of sensor configurations

THSensor **sensors = new THSensor*[SENSOR_COUNT];

// Forward declaration of the packet processing function, so it can be used in 
// other modules like manage_local_th.h
void processPacket(THPacket packet);

/******************************************************************************
 *                                                                            *
 * EEPROM configuration                                                       *
 *                                                                            *
 *****************************************************************************/
#define CONFIG_EEPROM_ADDRESS 0
#define CONFIG_EEPROM_SIZE sizeof(Config) + sizeof(THSensorConfig) * SENSOR_COUNT
#define CONFIG_EEPROM_ADDRESS 0
#define CONFIG_SENSORS_ADDRESS CONFIG_EEPROM_ADDRESS + sizeof(Config)

/******************************************************************************
 *                                                                            *
 * Logging functions                                                          *
 *                                                                            *
 *****************************************************************************/
#if SERIAL_DEBUGGING_ENABLED > 0
  #define SETUP_SERIAL_LOGGING() {\
     Log.info("Starting serial logging...");\
     delay(100); /* Wait for system to stabilize and allow time to connect serial monitor after reset */\
     Log.addOwnedChannel(new SerialChannel(SERIAL_DEBUGGING_LEVEL));\
     Log.trace("Serial logging initialized");\
  }
#endif

#if TELNET_DEBUGGING_ENABLED > 0
  WiFiServer telnetServer(TELNET_SERVER_PORT);
  WiFiClient telnetClient;
  TelnetChannel telnetChannel(&telnetClient, TELNET_DEBUGGING_LEVEL);

  /**
   * @brief Checks for incoming telnet connections and accepts them. This allows 
   * for remote logging over telnet. 
   */
  bool handleTelnetConnections() {
    // Check for telnet clients
    if (telnetServer.hasClient()) {
        telnetClient = telnetServer.accept();
        Log.info("Telnet log client accepted");
        return true;
    }
    return false;
  }

  #define SETUP_TELNET_LOGGING() {\
     Log.info("Starting telnet logging, waiting for client to connect...");\
     telnetServer.begin();\
     Log.addChannel(&telnetChannel);\
     long unsigned int timeout = millis() + 20000; /* Wait up to 20 seconds for a telnet client to connect before proceeding without telnet logging */\
     bool connected = false;\
     while (!connected && (millis() < timeout)) {\
       connected = handleTelnetConnections();\
       delay(100);\
     }\
  }
#endif

#endif // CONFIG_H

