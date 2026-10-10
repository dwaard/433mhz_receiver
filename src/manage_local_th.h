/******************************************************************************
 * @file manage_local_th.h
 * @brief Functions for managing the local TH sensor.
 * 
 * The local sensor is a BMP280 sensor connected via I2C.
 * 
 *****************************************************************************/
#ifndef MANAGE_LOCAL_TH_H
#define MANAGE_LOCAL_TH_H

#include "SparkFunBME280.h"

/******************************************************************************
 *                                                                            *
 * BMP280 sensor functions                                                    *
 *                                                                            *
 *****************************************************************************/
bool bmp280Initialized = false;

BME280 bmp280; //Uses default I2C address 0x76

unsigned long nextLocalMeasurement = 0;

#define SETUP_BMP280() {\
  bmp280.setI2CAddress(BMP280_I2C_ADDRESS);\
  /* The I2C address must be set before .begin() otherwise the cal values will fail to load. */\
  if(bmp280.beginI2C() == false) {\
    Log.error("BMP280 init failed");\
  } else {\
    bmp280Initialized = true;\
    Log.info("BMP280 is ready to use");\
  } \
}

#define LOOP_BMP280() {\
  if ((long)(millis() - nextLocalMeasurement) >= 0) {\
    nextLocalMeasurement = millis() + LOCAL_MEASUREMENT_INTERVAL;\
    if (bmp280Initialized) {\
      THPacket packet = {};\
      packet.deviceID = BMP280_DEVICE_ID;\
      packet.channelNo = BMP280_DEVICE_CHANNEL;\
      packet.batteryState = 1;\
      packet.temperature = static_cast<float>(static_cast<int>(bmp280.readTempC() * 10.)) / 10.;\
      packet.humidity = bmp280.readFloatHumidity();\
      packet.timestamp = millis();\
      processPacket(packet);\
    } else {\
      Log.warning("Skipping local measurement, BMP280 failed to initialize");\
    }\
  }\
}

#endif // MANAGE_LOCAL_TH_H