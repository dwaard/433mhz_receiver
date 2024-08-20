// Include guard. Prevents program errors if the library is accidentally included more than once in the sketch
#ifndef _THMeasurememt_h 
  #define _THMeasurememt_h

  #if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
  #elif defined(ENERGIA) // LaunchPad, FraunchPad and StellarPad specific
      #include "Energia.h"
  #elif defined(RPI) // Raspberry Pi
      #define RaspberryPi
      // Include libraries for RPi:
      #include <string.h> /* memcpy */
      #include <stdlib.h> /* abs */
      #include <wiringPi.h>
  #elif defined(SPARK)
      #include "application.h"
  #else
      #include <cstdint>
  #endif

  /* The struct for an actual measurement */
  struct Measurement {
    uint8_t deviceID;
    bool batteryState;
    uint8_t channelNo;
    float temperature;
    uint8_t humidity;
  };
#endif