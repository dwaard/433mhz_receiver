// TODO add proper comments here
// TODO add serious header stuff here

// Include guard. Prevents program errors if the library is accidentally included more than once in the sketch
#ifndef _THReceiver_h 
  #define _THReceiver_h

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
      // #include "WProgram.h"
      #include <cstdint>
  #endif

  /* Signal and bit stream characteristics */
  #define BITS_PER_PACKET 36
  #define PACKETS_PER_STREAM 10

  #include "THMeasurement.h"

  class THReceiver {
    public:
      void begin(int interruptPin);
      bool isAvailable();
      Measurement getLastMeasurement();

    private:
      int interrupt; /* The interrupt this receiver is listening to NOT THE PIN */

      static void handleInterrupt();
      static bool processRawBitstream();
  };

#endif
