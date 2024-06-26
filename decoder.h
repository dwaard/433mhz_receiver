// Include guard. Prevents program errors if the library is accidentally included more than once in the sketch
#ifndef decoder_h 
#define decoder_h
  #include "Arduino.h"
  /* Platform specific stuff */
  #if defined(ARDUINO_ARCH_ESP8266)
    #define INTERRUPT_ROUTINE void ICACHE_RAM_ATTR
  #elif defined(ARDUINO_ARCH_AVR)
    #define INTERRUPT_ROUTINE void
  #elif defined(ARDUINO_ARCH_ESP32)
    #define INTERRUPT_ROUTINE void IRAM_ATTR
  #endif

  /* Signal and bit stream characteristics */
  #define BITS_PER_PACKET 36
  #define PACKETS_PER_STREAM 10

  struct Measurement {
    unsigned long timestamp;
    uint8_t deviceID;
    bool batteryState;
    uint8_t channelNo;
    float temperature;
    uint8_t humidity;
  };

  class DecoderClass {
    public:

      static void begin(int pin);

      static bool hasMeasurement();

      static Measurement getMeasurement();

    private:
      static int _pin;                                              /* the pin the decoder will listen to */

      static INTERRUPT_ROUTINE mapPulseTimingsToBits();
      static bool processRawBitstream();
  };

  extern DecoderClass Decoder;
#endif
