// Include guard. Prevents program errors if the library is accidentally included more than once in the sketch
#ifndef _THDisplay_h 
  #define _THDisplay_h
  
  #include <Wire.h>
  #if defined(ARDUINO_ARCH_ESP8266)
  #include <ESP8266WiFi.h>

  #elif defined(ARDUINO_ARCH_AVR)
  #include <WiFi.h>

  #elif defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
  #endif

  #include <Adafruit_SSD1306.h>


  class THDisplay {
    public:
      THDisplay(uint8_t w, uint8_t h, TwoWire *twi = &Wire, int8_t rst_pin = (int8_t)(-1), uint32_t clkDuring = 400000U, uint32_t clkAfter = 100000U);

      bool begin(uint8_t switchvcc = (uint8_t)2U, uint8_t i2caddr = (uint8_t)0U, bool reset = true, bool periphBegin = true);

      void setRotation(uint8_t r);

      void printnln(String line);
      void println(String line);
      void updateDeviceInfo(unsigned int index, String status);

      void updateThingSpeakStatus(int status);

      void updateWifiStatus(bool connected, int8_t rssi);

    private:
      Adafruit_SSD1306 display;
      
      static const size_t DEVICE_ARRAY_SIZE = 8; // Size of the array
      String devices[DEVICE_ARRAY_SIZE];

      static const size_t STATUS_ARRAY_SIZE = 8; // Size of the array
      String lines[STATUS_ARRAY_SIZE];

      int tspkStatus = 0;
      bool wifiConnected = false;
      int8_t wifiRSSI = 0;
      unsigned int lineptr = 0;
      unsigned int visibleLines = 3;


      void render();
  };

#endif
