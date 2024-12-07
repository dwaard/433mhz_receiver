// Include guard. Prevents program errors if the library is accidentally included more than once in the sketch
#ifndef _THDisplay_h 
  #define _THDisplay_h
  #include <Wire.h>
  #include <Adafruit_SSD1306.h>

  #define SCREEN_WIDTH    128 // OLED display width, in pixels
  #define SCREEN_HEIGHT    64 // OLED display height, in pixels
  #define OLED_RESET       -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  #define SCREEN_ADDRESS 0x3C //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

  #define MAX_LOG_LINES     8 // Amount of log lines visible on the display
  #define LOG_FIRST_LINE_X  0 // X-coordinate of the first log line

  class THDisplay {
    public:
      THDisplay();

      bool begin();

      void println(String l);

    private:
      Adafruit_SSD1306 display;

      String lines[MAX_LOG_LINES];
      unsigned int lineptr = 0;

      void render();      
  };
  
  extern THDisplay Display;
#endif