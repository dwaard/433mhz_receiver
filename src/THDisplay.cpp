#include "THDisplay.h"

THDisplay Display;

THDisplay::THDisplay() {
  display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  for (int i = 0; i < MAX_LOG_LINES; i++) {
    lines[i] = "";
  }
}

bool THDisplay::begin() {
  bool succeeded = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(succeeded) {
    display.setRotation(2);
    println("Display connected.");
  }
  return succeeded;
}

void THDisplay::render() {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 1);
  display.clearDisplay();
  // Display the lines
  for (unsigned int index = 0; index < MAX_LOG_LINES; index++) {
    display.println(lines[(lineptr + index) % MAX_LOG_LINES]);
  }
  display.display(); 
}


void THDisplay::println(String l) {
  lines[lineptr] = l;
  lineptr = (lineptr + 1) % MAX_LOG_LINES;
  render();
}

