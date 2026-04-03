#include "THDisplay.h"

// The variable that holds the global Display instance
THDisplay Display;

THDisplay::THDisplay() {
  display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  for (int i = 0; i < MAX_LOG_LINES; i++) {
    lines[i] = "";
  }
}

bool THDisplay::begin() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  bool succeeded = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  if(succeeded) {
    display.setRotation(2);
    println("Display connected.");
  }
  return succeeded;
}

void THDisplay::updateDeviceInfo(unsigned int index, String status) {
  devices[index] = status;
  render();
}

void THDisplay::updateIPAdress(String updated) {
  ipAddress = updated;
  render();
}

void THDisplay::updateThingSpeakStatus(int status) {
  tspkStatus = status;
  render();
}

void THDisplay::updateWifiStatus(bool connected, int8_t rssi) {
  wifiConnected = connected;
  wifiRSSI = rssi;
  render();
}

void THDisplay::render() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  unsigned int x = 0;
  unsigned int y = STATUS_FIRST_LINE_Y;
  display.setCursor(x, y);

  if (!booting) {
    // ThingSpeak status
    display.drawBitmap(x, y, tspk_bmp, FONT_WIDTH, FONT_HEIGHT, 1);
    display.setCursor(x + FONT_WIDTH, y);
    display.print(String(tspkStatus));
    display.drawLine(0, y + FONT_HEIGHT, SCREEN_WIDTH, y + FONT_HEIGHT, 1);

    // IPAdress
    display.setCursor(32, y);
    display.print(ipAddress);

    // WiFi status
    display.setCursor(80, y);
    // display.print(String(wifiRSSI));
    x = 121;
    if (!wifiConnected) {
      display.drawBitmap(x, y, wifi_off_bmp, FONT_WIDTH, FONT_HEIGHT, 1);
    } else {
      display.drawLine(x + 2, y + 6, x + 2, y + 5, 1);
      if (wifiRSSI >= -70) {
        display.drawLine(x + 4, y + 6, x + 4, y + 4, 1);    
      }
      if (wifiRSSI >= -60) {
        display.drawLine(x + 6, y + 6, x + 6, y + 3, 1);
      }
      if (wifiRSSI >= -55) {
        display.drawLine(x + 8, y + 6, x + 8, y + 2, 1);
      }
    }

    x = 0;
    y = DEVICES_FIRST_LINE_Y;
    // Device info
    for (unsigned int index = 0; index < DEVICE_ARRAY_SIZE; index++) {
      if (index >= DEVICE_ARRAY_SIZE / 2 && x == 0) {
        x = SCREEN_WIDTH / 2;
        y = DEVICES_FIRST_LINE_Y;
      }
      display.setCursor(x, y);
      display.print(devices[index]);
      y += FONT_HEIGHT;
    }

    y = LOG_FIRST_LINE_Y - 2;
    display.drawLine(0, y, SCREEN_WIDTH, y, 1);
    display.setCursor(0, LOG_FIRST_LINE_Y);
  }

  unsigned int linesToShow = booting ? MAX_LOG_LINES_BOOT : MAX_LOG_LINES;
  // Display the logging lines
  int printIndex = lineptr; // Start from the oldest line to show
  for (unsigned int index = 0; index < linesToShow; index++) {
    display.println(lines[printIndex]);
    // Move to previous line, wrapping around if necessary
    printIndex = (printIndex + 1) % linesToShow;
  }
  display.display(); 
}


void THDisplay::println(String l) {
  lines[lineptr] = l;
  lineptr = (lineptr + 1) % MAX_LOG_LINES_BOOT;
  render();
}

