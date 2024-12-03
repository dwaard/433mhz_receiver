#include "THDisplay.h"

static const unsigned char PROGMEM tspk_bmp[] =
{ B01111110,
  B01000010,
  B01000010,
  B01000010,
  B01111110,
  B00011000,
  B00010000,
  B00000000 };

static const unsigned char PROGMEM wifi_off_bmp[] =
{ B01010000,
  B00100000,
  B01010000,
  B00000000,
  B00000000,
  B00000000,
  B01010101,
  B00000000 };


THDisplay::THDisplay(uint8_t w, uint8_t h, TwoWire *twi, int8_t rst_pin, uint32_t clkDuring, uint32_t clkAfter) {
  display = Adafruit_SSD1306(w, h, twi, rst_pin, clkDuring, clkAfter);
  for (size_t i = 0; i < STATUS_ARRAY_SIZE; ++i) {
      lines[i] = ""; // Explicitly initialize each element
  }
  for (size_t i = 0; i < DEVICE_ARRAY_SIZE; ++i) {
      devices[i] = ""; // Explicitly initialize each element
  }
}

bool THDisplay::begin(uint8_t switchvcc, uint8_t i2caddr, bool reset, bool periphBegin) {
  return display.begin(switchvcc, i2caddr, reset, periphBegin);
}

void THDisplay::setRotation(uint8_t r) {
  display.setRotation(r);
}
 
void THDisplay::printnln(String line) {
  lines[lineptr] = line;
  render();
}

void THDisplay::println(String line) {
  lines[lineptr] = line;
  lineptr = (lineptr + 1) % STATUS_ARRAY_SIZE;
  render();
}

void THDisplay::updateDeviceInfo(unsigned int index, String status) {
  devices[index] = status;
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
  unsigned int y = 0;
  // ThingSpeak status
  display.drawBitmap(x, y, tspk_bmp, 8, 8, 1);
  display.setCursor(x + 8, y);
  display.print(String(tspkStatus));

  // WiFi status
  // display.setCursor(80, y);
  // display.print(String(wifiRSSI));
  x = 121;
  if (!wifiConnected) {
    display.drawBitmap(x, y, wifi_off_bmp, 8, 8, 1);
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
  y = 1;
  // Device info
  for (unsigned int index = 0; index < DEVICE_ARRAY_SIZE; index++) {
    if (index == DEVICE_ARRAY_SIZE / 2) {
      x = 64;
      y = 9;
    } else {
      y += 8;
    }
    display.setCursor(x, y);
    display.print(devices[index]);
  }

  // Display the status lines
  display.setCursor(0, (8 - visibleLines) * 8 + 1);
  unsigned int startindex = (lineptr + STATUS_ARRAY_SIZE - (visibleLines % STATUS_ARRAY_SIZE)) % STATUS_ARRAY_SIZE;
  for (unsigned int index = 0; index < visibleLines; index++) {
    display.println(lines[(startindex + index) % STATUS_ARRAY_SIZE]);
  }

  display.display(); 
}
