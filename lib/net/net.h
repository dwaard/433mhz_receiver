/**
 * net.h - A header file to include the appropriate WiFi library based on the platform.
 * It also defines a helper function to convert WiFi status codes to human-readable strings.
 */
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#pragma message("Building for ESP8266 platform...")

#elif defined(ARDUINO_ARCH_AVR)
#include <WiFi.h>
#pragma message("Building for Arduino AVR platform...")

#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#pragma message("Building for ESP32 platform...")
#endif

/**
 * Converts a WiFi status code to a human-readable string. This is useful for logging
 * the current WiFi status in a more understandable way.
 */
const char *wl_status_to_string(wl_status_t status)
{
  switch (status)
  {
  case WL_NO_SHIELD:
    return "WL_NO_SHIELD";
  case WL_IDLE_STATUS:
    return "WL_IDLE_STATUS";
  case WL_NO_SSID_AVAIL:
    return "WL_NO_SSID_AVAIL";
  case WL_SCAN_COMPLETED:
    return "WL_SCAN_COMPLETED";
  case WL_CONNECTED:
    return "WL_CONNECTED";
  case WL_CONNECT_FAILED:
    return "WL_CONNECT_FAILED";
  case WL_CONNECTION_LOST:
    return "WL_CONNECTION_LOST";
  case WL_WRONG_PASSWORD:
    return "WL_WRONG_PASSWORD";
  case WL_DISCONNECTED:
    return "WL_DISCONNECTED";
  }
  // Create a static buffer to hold the status value
  static char buffer[32];
  snprintf(buffer, sizeof(buffer), "UNKNOWN STATUS: %d", status);
  return buffer;
}
