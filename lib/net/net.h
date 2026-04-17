/**
 * net.h - A header file to include the appropriate WiFi library based on the platform.
 * It also defines a helper function to convert WiFi status codes to human-readable strings.
 */
#include "Logger.h"

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_AVR)
#include <WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

/**
 * Converts a WiFi status code to a human-readable string. This is useful for logging
 * the current WiFi status in a more understandable way.
 */
const char *wl_status_to_string(wl_status_t status)
{
  switch (status)
  {
  case WL_NO_SHIELD      : return "No shield";
  case WL_IDLE_STATUS    : return "Idle";
  case WL_NO_SSID_AVAIL  : return "No SSID available";
  case WL_SCAN_COMPLETED : return "Scan completed";
  case WL_CONNECTED      : return "Connected";
  case WL_CONNECT_FAILED : return "Connection failed";
  case WL_CONNECTION_LOST: return "Connection lost";
  case WL_WRONG_PASSWORD : return "Wrong password";
  case WL_DISCONNECTED   : return "Disconnected";
  }
  // Create a static buffer to hold the status value
  static char buffer[32];
  snprintf(buffer, sizeof(buffer), "UNKNOWN STATUS: %d", status);
  return buffer;
}

int status = WL_IDLE_STATUS;     // the WiFi radio's status

/**
 * @brief (Re)connects to the WiFi network using the provided SSID and password.
 */
void connectWifi(const char *ssid, const char *passphrase) {
  if (status == WL_CONNECTED) {
    return;
  }
  // check for the WiFi module:
  while (WiFi.status() == WL_NO_SHIELD) {
    // don't continue
    Log.critical("No WiFi module! System HALT");
    delay(1000);
  }
  // Connect or reconnect to WiFi
  bool initializing = status == WL_IDLE_STATUS || WiFi.SSID() != ssid;
  if (initializing) {
    Log.info("Initializing Wifi. SSID: " + String(ssid));
    WiFi.mode(WIFI_STA);
  }
  if (WiFi.status() != WL_CONNECTED) {
    if (!initializing) {
      Log.warning("WiFi not connected. Attempting to connect...");
    }
    status = WiFi.begin(ssid, passphrase);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
    delay(500); // Wait for the wifi module to initialize and attempt connection
    while(status != WL_CONNECTED) {
      Log.warning("Awaiting WiFi connection. Status: " + String(wl_status_to_string(WiFi.status())));
      delay(500);
      status = WiFi.status();
    }
  }
  Log.info("WiFi connection successful. IP address: " + WiFi.localIP().toString());
}
