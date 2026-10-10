/******************************************************************************
 * @file manage_webserver.h
 * @brief Functions for managing the web server and its endpoints.
 * 
 *****************************************************************************/
#ifndef MANAGE_WEBSERVER_H
#define MANAGE_WEBSERVER_H

#include <ArduinoJson.h>
#include <ESP8266WebServer.h>

#include "config.h"
#include "manage_eeprom.h"

ESP8266WebServer server(80);

void configToJson(JsonObject configJson) {
  configJson["telnetLogLevel"] = config.telnetLogLevel;
  configJson["thingSpeakLogLevel"] = config.thingSpeakLogLevel;
  configJson["version"] = config.version;
}

void sensorConfigToJson(JsonObject sensor, int index) {
  sensor["id"] = index + 1;
  sensor["deviceID"] = sensorConfigs[index].deviceID;
  sensor["displayID"] = sensorConfigs[index].displayID;
  sensor["code"] = sensorConfigs[index].code;
  sensor["name"] = sensorConfigs[index].name;
  sensor["hasHumidity"] = sensorConfigs[index].hasHumidity();
  sensor["channel"] = sensorConfigs[index].getChannel();
  sensor["correction"] = sensorConfigs[index].getCorrection();
  sensor["maxValidDelta"] = sensorConfigs[index].getMaxValidDelta();
  sensor["maxValidInterval"] = sensorConfigs[index].getMaxValidInterval();
}

void sensorStatusToJson(JsonObject sensor, int index) {
  sensor["id"] = index + 1;
  sensor["displayID"] = sensorConfigs[index].displayID;
  sensor["deviceID"] = sensorConfigs[index].deviceID;
  sensor["state"] = sensors[index]->getState();
  sensor["code"] = sensorConfigs[index].code;
  sensor["channel"] = sensorConfigs[index].getChannel();
  sensor["name"] = sensorConfigs[index].name;
  sensor["temperature"] = sensors[index]->getLastTemp();
  if (sensorConfigs[index].hasHumidity()) {
    sensor["humidity"] = sensors[index]->getLastHumidity();
  }
  sensor["batteryState"] = sensors[index]->getLastBatteryState();
  sensor["age"] = sensors[index]->getLastAge() / 1000; // Age in seconds
}

// Forward declarations of endpoint handlers
// Include this macro in setup() to initialize the web server and register the endpoints
#define BEGIN_WEB_SERVER() \
  server.on("/", HTTP_GET, handleRoot); \
  server.on("/config", HTTP_GET, handleGetConfig); \
  server.on("/config", HTTP_PATCH, handlePatchConfig); \
  server.on("/config/reset", HTTP_POST, handleResetConfig); \
  server.on("/config/sensors", HTTP_GET, handleGetConfigSensors); \
  server.on(UriBraces("/config/sensors/{}"), HTTP_PATCH, handlePatchConfigSensors); \
  server.on("/data/report", HTTP_POST, handlePacket); \
  server.begin(); \
  Log.info("Webserver started on port 80");

/**
 * Handles GET requests to the root (/) endpoint.
 * 
 * It returns a JSON response containing an array of sensors with their actual
 * data.
 */
void handleRoot() {
  Log.debug("Received GET / request");
  JsonDocument json;
  JsonArray sensorsJson = json.to<JsonArray>();
  for (int i = 0; i < SENSOR_COUNT; i++) {
    JsonObject sensor = sensorsJson.add<JsonObject>();
    sensorStatusToJson(sensor, i);
  }
  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);
}

/**
 * Handles GET requests to the /config endpoint. 
 * 
 * It returns a JSON response containing the global config.
 */
void handleGetConfig() {
  Log.debug("Received GET /config request");
  JsonDocument response;
  JsonObject configJson = response.to<JsonObject>();
  configToJson(configJson);
  String output;
  serializeJson(response, output);
  server.send(200, "application/json", output);
}

/**
 * Handles PATCH request to the /config endpoint.
 * 
 * It updates the global config
 */
void handlePatchConfig() {
  Log.debug("Received PATCH /config request");
  if (server.hasArg("plain") == false) {
    server.send(400, "application/json", "{\"error\":\"Geen body\"}");
    return;
  }
  String body = server.arg("plain");
  JsonDocument json;
  DeserializationError error = deserializeJson(json, body);

  if (error) {
    server.send(400, "application/json", "{\"error\":\"Ongeldige JSON\"}");
    return;
  }

  if (json["telnetLogLevel"].is<uint8_t>()) {
    config.telnetLogLevel = json["telnetLogLevel"].as<uint8_t>();
    #ifdef TELNET_DEBUGGING_ENABLED
    telnetChannel.setLogLevel(config.telnetLogLevel);
    #endif
  }

  if (json["thingSpeakLogLevel"].is<uint8_t>()) {
    config.thingSpeakLogLevel = json["thingSpeakLogLevel"].as<uint8_t>();
  }

  // TODO remove this in the future, it's only needed to trigger reinitialization of config on next load after adding version control
  if (json["version"].is<uint8_t>()) {
    config.version = json["version"].as<uint8_t>();
  }

  saveConfig(config); // Sla bijgewerkte configuratie op

  JsonDocument response;
  JsonObject configJson = response.to<JsonObject>();
  configToJson(configJson);
  String output;
  serializeJson(response, output);
  server.send(200, "application/json", output);
} 

/**
 * Handles reset config requests
 */
void handleResetConfig() {
  Log.debug("Received POST /config/reset request");
  config.version = 0; // Reset version to trigger reinitialization on next load
  saveConfig(config);
  JsonDocument response;
  JsonObject configJson = response.to<JsonObject>();
  configToJson(configJson);
  String output;
  serializeJson(response, output);
  server.send(200, "application/json", output);
}

/**
 * Handles GET requests to the /config/sensors endpoint.
 */
void handleGetConfigSensors() {
  Log.debug("Received GET /config/sensors request");
  JsonDocument json;
  JsonArray sensorsJson = json.to<JsonArray>();
  for (int i = 0; i < SENSOR_COUNT; i++) {
    JsonObject sensor = sensorsJson.add<JsonObject>();
    sensorConfigToJson(sensor, i);
  }
  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);
}

/**
 * Handles PATCH requests to the /config/sensors endpoint.
 */
void handlePatchConfigSensors() {
  String arg = server.pathArg(0);
  int id = arg.toInt() - 1; // Convert to zero-based index
  Log.debug("Received PATCH /config/sensors request for sensor ID: " + arg);

  if (id < 0 || id >= SENSOR_COUNT) {
    server.send(404, "application/json", "{\"error\":\"Ongeldige sensor ID\"}");
    return;
  }

  if (server.hasArg("plain") == false) {
    server.send(400, "application/json", "{\"error\":\"Geen body\"}");
    return;
  }

  String body = server.arg("plain");
  JsonDocument json;
  DeserializationError error = deserializeJson(json, body);

  if (error) {
    server.send(422, "application/json", "{\"error\":\"Ongeldige JSON\"}");
    return;
  }

  if (json["deviceID"].is<uint8_t>()) {
    sensorConfigs[id].deviceID = json["deviceID"];
  }

  if (json["displayID"].is<uint8_t>()) {
    sensorConfigs[id].displayID = json["displayID"];
  }

  if (json["code"].is<const char*>()) {
    strlcpy(sensorConfigs[id].code, json["code"], sizeof(sensorConfigs[id].code));
  }

  if (json["name"].is<const char*>()) {
    strlcpy(sensorConfigs[id].name, json["name"], sizeof(sensorConfigs[id].name));
  }

  if (json["settings"].is<uint8_t>()) {
    sensorConfigs[id].setSettings(json["settings"]);
  }

  if (json["hasHumidity"].is<bool>()) {
    sensorConfigs[id].setHasHumidity(json["hasHumidity"]);
  }

  if (json["channel"].is<uint8_t>()) {
    sensorConfigs[id].setChannel(json["channel"]);
  }

  if (json["correction"].is<float>()) {
    sensorConfigs[id].setCorrection(json["correction"]);
  }

  if (json["maxValidDelta"].is<float>()) {
    sensorConfigs[id].setMaxValidDelta(json["maxValidDelta"]);
  }

  if (json["maxValidInterval"].is<uint8_t>()) {
    sensorConfigs[id].setMaxValidInterval(json["maxValidInterval"]);
  }

  saveSensorConfig(id, sensorConfigs[id]); // Sla bijgewerkte configuratie op

  JsonDocument response;
  JsonObject sensor = response.to<JsonObject>();
  sensorConfigToJson(sensor, id);

  String output;
  serializeJson(response, output);
  server.send(200, "application/json", output);
}

String getValueStr(const String& data, const char* key) {
  int keyIndex = data.indexOf(String(key) + "=");
  if (keyIndex == -1) {
    return "";
  }
  int valueStart = keyIndex + strlen(key) + 1; // +1 for the '=' character
  int valueEnd = data.indexOf("&", valueStart);
  if (valueEnd == -1) {
    valueEnd = data.length();
  }
  return data.substring(valueStart, valueEnd);
}

void handlePacket() {
  Log.trace("Received POST /data/report request");
  String body = server.arg("plain");
  // The body is in form-data. The keys needed are: "tempinf", "humidityin"
  // We will extract these values and log them
  String value = getValueStr(body, "tempinf");
  if (value == "") {
    Log.warning("Received weather station report without tempinf");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing tempinf\"}");
    return;
  }
  float tempIn = (value.toFloat() - 32) * 5.0 / 9.0;
  value = getValueStr(body, "humidityin");
  if (value == "") {
    Log.warning("Received weather station report without humidityin");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing humidityin\"}");
    return;
  }
  float humidityIn = value.toFloat();
  value = getValueStr(body, "tempf");
  if (value == "") {
    Log.warning("Received weather station report without tempf");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing tempf\"}");
    return;
  }
  float tempOut = (value.toFloat() - 32) * 5.0 / 9.0;
  value = getValueStr(body, "humidity");
  if (value == "") {
    Log.warning("Received weather station report without humidity");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing humidity\"}");
    return;
  }
  float humidityOut = value.toFloat();
  value = getValueStr(body, "wh65batt");
  if (value == "") {
    Log.warning("Received weather station report without wh65batt");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing wh65batt\"}");
    return;
  }
  bool batteryState = value.toInt() == 0; // wh65batt=1 indicates low battery for the outdoor sensor array, so we invert it for our batteryState

  THPacket packet = {};
  packet.deviceID = WS2910_IN_DEVICE_ID;
  packet.channelNo = WS2910_IN_DEVICE_CHANNEL;
  packet.batteryState = true;
  packet.temperature = tempIn;
  packet.humidity = humidityIn;
  packet.timestamp = millis();
  processPacket(packet);
  packet.deviceID = WS2910_OUT_DEVICE_ID;
  packet.channelNo = WS2910_OUT_DEVICE_CHANNEL;
  packet.batteryState = batteryState; // The outdoor sensor array is battery powered
  packet.temperature = tempOut;
  packet.humidity = humidityOut;
  packet.timestamp = millis();
  processPacket(packet);

  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

/**
 * Handles unhandlesd routes
 */
void handleNotFound() {
  Log.debug("Received 404 request: " + server.uri());
  String message = "File Not Found\n\n";
  server.send(404, "text/plain", message);
}

#endif // MANAGE_WEBSERVER_H