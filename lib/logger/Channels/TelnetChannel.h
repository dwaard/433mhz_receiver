// TelnetChannel.h
#ifndef TELNETCHANNEL_H
#define TELNETCHANNEL_H

#include "Logger.h"
#include "LogChannel.h"
#include <WiFiClient.h>

class TelnetChannel : public LogChannel 
{
public:
    // Constructor met logLevel-parameter (default: INFO)
    TelnetChannel(WiFiClient* client, uint8_t level = Logger::INFO) : LogChannel(level) {
        telnetClient = client;
    }

protected:
    void processEvent(const LogEvent& event) override {
        // Formatteer het logberichten
        const char* levelStr = getLevelStr(event);
        if (telnetClient->connected()) {
            telnetClient->print(levelStr);
            telnetClient->print("[");
            telnetClient->print(event.module);
            telnetClient->print("][");
            telnetClient->print(event.timestamp);
            telnetClient->print("] ");
            telnetClient->println(event.message);
        }
    }
    
private:
    WiFiClient* telnetClient;

};

#endif
