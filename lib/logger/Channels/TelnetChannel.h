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
        if (telnetClient->connected()) {
            telnetClient->println(formatEventDefault(event));
        }
    }
    
private:
    WiFiClient* telnetClient;

};

#endif
