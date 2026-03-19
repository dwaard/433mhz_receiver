// SerialLogChannel.h
#ifndef SERIALCHANNEL_H
#define SERIALCHANNEL_H

#include "Logger.h"
#include "LogChannel.h"

class SerialChannel : public LogChannel 
{
public:
    // Constructor met logLevel-parameter (default: INFO)
    SerialChannel(uint8_t level = Logger::INFO) : LogChannel(level) {
    }

protected:
    void processEvent(const LogEvent& event) override {
        Serial.println(formatEventDefault(event));
    }
};

#endif
