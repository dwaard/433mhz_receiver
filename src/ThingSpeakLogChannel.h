// SerialLogChannel.h
#ifndef THINGSPEAKLOGCHANNEL_H
#define THINGSPEAKLOGCHANNEL_H

#include "Logger.h"
#include "LogChannel.h"

class ThingSpeakLogChannel : public LogChannel 
{
public:
    // Constructor met logLevel-parameter (default: INFO)
    ThingSpeakLogChannel(uint8_t level = Logger::INFO) : LogChannel(level) {
    }

    bool hasStatusToSend() {
        return unsendStatusString.length() > 0;
    }

    String getStatusForSending() {
        String result = unsendStatusString;
        unsendStatusString = "";
        return result;
    }

protected:
    void processEvent(const LogEvent& event) override {
        append(formatEventDefault(event));
    }

private:
    String unsendStatusString;

    void append(String status) {
        if (unsendStatusString.length() > 0) {
            unsendStatusString += "\n";
        }
        unsendStatusString += status;
    }
};

#endif
