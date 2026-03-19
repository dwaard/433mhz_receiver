// SerialLogChannel.h
#ifndef DISPLAYLOGCHANNEL_H
#define DISPLAYLOGCHANNEL_H

#include "Logger.h"
#include "LogChannel.h"

class DisplayLogChannel : public LogChannel 
{
public:
    // Constructor met logLevel-parameter (default: INFO)
    DisplayLogChannel(uint8_t level = Logger::INFO) : LogChannel(level) {
    }

    char* getRecent(int index) {
        // Return the most recent log message at the specified index
        // Index 0 is the most recent, index 1 is the second most recent, etc.
        return recentMessages[(currentIndex + index) % 3];
    }   

protected:
    char recentMessages[3][32] = {{""}, {""}, {""}}; // Buffer to store the 3 most recent log messages
    uint8_t currentIndex = 0; // Index to track the current position in the recentMessages buffer

    void processEvent(const LogEvent& event) override {
        // Store the log message in the recentMessages buffer
        recentMessages[currentIndex][0] = '\0'; // Clear the current message buffer
        snprintf(recentMessages[currentIndex], sizeof(recentMessages[currentIndex]), formatEventDefault(event).c_str());
        currentIndex = (currentIndex + 1) % 3; // Move to the next index in a circular manner
    }
};

#endif
