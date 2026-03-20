// SerialLogChannel.h
#ifndef DISPLAYLOGCHANNEL_H
#define DISPLAYLOGCHANNEL_H

#include "Logger.h"
#include "LogChannel.h"
#include "THDisplay.h"

class DisplayLogChannel : public LogChannel 
{
public:
    // Constructor met logLevel-parameter (default: INFO)
    DisplayLogChannel(THDisplay& display, uint8_t level = Logger::INFO) : LogChannel(level), display(display)  {
    }

    char* getRecent(int index) {
        // Return the most recent log message at the specified index
        // Index 0 is the most recent, index 1 is the second most recent, etc.
        return recentMessages[(currentIndex + index) % 3];
    }   

protected:
    THDisplay& display; // Reference to the display object for rendering log messages
    char recentMessages[3][32] = {{""}, {""}, {""}}; // Buffer to store the 3 most recent log messages
    uint8_t currentIndex = 0; // Index to track the current position in the recentMessages buffer

    void processEvent(const LogEvent& event) override {
        display.println(formatEventDefault(event)); // Print the log message to the display
    }
};

#endif
