#ifndef LOGCHANNEL_H
#define LOGCHANNEL_H

#include "LogEvent.h"

/**
 * @class LogChannel
 * @brief Abstract base class for handling log events with level filtering
 */
class LogChannel 
{
public:
    /**
     * @brief Constructor that sets the minimum log level for this channel
     * @param level The minimum LogLevel to process (default: INFO = 20)
     */
    LogChannel(uint8_t level = 20) : logLevel(level) {}
    
    /**
     * @brief Sets the minimum log level for this channel
     * @param level The new LogLevel threshold
     */
    void setLogLevel(uint8_t level) {
        logLevel = level;
    }

    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~LogChannel() = default;

    /**
     * @brief Processes a log event if it meets the minimum log level threshold
     * @param event The LogEvent to handle
     */
    void handle(const LogEvent& event) {
        if (event.level < logLevel) {
            return;
        }
        processEvent(event);
    }

protected:
    /**
     * @brief Pure virtual method for derived classes to implement event processing
     * @param event The LogEvent to process
     */
    virtual void processEvent(const LogEvent& event) = 0;

    /**
     * @brief Helper method to convert LogLevel to a string representation
     * @param event The LogEvent containing the log level
     * @return A pointer to the string representation of the log level
     */
    const char* getLevelStr(const LogEvent& event);

    /**
     * @brief Helper method to convert LogEvent data to a string representation
     * @param event The LogEvent containing the data
     * @return A String containing the formatted data
     */
    String getDataStr(const LogEvent& event);

    /**
     * @brief Helper method to format a LogEvent into a human-readable string
     * @param event The LogEvent to format
     */
    String formatEventDefault(const LogEvent& event) {
        String result = "[" + String(getLevelStr(event)) + "] [" + String(event.timestamp) + "] ";
        if (event.code != 100) {
            result += "(" + String(event.code) + ")";
        }
        result += " " + getDataStr(event);
        return result;
    }

    uint8_t logLevel;  ///< Current log level threshold
};

#endif
