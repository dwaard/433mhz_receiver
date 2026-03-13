// LogEvent.h
#ifndef LOGEVENT_H
#define LOGEVENT_H

#include <Arduino.h>

/**
 * @struct LogEvent
 * @brief Data structure containing all information associated with a single log entry
 */
struct LogEvent 
{
    unsigned long timestamp;

    uint8_t level;  // Use int instead of Logger::LogLevel to avoid circular dependency

    String module;
    
    String message;
};

#endif
