// LogEvent.h
#ifndef LOGEVENT_H
#define LOGEVENT_H

#include <stdint.h>

/**
 * @struct LogEvent
 * @brief Data structure containing all information associated with a single log entry
 */
struct LogEvent 
{
    /**
     * Timestamp of the log event. Typically provided by a TimestampProvider. 
     */
    unsigned long timestamp;

    /**
     * Log level indicating the severity of the event. Should correspond to Logger's 
     * log level constants (e.g., Logger::INFO, Logger::ERROR).
     */
    uint8_t level;

    /**
     * Optional code for categorizing log events (e.g., error codes, event types)
     */
    uint16_t code;

    /**
     * Pointer to additional data relevant to the log event (e.g., message, sensor 
     * readings, error details). The LogChannel implementation should know how 
     * to handle this. The default LogChannel implementations will expect this 
     * to be a pointer to a char array containing the log message.
     */
    void* data;
};
#endif
