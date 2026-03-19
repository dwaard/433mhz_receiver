// Logger.h
#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <vector>
#include "LogEvent.h"
#include "LogChannel.h"
#include "TimestampProvider.h"
#include "TimestampProviders/MillisTimestampProvider.h"

/**
 * @class Logger
 * @brief Central logging manager that dispatches log events to registered channels
 * @details Manages log channels, timestamp providers, and routes log events based on severity levels
 */
class Logger {
public:
    // declare a constant NONE log level for disabling logging
    static const uint8_t NONE = 0; ///< No logging
    static const uint8_t TRACE = 5; ///< Very detailed tracing information (not commonly used)
    static const uint8_t DEBUG = 10; ///< Detailed debugging information
    static const uint8_t INFO = 20; ///< General informational messages
    static const uint8_t WARNING = 30; ///< Warning messages for potentially problematic situations
    static const uint8_t ERROR = 40; ///< Error messages for recoverable failures
    static const uint8_t CRITICAL = 50; ///< Critical messages for severe failures

    static const uint16_t CODE_CHAR_ARRAY = 0;
    static const uint16_t CODE_STRING = 1;
    static const uint16_t CODE_INT = 2;
    static const uint16_t CODE_FLOAT = 3;
    static const uint16_t CODE_DOUBLE = 4;
    static const uint16_t CODE_BOOL = 5;

    /**
     * @brief Constructor initializing the logger with default MillisTimestampProvider
     */
    Logger() : timestampProvider(&defaultMillisProvider) {}

    /**
     * @brief Destructor that cleans up all allocated LogChannel objects
     */
    ~Logger() {
        for (auto channel : channels) {
            delete channel;
        }
    }

    /**
     * @brief Sets a custom timestamp provider for log events
     * @param provider Pointer to a TimestampProvider implementation
     */
    void setTimestampProvider(TimestampProvider* provider) {
        timestampProvider = provider;
    }

    /**
     * @brief Registers a log channel to receive log events
     * @param channel Pointer to a LogChannel implementation to add
     */
    void addChannel(LogChannel* channel) {
        channels.push_back(channel);
    }

    /**
     * @brief Dispatches a log event to all registered channels
     * @param event The LogEvent to be processed by each channel
     */
    void log(const LogEvent& event) {
        // When there are channels, pass the event to each one
        if (!channels.empty()) {
            for (auto channel : channels) {
                channel->handle(event);
            }
        }
    }

    /**
     * @brief Logs a trace-level message with module identification
     * @param module The source module or component name
     * @param data The trace message content
     * @param code Optional code for categorizing the log event (default: 0)
     */
    void trace(void* data, u_int16_t code = 0) {
        log({timestampProvider->getTimestamp(), Logger::TRACE, code, data});
    }

    /**
     * @brief Logs a trace-level message with module identification
     * @param message The trace message content
     */
    void trace(String message) {
        trace((void*)message.c_str());
    }

    /**
     * @brief Logs a debug-level message with module identification
     * @param data The debug message content
     * @param code Optional code for categorizing the log event (default: 0)
     */
    void debug(void* data, u_int16_t code = 0) {
        log({timestampProvider->getTimestamp(), Logger::DEBUG, code, data});
    }

    /**
     * @brief Logs a debug-level message with module identification
     * @param message The debug message content
     */
    void debug(String message) {
        debug((void*)message.c_str());
    }

    /**
     * @brief Logs an info-level message with module identification
     * @param data The informational message content
     * @param code Optional code for categorizing the log event (default: 0)
     */
    void info(void* data, u_int16_t code = 0) {
        log({timestampProvider->getTimestamp(), Logger::INFO, code, data});
    }

    /**
     * @brief Logs an info-level message with module identification
     * @param message The info message content
     */
    void info(String message) {
        info((void*)message.c_str());
    }

    /**
     * @brief Logs a warning-level message with module identification
     * @param data The warning message content
     * @param code Optional code for categorizing the log event (default: 0)
     */
    void warning(void* data, u_int16_t code = 0) {
        log({timestampProvider->getTimestamp(), Logger::WARNING, code, data});
    }

    /**
     * @brief Logs a warning-level message with module identification
     * @param message The warning message content
     */
    void warning(String message) {
        warning((void*)message.c_str());
    }

    /**
     * @brief Logs an error-level message with module identification
     * @param data The error message content
     * @param code Optional code for categorizing the log event (default: 0)
     */
    void error(void* data, u_int16_t code = 0) {
        log({timestampProvider->getTimestamp(), Logger::ERROR, code, data});
    }

    /**
     * @brief Logs an error-level message with module identification
     * @param message The error message content
     */
    void error(String message) {
        error((void*)message.c_str());
    }

    /**
     * @brief Logs a critical-level message with module identification
     * @param data The critical message content
     * @param code Optional code for categorizing the log event (default: 0)
     */
    void critical(void* data, u_int16_t code = 0) {
        log({timestampProvider->getTimestamp(), Logger::CRITICAL, code, data});
    }

    /**
     * @brief Logs a critical-level message with module identification
     * @param message The critical message content
     */
    void critical(String message) {
        critical((void*)message.c_str());
    }

private:
    std::vector<LogChannel*> channels;              ///< Collection of registered log channels
    TimestampProvider* timestampProvider;           ///< Current timestamp provider implementation
    MillisTimestampProvider defaultMillisProvider;  ///< Default millisecond-based timestamp provider
};

/**
 * @brief Global logger instance for application-wide logging
 */
extern Logger Log;

#endif
