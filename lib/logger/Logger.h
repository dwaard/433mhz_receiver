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
        // When there are no channels, just print to Serial as a fallback
        if (channels.empty()) {
            Serial.print("[FALLBACK] [");
            Serial.print(event.module);
            Serial.print("] ");
            Serial.println(event.message);
            return;
        } else {
            for (auto channel : channels) {
                channel->handle(event);
            }
        }
    }

    /**
     * @brief Logs a trace-level message with module identification
     * @param module The source module or component name
     * @param message The trace message content
     */
    void trace(const String& module, const String& message) {
        log({timestampProvider->getTimestamp(), Logger::TRACE, module, message});
    }

    /**
     * @brief Logs a debug-level message with module identification
     * @param module The source module or component name
     * @param message The debug message content
     */
    void debug(const String& module, const String& message) {
        log({timestampProvider->getTimestamp(), Logger::DEBUG, module, message});
    }

    /**
     * @brief Logs an info-level message with module identification
     * @param module The source module or component name
     * @param message The informational message content
     */
    void info(const String& module, const String& message) {
        log({timestampProvider->getTimestamp(), Logger::INFO, module, message});
    }

    /**
     * @brief Logs a warning-level message with module identification
     * @param module The source module or component name
     * @param message The warning message content
     */
    void warning(const String& module, const String& message) {
        log({timestampProvider->getTimestamp(), Logger::WARNING, module, message});
    }

    /**
     * @brief Logs an error-level message with module identification
     * @param module The source module or component name
     * @param message The error message content
     */
    void error(const String& module, const String& message) {
        log({timestampProvider->getTimestamp(), Logger::ERROR, module, message});
    }

    /**
     * @brief Logs a critical-level message with module identification
     * @param module The source module or component name
     * @param message The critical message content
     */
    void critical(const String& module, const String& message) {
        log({timestampProvider->getTimestamp(), Logger::CRITICAL, module, message});
    }

private:
    std::vector<LogChannel*> channels;              ///< Collection of registered log channels
    TimestampProvider* timestampProvider;           ///< Current timestamp provider implementation
    MillisTimestampProvider defaultMillisProvider;  ///< Default millisecond-based timestamp provider
};

/**
 * @brief Global logger instance for application-wide logging
 */
extern Logger Logging;

#endif
