#ifndef CALLBACKCHANNEL_H
#define CALLBACKCHANNEL_H

#include "Logger.h"
#include "LogChannel.h"
#include <functional>

/**
 * @class CallbackChannel
 * @brief LogChannel implementation that invokes a callback function for each log event
 */
class CallbackChannel : public LogChannel 
{
public:
    /**
     * @typedef Callback
     * @brief Function type for handling log events
     */
    using Callback = std::function<void(const LogEvent&)>;

    /**
     * @brief Constructor that sets the minimum log level and callback function
     * @param level The minimum LogLevel to process
     * @param callback The function to invoke when a log event is processed
     */
    CallbackChannel(Logger::LogLevel level, Callback callback)
        : LogChannel(level), callback(callback) {}

protected:
    /**
     * @brief Processes a log event by invoking the registered callback function
     * @param event The LogEvent to process
     */
    void processEvent(const LogEvent& event) override {
        callback(event);
    }

private:
    Callback callback;  ///< The callback function to invoke for each log event
};

#endif
