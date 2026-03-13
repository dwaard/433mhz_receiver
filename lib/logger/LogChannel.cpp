#include "Logger.h"
#include "LogChannel.h"

const char* LogChannel::getLevelStr(const LogEvent& event) {
    switch (event.level) {
        case Logger::DEBUG:    return "[DBG]";
        case Logger::INFO:     return "[INF]";
        case Logger::WARNING:  return "[WRN]";
        case Logger::ERROR:    return "[ERR]";
        case Logger::CRITICAL: return "[CRT]";
        default:               return "";
    }
}
