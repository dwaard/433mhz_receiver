#include "Logger.h"
#include "LogChannel.h"

const char* LogChannel::getLevelStr(const LogEvent& event) {
    if (event.level >= Logger::CRITICAL) return "CRIT";
    if (event.level >= Logger::ERROR)    return "ERRR";
    if (event.level >= Logger::WARNING)  return "WARN";
    if (event.level >= Logger::INFO)     return "INFO";
    if (event.level >= Logger::DEBUG)    return "DBUG";
    if (event.level >= Logger::TRACE)    return "TRAC";
    return "UNKN";
}

String LogChannel::getDataStr(const LogEvent& event) {
    switch (event.code) {
        case Logger::CODE_CHAR_ARRAY:
            return String((char*)event.data);
        case Logger::CODE_STRING:
            return String(*(String*)event.data);
        case Logger::CODE_INT:
            return String("Integer value: ") + String(*(int*)event.data);
        case Logger::CODE_FLOAT:
            return String("Float value: ") + String(*(float*)event.data);
        case Logger::CODE_DOUBLE:
            return String("Double value: ") + String(*(double*)event.data);
        case Logger::CODE_BOOL:
            return String("Boolean value: ") + (*(bool*)event.data ? "true" : "false");
        default:
            return String("Unknown data type. Unable to log data"); // Default to char array
    }
}