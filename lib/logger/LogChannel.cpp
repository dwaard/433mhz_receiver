#include "LogTypes.h"
#include "LogChannel.h"

const char* LogChannel::getLevelStr(const LogEvent& event) {
    if (event.level >= LoggerTypes::CRITICAL) return "CRIT";
    if (event.level >= LoggerTypes::ERROR)    return "ERRR";
    if (event.level >= LoggerTypes::WARNING)  return "WARN";
    if (event.level >= LoggerTypes::INFO)     return "INFO";
    if (event.level >= LoggerTypes::DEBUG)    return "DBUG";
    if (event.level >= LoggerTypes::TRACE)    return "TRAC";
    return "UNKN";
}

String LogChannel::getDataStr(const LogEvent& event) {
    switch (event.code) {
        case LoggerTypes::CODE_CHAR_ARRAY:
            return String((char*)event.data);
        case LoggerTypes::CODE_STRING:
            return String(*(String*)event.data);
        case LoggerTypes::CODE_INT:
            return String("Integer value: ") + String(*(int*)event.data);
        case LoggerTypes::CODE_FLOAT:
            return String("Float value: ") + String(*(float*)event.data);
        case LoggerTypes::CODE_DOUBLE:
            return String("Double value: ") + String(*(double*)event.data);
        case LoggerTypes::CODE_BOOL:
            return String("Boolean value: ") + (*(bool*)event.data ? "true" : "false");
        default:
            return String("Unknown data type. Unable to log data"); // Default to char array
    }
}