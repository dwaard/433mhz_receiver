#ifndef LOGTYPES_H
#define LOGTYPES_H

#include <stdint.h>

namespace LoggerTypes {
    static const uint8_t NONE = 0;
    static const uint8_t TRACE = 5;
    static const uint8_t DEBUG = 10;
    static const uint8_t INFO = 20;
    static const uint8_t WARNING = 30;
    static const uint8_t ERROR = 40;
    static const uint8_t CRITICAL = 50;

    static const uint16_t CODE_CHAR_ARRAY = 0;
    static const uint16_t CODE_STRING = 1;
    static const uint16_t CODE_INT = 2;
    static const uint16_t CODE_FLOAT = 3;
    static const uint16_t CODE_DOUBLE = 4;
    static const uint16_t CODE_BOOL = 5;
}

#endif