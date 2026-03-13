// MillisTimestampProvider.h
#ifndef MILLISTIMESTAMPPROVIDER_H
#define MILLISTIMESTAMPPROVIDER_H

#include "TimestampProvider.h"

class MillisTimestampProvider : public TimestampProvider 
{
public:
    unsigned long getTimestamp() override {
        return millis();
    }
};

#endif
