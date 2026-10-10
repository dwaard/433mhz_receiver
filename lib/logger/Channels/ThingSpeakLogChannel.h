// SerialLogChannel.h
#ifndef THINGSPEAKLOGCHANNEL_H
#define THINGSPEAKLOGCHANNEL_H

#include "Logger.h"
#include "LogChannel.h"

class ThingSpeakLogChannel : public LogChannel 
{
public:
    static constexpr uint8_t QUEUE_SIZE = 16;

    // Constructor met logLevel-parameter (default: INFO)
    ThingSpeakLogChannel(uint8_t level = Logger::INFO) : LogChannel(level) {
        for (uint8_t i = 0; i < QUEUE_SIZE; i++) {
            statusQueue[i] = nullptr;
        }
    }

    ~ThingSpeakLogChannel() {
        clearQueue();
    }

    ThingSpeakLogChannel(const ThingSpeakLogChannel&) = delete;
    ThingSpeakLogChannel& operator=(const ThingSpeakLogChannel&) = delete;

    bool hasStatusToSend() {
        return queueCount > 0;
    }

    String* popStatusForSending() {
        if (queueCount == 0) {
            return nullptr;
        }

        String* result = statusQueue[queueHead];
        statusQueue[queueHead] = nullptr;
        queueHead = (queueHead + 1) % QUEUE_SIZE;
        queueCount--;

        return result;
    }

    String getStatusForSending() {
        String* statusPtr = popStatusForSending();
        if (statusPtr == nullptr) {
            return "";
        }

        String result = *statusPtr;
        delete statusPtr;
        return result;
    }

protected:
    void processEvent(const LogEvent& event) override {
        append(formatEventDefault(event));
    }

private:
    String* statusQueue[QUEUE_SIZE];
    uint8_t queueHead = 0;
    uint8_t queueTail = 0;
    uint8_t queueCount = 0;
    

    void append(String status) {
        String* queuedStatus = new String(status);
        if (queuedStatus == nullptr) {
            return;
        }

        if (queueCount == QUEUE_SIZE) {
            delete statusQueue[queueHead];
            statusQueue[queueHead] = nullptr;
            queueHead = (queueHead + 1) % QUEUE_SIZE;
            queueCount--;
        }

        statusQueue[queueTail] = queuedStatus;
        queueTail = (queueTail + 1) % QUEUE_SIZE;
        queueCount++;
    }

    void clearQueue() {
        while (queueCount > 0) {
            delete statusQueue[queueHead];
            statusQueue[queueHead] = nullptr;
            queueHead = (queueHead + 1) % QUEUE_SIZE;
            queueCount--;
        }

        queueHead = 0;
        queueTail = 0;
    }
};

#endif
