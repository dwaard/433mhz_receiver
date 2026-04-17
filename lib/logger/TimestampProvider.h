#ifndef TIMESTAMPPROVIDER_H
#define TIMESTAMPPROVIDER_H

/**
 * @class TimestampProvider
 * @brief Abstract base class for providing timestamps to log events
 */
class TimestampProvider {
public:
    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~TimestampProvider() = default;

    /**
     * @brief Retrieves the current timestamp
     * @return The current timestamp as an unsigned long value
     */
    virtual unsigned long getTimestamp() = 0;
};

#endif
