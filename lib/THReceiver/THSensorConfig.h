#ifndef _THSensorConfig_h
#define _THSensorConfig_h

class THSensorConfig
{
public:
    /**
     * @brief Unique identifier byte for each device.
     *
     * Normally a 433 MHz receiver assigns a random 8-bit ID each time it powers
     * on. This value is used to match incoming RF packets to the correct sensor
     * configuration entry in EEPROM. A value of 0xFF indicates an uninitialized
     * (invalid) entry.
     */
    uint8_t deviceID;

    /**
     * @brief Position index used by the front-end display.
     *
     * Determines where this sensor appears on the dashboard. For example, with
     * a 4-slot display, valid values are 0–3. A value of 0 typically means
     * "not assigned".
     */
    uint8_t displayID;

    /**
     * @brief Short alphanumeric code identifying this sensor.
     *
     * Used in compact log lines and small displays. Maximum 4 printable
     * characters; the 5th byte is always a null terminator. Examples: "T1",
     * "OUT1", "H2".
     */
    char code[5];

    /**
     * @brief Human-readable name for this sensor.
     *
     * Shown on front-end dashboards. Maximum 15 characters; the 16th byte is
     * always a null terminator. Example: "Living Room".
     */
    char name[16];

    // -------------------------------------------------------------------------
    // Constructors
    // -------------------------------------------------------------------------

    /**
     * @brief Default constructor — initializes the object with safe defaults.
     *
     * The resulting object represents a logically "invalid" sensor that will not
     * match any real device:
     * - `deviceID`       = 0xFF (unassigned)
     * - `displayID`      = 0
     * - `code` / `name`  = empty strings
     * - `hasHumidity`    = true  (bit 0 of settings = 1)
     * - `channel`        = 0     (bits 1–2 of settings = 0)
     * - `correction`     = 0     (no calibration offset)
     * - `maxValidDelta`  = 0
     * - `maxValidInterval` = 0
     */
    THSensorConfig()
        : deviceID(0xFF), displayID(0), code{0}, name{0},
          settings(0b00000001), correction(0), maxValidDelta(0), maxValidInterval(0) {}

    // -------------------------------------------------------------------------
    // Humidity flag — bit 0 of the settings byte
    // -------------------------------------------------------------------------

    /**
     * @brief Returns whether this sensor reports humidity measurements.
     *
     * The humidity flag is stored in bit 0 of the internal settings byte.
     *
     * @return `true`  if the sensor produces humidity readings.
     * @return `false` if the sensor is temperature-only.
     */
    bool hasHumidity() const
    {
        return (settings & 0b00000001) != 0;
    }

    /**
     * @brief Sets or clears the humidity flag for this sensor.
     *
     * Only bit 0 of the settings byte is modified; all other bits are preserved.
     *
     * @param hasHumidity `true` to mark this sensor as having humidity,
     *                    `false` to mark it as temperature-only.
     */
    void setHasHumidity(bool hasHumidity)
    {
        if (hasHumidity)
            settings |= 0b00000001;
        else
            settings &= ~0b00000001;
    }

    // -------------------------------------------------------------------------
    // Channel number — bits 1–2 of the settings byte
    // -------------------------------------------------------------------------

    /**
     * @brief Returns the RF channel number for this sensor (0–3).
     *
     * 433 MHz weather sensors can transmit on one of up to four sub-channels to
     * allow multiple sensors of the same brand to coexist. The channel is stored
     * in bits 1 and 2 of the settings byte.
     *
     * @return Channel number in the range 0–3.
     */
    uint8_t getChannel() const
    {
        return (settings & 0b00000110) >> 1;
    }

    /**
     * @brief Sets the RF channel number for this sensor.
     *
     * Only bits 1–2 of the settings byte are modified; all other bits (including
     * the humidity flag) are preserved. Values above 3 are masked to the lower
     * 2 bits automatically.
     *
     * @param channel Channel number (0–3). Values outside this range are clamped
     *                by masking with 0b11.
     */
    void setChannel(uint8_t channel)
    {
        settings &= ~0b00000110;
        settings |= ((channel & 0b00000011) << 1) & 0b00000110;
    }

    // -------------------------------------------------------------------------
    // Raw settings byte
    // -------------------------------------------------------------------------

    /**
     * @brief Returns the raw settings byte.
     *
     * Useful for serialization (e.g. saving to / reading from a JSON API)
     * when the entire byte must be transferred at once. Prefer the typed
     * accessors (`hasHumidity`, `getChannel`, etc.) for normal use.
     *
     * @return The raw uint8_t settings byte.
     */
    uint8_t getSettings() const
    {
        return settings;
    }

    /**
     * @brief Overwrites the entire settings byte.
     *
     * Use with care — this replaces the humidity flag, channel bits, and any
     * reserved bits in one operation. Prefer `setHasHumidity()` and
     * `setChannel()` to update individual fields safely.
     *
     * @param s The new raw settings byte value.
     */
    void setSettings(uint8_t s)
    {
        settings = s;
    }

    // -------------------------------------------------------------------------
    // Temperature correction — stored in steps of 0.1 °C (int8_t)
    // -------------------------------------------------------------------------

    /**
     * @brief Returns the temperature calibration offset in degrees Celsius.
     *
     * The correction is stored internally as a signed integer in steps of 0.1 °C
     * (e.g. raw value 5 = +0.5 °C, raw value -3 = −0.3 °C). This getter divides
     * by 10 to return the physical value.
     *
     * @return Correction offset in °C (e.g. 0.5, -0.3). Resolution: 0.1 °C.
     */
    float getCorrection() const
    {
        return correction / 10.0f;
    }

    /**
     * @brief Sets the temperature calibration offset in degrees Celsius.
     *
     * The value is multiplied by 10 and truncated to int8_t for storage.
     * Valid input range is −12.8 °C to +12.7 °C (storage limits of int8_t).
     *
     * @param value Correction offset in °C (e.g. pass 0.5 to add +0.5 °C).
     */
    void setCorrection(float value)
    {
        correction = static_cast<int8_t>(value * 10);
    }

    // -------------------------------------------------------------------------
    // Max valid delta — stored in steps of 0.1 °C (uint8_t)
    // -------------------------------------------------------------------------

    /**
     * @brief Returns the maximum allowed temperature jump between readings, in °C.
     *
     * During baseline accumulation, any new reading that differs from the
     * previous valid reading by more than this value is rejected as an outlier.
     * Stored as uint8_t in steps of 0.1 °C; this getter divides by 10.
     *
     * @return Max valid delta in °C (e.g. 2.5 for a raw value of 25).
     */
    float getMaxValidDelta() const
    {
        return maxValidDelta / 10.0f;
    }

    /**
     * @brief Sets the maximum allowed temperature jump between readings, in °C.
     *
     * The value is multiplied by 10 and stored as uint8_t. Valid input range is
     * 0.0–25.5 °C.
     *
     * @param value Max delta threshold in °C (e.g. pass 2.5 to store 25).
     */
    void setMaxValidDelta(float value)
    {
        maxValidDelta = static_cast<uint8_t>(value * 10);
    }

    // -------------------------------------------------------------------------
    // Max valid interval — stored in minutes (uint8_t)
    // -------------------------------------------------------------------------

    /**
     * @brief Returns the sensor timeout interval in milliseconds.
     *
     * If no valid reading has been received within this interval the sensor is
     * considered timed out. Stored internally as uint8_t (minutes); this getter
     * converts to milliseconds for direct comparison with `millis()`.
     *
     * @return Timeout threshold in milliseconds.
     *         Example: a stored value of 15 returns 900 000 ms (15 min).
     */
    unsigned long getMaxValidIntervalRaw() const
    {
        return maxValidInterval * 60UL * 1000UL;
    }

    /**
     * @brief Returns the raw stored timeout interval in minutes.
     *
     * Useful for JSON APIs and EEPROM serialization that work with minute values
     * directly, without millisecond conversion.
     *
     * @return Raw uint8_t value in minutes (0–255).
     */
    uint8_t getMaxValidInterval() const
    {
        return maxValidInterval;
    }

    /**
     * @brief Sets the sensor timeout interval directly in minutes.
     *
     * No unit conversion is applied.
     *
     * @param minutes Timeout threshold in minutes (0–255).
     */
    void setMaxValidInterval(uint8_t minutes)
    {
        maxValidInterval = minutes;
    }

    /**
     * @brief Prints a human readable name of this device to the specified buffer.
     * 
     */
    String getLogName() {
    return String(String(code) 
        + " (0x" + (deviceID < 0x10 ? "0" : "") 
        + String(deviceID, HEX) + "|" + String(getChannel()) + ")");
    }
    

private:
    /**
     * Packed settings byte.
     * - Bit 0       : hasHumidity flag
     * - Bits 1–2    : channel number (0–3)
     * - Bits 3–7    : reserved
     */
    uint8_t settings;

    /** Temperature calibration offset stored in steps of 0.1 °C. */
    int8_t correction;

    /** max valid temperature jump between readings stored in steps of 0.1 °C. */
    uint8_t maxValidDelta;

    /** Sensor timeout threshold stored in minutes. */
    uint8_t maxValidInterval;
};

#endif // _THSensorConfig_h