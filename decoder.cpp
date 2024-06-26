#include "decoder.h"

volatile unsigned long lastInt = 0;                    /* the most recent interrupt time */
volatile unsigned long currInt;                        /* current interrupt time */
unsigned long currMicros;                              /* current time for main loop */
volatile unsigned long timeInt1 = 0, timeInt2 = 0;     /* These variables store the last two pulse timings */
volatile bool keepWaiting = true;                      /* this is reset when the signal is idle and we can start processing sampled data */
volatile unsigned long lastSync = 0;                   /* last time a sync frame got received */
bool rawBits[PACKETS_PER_STREAM][BITS_PER_PACKET + 1]; /* two dimension array for raw bitstream; number of rows and columns is set for expected signal */
                                                        /* there is one extra bit at the end which will be used as complete flag (i.e. if the row has been filled by exactly 36 bits) */
uint8_t i = 0, j = 0;                                  /* i is the column, j is the row */
Measurement lastMeasurement;                           /* Holds the last received Measurement */

void DecoderClass::begin(int pin) {
  _pin = pin;
  pinMode(_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(_pin), mapPulseTimingsToBits, CHANGE);
}

INTERRUPT_ROUTINE DecoderClass::mapPulseTimingsToBits() {
  currInt = micros();

  // Copy the second pulse timing to the first variable
  timeInt1 = timeInt2;

  // Update the second pulse timing with the last measured pulse
  timeInt2 = currInt - lastInt;

  // Update the most recent interrupt time
  lastInt = currInt;

  // if the first pulse timing fits the ON state
  if (timeInt1 < timeInt2 && timeInt1 > 400 && timeInt1 < 600) {
    // continue sampling the signal
    keepWaiting = true;

    // check if the pulse that followed matches bit 0
    if (timeInt2 > 900 && timeInt2 < 1100) {
      rawBits[j][i] = 0;

      if (i < BITS_PER_PACKET)
        i += 1;
      else rawBits[j][BITS_PER_PACKET] = 0;  // more than expected bits received; set error flag

      // DEBUG: Serial.print("0");
    }

    // otherwise if it maches bit 1
    else if (timeInt2 > 1900 && timeInt2 < 2100) {
      rawBits[j][i] = 1;

      if (i < BITS_PER_PACKET)
        i += 1;
      else rawBits[j][BITS_PER_PACKET] = 0;  // more than expected bits received; set error flag

      // DEBUG: Serial.print("1");
    }

    // if it is longer than that is probably a sync pulse
    else if (timeInt2 < 4500) {

      // if less than 36 bits received, set the flag to false
      if (i < (BITS_PER_PACKET - 1)) {
        rawBits[j][BITS_PER_PACKET] = 0;
      } else {
        rawBits[j][BITS_PER_PACKET] = 1;
      }

      i = 0;  // reset column

      // start filling the next row; ignore extra streams
      if (j < (PACKETS_PER_STREAM - 1))
        j += 1;

      lastSync = currInt;

      // DEBUG: Serial.println();
    }
  }
}


bool DecoderClass::hasMeasurement() {
  bool result = false;
  currMicros = micros();

  // if no sync frame received within 100 ms (assuming a 36-bit packet of "1" will be sent in 95 ms)
  if ((currMicros > lastSync + 100000) && (keepWaiting == true)) {
    // disable interrrupt for a while
    detachInterrupt(digitalPinToInterrupt(_pin));

    yield();

    // disable sampling pulses and process existing data
    keepWaiting = false;

    // clear last timeframes
    timeInt1 = 0;
    timeInt2 = 0;

    // if at least one packet received
    if (j > 0) result = processRawBitstream();

    j = 0;
    i = 0;  // reset indexes

    // reattach interrupt
    attachInterrupt(digitalPinToInterrupt(_pin), mapPulseTimingsToBits, CHANGE);
    return result;
  }
}

bool DecoderClass::processRawBitstream() {
  bool finalBits[BITS_PER_PACKET] = { 0 };
  int8_t bitLowCnt = 0, bitHighCnt = 0;

  for (int8_t b = 0; b < BITS_PER_PACKET; b++) {
    for (int8_t a = 0; a < PACKETS_PER_STREAM; a++) {
      // only if packet is complete
      if (rawBits[a][BITS_PER_PACKET] == 1) {

        // take each bit in the same position over all packets
        // in theory all should be identical
        // assuming poor signal strength, go with the majority
        // by counting packets with each of the two bit states
        if (rawBits[a][b] == 1) bitHighCnt += 1;
        else bitLowCnt += 1;

        if (bitHighCnt >= bitLowCnt) finalBits[b] = 1;
        else finalBits[b] = 0;

        bitLowCnt = 0;
        bitHighCnt = 0;
      }
    }
  }

  // clear raw bit stream
  for (int8_t i = 0; i < PACKETS_PER_STREAM; i++)
    for (int8_t j = 0; j < BITS_PER_PACKET; j++)
      rawBits[i][j] = 0;

  // another sanity check
  if (finalBits[24] && finalBits[25] && finalBits[26] && finalBits[27]) {

    // get device ID
    uint8_t deviceID = 0;
    for (int8_t i = 0; i < 8; i++) {
      if (finalBits[i] == 1) deviceID |= 1 << (7 - i);
    }

    // get battery state
    bool batteryState = finalBits[8];

    // get channel number
    uint8_t channelNo = (int8_t(finalBits[10]) << 1) | int8_t(finalBits[11]) + 1;

    // get temperature
    int16_t t12 = 0;
    for (int8_t i = 0; i < 12; i++) {
      if (finalBits[i + 12] == 1) t12 |= 1 << (11 - i);
    }
    if (t12 > 2048) t12 = t12 - 4096;
    float temperature = t12 / 10.0;
    //temperature = (t12 >> 4) * 0.1f;

    // get humidity
    uint8_t humidity = 0;
    for (int8_t i = 0; i < 8; i++) {
      if (finalBits[i + 28] == 1) humidity |= 1 << (7 - i);
    }

    // valid data received
    if (deviceID != 0) {
      // process received data
      lastMeasurement = {millis(), deviceID, batteryState, channelNo, temperature, humidity};
      return true;
    }
  }
  return false;
}

Measurement DecoderClass::getMeasurement() {
  return lastMeasurement;
}


