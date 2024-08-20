#define DOCTEST_CONFIG_IMPLEMENT  // REQUIRED: Enable custom main()
#include <doctest.h>

#include "THDeviceManager.h"
#include "THMeasurement.h"

TEST_CASE("testing the printed name function") {
  char name[] = "Buiten slk";
  THDevice *uut = new THDevice(0x1D, 1, name, 0);
  char out[64];
  uut->printName(out);
  CHECK(strcmp(out, "Buiten slk (0x1D)") == 0);
}

TEST_CASE("Testing the process method") {
  char name[] = "Buiten slk";
  THDevice *uut = new THDevice(0x1D, 1, name, 0);
  Measurement m = {
    0x1D, //uint8_t deviceID;
    false, //bool batteryState;
    0, //uint8_t channelNo;
    11, //float temperature;
    77 //uint8_t humidity;
  };
  uut->process(m);
  CHECK(uut->hasUpdates() == true);
}


int main(int argc, char **argv)
{
  doctest::Context context;

  // BEGIN:: PLATFORMIO REQUIRED OPTIONS
  context.setOption("success", true);     // Report successful tests
  context.setOption("no-exitcode", true); // Do not return non-zero code on failed test case
  // END:: PLATFORMIO REQUIRED OPTIONS

  // YOUR CUSTOM DOCTEST OPTIONS

  context.applyCommandLine(argc, argv);
  return context.run();
}