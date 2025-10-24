#pragma once
#ifdef LIGHTING
#include <FastLED.h>
#include "config.h"

#define NUM_LEDS 21
#define WHEEL_LITE_DATA_PIN 5

class Lighting {
public:
  Lighting();
  void update(float accelPct);
  void incrementWheelLed();
  void turnOffAllWheelLites();
  void processWheelLites(long time, float accelPct);
private:
  uint8_t wheelLed;
  long timeUntilNextWheelLite;
};
#endif
