#include "Lighting.h"
#ifdef LIGHTING
extern CRGB leds[];

Lighting::Lighting(void) {
  FastLED.addLeds<WS2811, WHEEL_LITE_DATA_PIN, BRG>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
}

void Lighting::update(float accelPct) {
  processWheelLites(millis(), accelPct);
}

void Lighting::incrementWheelLed() {
  ++wheelLed;
  if (wheelLed >= NUM_LEDS) {
    wheelLed = 0;
  }
}

void Lighting::turnOffAllWheelLites() {
  FastLED.clear();  // clear all pixel data
  FastLED.show();
}

void Lighting::processWheelLites(long time, float accelPct) {
  if (accelPct > 0.03) {
    if (time > timeUntilNextWheelLite) {
      leds[wheelLed] = CRGB::Blue;
      if (wheelLed > 0) {
        leds[wheelLed - 1] = CRGB::Black;
      } else if (wheelLed == 0) {
        leds[NUM_LEDS - 1] = CRGB::Black;
      }
      FastLED.show();
      timeUntilNextWheelLite = time + ((1 - accelPct) * 30);
      incrementWheelLed();
    }
  } else {
    turnOffAllWheelLites();
  }
}
#endif
