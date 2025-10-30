//#ifdef MOTOGP
#include "Lighting.h"
#include "config.h"
#include "wheel.h"

extern Wheel wheel;

Lighting::Lighting(void) {
  FastLED.addLeds<WS2811, WHEEL_LITE_DATA_PIN, BRG>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  blower = new AnalogOut(BLOWER_PIN);
  brakeLed = new AnalogOut(BRAKE_LIGHT_PIN);
}

void Lighting::update(float accelPct) {
  processWheelLites(millis(), accelPct);
  processBlower(accelPct);
  if ((wheel.buttons & (uint32_t)pow(2, BTN_BRAKE_INDEX)) != 0) {
    brakeLed->write(MAX_LIGHT_PWM);
  } else {
    brakeLed->write(0);
  }
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

void Lighting::processBlower(float accelPct) {
  int16_t pwmValBlower = accelPct * MAX_BLOWER_PWM;
  if (pwmValBlower < 0) {
    pwmValBlower = 0;
  } else if (pwmValBlower > MAX_BLOWER_PWM) {
    pwmValBlower = MAX_BLOWER_PWM;
  }
  blower->write(pwmValBlower);
}
//#endif