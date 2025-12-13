//#ifdef MOTOGP
#include "Lighting.h"

Lighting::Lighting(void) {
  FastLED.addLeds<WS2811, WHEEL_LITE_DATA_PIN, BRG>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  blower = new AnalogOut(BLOWER_PIN);
  brakeLed = new AnalogOut(BRAKE_LIGHT_PIN);
}

void Lighting::update(float accelPct, float brakePct) {
  processWheelLites(millis(), accelPct);
  processBlower(accelPct);
  processBrake(brakePct);
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

void Lighting::processBrake(float brakePct) {
  int16_t pwmVal = brakePct * MAX_LIGHT_PWM;
  if (pwmVal < 0) {
    pwmVal = 0;
  } else if (pwmVal > MAX_LIGHT_PWM) {
    pwmVal = MAX_LIGHT_PWM;
  }
  brakeLed->write(pwmVal);
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
//#endif