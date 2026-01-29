#ifdef MOTOGP
#pragma once

#include <FastLED.h>
#include <AnalogIO.h>

#define NUM_LEDS 21
#define WHEEL_LITE_DATA_PIN 5
#define BLOWER_PIN 9
#define BRAKE_LIGHT_PIN 10
#define MAX_BLOWER_PWM 255
#define MAX_LIGHT_PWM 255
#define TRAK_LIGHT_DELAY 250

class Lighting {
public:
  Lighting();
  void update(float accelPct, float brakePct);
  void incrementWheelLed();
  void turnOffAllWheelLites();
  void processWheelLites(long time, float accelPct);
  void Lighting::processBrake(float brakePct);
  void processBlower(float accelPct);
private:
  CRGB leds[NUM_LEDS];
  uint8_t wheelLed;
  long timeUntilNextWheelLite;
  AnalogOut *blower;
  AnalogOut *brakeLed;
};
#endif