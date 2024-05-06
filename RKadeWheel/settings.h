#pragma once
#include <Arduino.h>

#pragma once

#define FIRMWARE_VER "1.1.3"
#define FIRMWARE_TYPE "RKADE"

#define AXIS_ACC 0
#define AXIS_BRAKE 1
#define AXIS_CLUTCH 2
#define AXIS_AUX1 3
#define AXIS_AUX2 4
#define AXIS_AUX3 5
#define AXIS_AUX4 6
#define AXIS_ST_ANALOG 7
#define AXIS_COUNT 8

struct SettingsAxis {
  int16_t axisMin;
  int16_t axisMax;
  int16_t axisCenter;
  int16_t axisDZ;
  int8_t axisOutputDisabled;
  int8_t axisBitTrim;
};

//settings in global variables
struct SettingsData {
  int16_t gain[13];

  int8_t centerButton;
  uint8_t debounce;

  int16_t minForce;
  int16_t maxForce;
  int16_t cutForce;

  int16_t endstopOffset;
  int16_t endstopWidth;
  uint8_t constantSpring;
  uint8_t afcOnStartup;
  uint8_t mplexShifter;
};

//all settings
class SettingsEEPROM {
public:
  SettingsAxis axes[AXIS_COUNT];
  SettingsData data;

  uint16_t range;
  uint8_t ffbBD;

  int16_t maxVelocityDamper;
  int16_t maxVelocityFriction;
  int16_t maxAcceleration;

  uint8_t checksum;

  void print();

  uint8_t calcChecksum();
};
