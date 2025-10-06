#pragma once
#include <Arduino.h>

#pragma once
#ifdef NO_KEYPAD
#define FIRMWARE_VER "2.0.4-DXN"
#else
#define FIRMWARE_VER "2.0.4-DX"
#endif
#define FIRMWARE_TYPE "RKADE"

#define AXIS_ACC 0
#define AXIS_BRAKE 1
#define AXIS_CLUTCH 2
#define AXIS_AUX1 3
#define AXIS_AUX2 4
#define AXIS_AUX3 5
#define AXIS_AUX4 6
#define AXIS_AUX5 7
#define AXIS_COUNT 5
#define AXIS_REPORT_COUNT 8

#define GAIN_COUNT 12
#define GAIN_SPRING 8

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
  int16_t gain[GAIN_COUNT];

  uint8_t shiftButton;
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

  //wheel settings
  uint16_t range;
  int16_t axisMin;
  int16_t axisMax;
  int16_t axisCenter;
  int16_t axisDZ;
  int8_t axisBitTrim;
  int8_t invertRotation;

  uint8_t ffbBD;

  int16_t maxVelocityDamper;
  int16_t maxVelocityFriction;
  int16_t maxAcceleration;

  uint8_t checksum;

  //void print();

  uint8_t calcChecksum();
};
