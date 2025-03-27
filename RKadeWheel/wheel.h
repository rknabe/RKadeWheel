#pragma once

#include "axis.h"
#include "FfbEngine.h"
#include "WHID.h"
#include "hidDescriptor.h"
#include "settings.h"

//Input Report
typedef struct {
  int16_t axes[AXIS_COUNT] = { -32768};
  uint32_t buttons;
} wheelData;

//Reports for GUI
typedef struct {
  uint8_t command;
  int16_t arg;
  uint8_t data[28];
} GUI_Report;

typedef struct {
  char id[12] = FIRMWARE_TYPE;
  char ver[12];
} GUI_Report_Version;

typedef struct {
  int32_t rawValue;
  int16_t value;
  int16_t range;
  int16_t velocity;
  int16_t acceleration;
  int16_t axisMin;
  int16_t axisMax;
  int16_t center;
  int16_t deadzone;
  uint8_t autoLimit;
  uint8_t bitTrim;
  uint8_t invertRotation;
} GUI_Report_SteerAxis;

typedef struct {
  int16_t rawValue;
  int16_t value;
  int16_t axisMin;
  int16_t axisMax;
  int16_t center;
  int16_t deadzone;
  uint8_t autoLimit;
  uint8_t hasCenter;
  uint8_t outputDisabled;
  uint8_t bitTrim;
} GUI_Report_AnalogAxis;

typedef struct {
  int32_t buttons;
  int8_t shiftButton;
  int8_t debounce;
  uint8_t mplexShifter;
} GUI_Report_Buttons;

typedef struct {
  uint16_t maxvd;
  uint16_t maxvf;
  uint16_t maxacc;
  uint16_t minForce;
  uint16_t maxForce;
  uint16_t cutForce;
  uint8_t ffbBD;
} GUI_Report_Settings;

class Wheel_ {
public:
  AxisWheel* axisWheel;
  Axis* analogAxes[AXIS_COUNT];
  uint32_t buttons;
  Wheel_();
  void update();
  FfbEngine ffbEngine;

  GUI_Report USB_GUI_Report;
private:
};
