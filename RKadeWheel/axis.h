#pragma once
#include <Arduino.h>
#include "config.h"
#include <Smooth.h>

/*
 * Analog axes
 */
class Axis {
public:
  int16_t rawValue;
  int16_t value = 0;
  bool autoLimit = false;
  bool autoCenter = true;
  int16_t axisMin;
  int16_t axisMax;
  int16_t axisCenterN;
  int16_t axisCenterP;

  bool outputDisabled = false;
  int8_t bitTrim = 0;

  Axis(uint8_t smoothLevel);
  void setValue(int32_t rawValue_);
  void setCenter(int16_t center);
  void setDZ(int16_t dz);
  void setLimits(int16_t _min, int16_t _max, bool _auto = false);
  void setAutoLimits(bool _auto);
  void updateRangeFactor();
  Smooth filter;

  int16_t getCenter();
  int16_t getDZ();
  float rangeFactorNeg, rangeFactorPos;
};

/*
 * Steering axis
 */
class AxisWheel : public Axis {
public:
  int32_t absValue;  //=raw constrained and smoothed
  int16_t range;
  bool invertRotation = false;
  int32_t lastPosition;
  int16_t velocity;
  int16_t acceleration;
  uint16_t lastUs;

  Smooth filterVelocity;
  Smooth filterAcceleration;

  AxisWheel();
  void setValue(int32_t rawValue_);
  void setRange(uint16_t _deg);
  void setCenterZero();
private:
  float rangeFactor;
};
