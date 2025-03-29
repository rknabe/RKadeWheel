#include "axis.h"

Axis::Axis(uint8_t smoothLevel) {
  filter.set_window(smoothLevel);
}

void Axis::setValue(int32_t rawValue_) {
  rawValue = rawValue_;

  if (autoLimit) {
    if (rawValue < axisMin)
      setLimits(rawValue, axisMax, true);
    if (rawValue > axisMax)
      setLimits(axisMin, rawValue, true);
  }

  value = constrain(rawValue, axisMin, axisMax);
  value = round(filter.add(value));

  if (bitTrim) {
    if (value >= 0)
      value = value >> bitTrim << bitTrim;
    else
      value = -(-value >> bitTrim << bitTrim);
  }

  if (value < axisCenterN)
    value = (value - axisCenterN) * rangeFactorNeg;
  else if (value > axisCenterP)
    value = (value - axisCenterP) * rangeFactorPos;
  else
    value = 0;
}

int16_t Axis::getCenter() {
  return ((int32_t)axisCenterN + axisCenterP) >> 1;
}

int16_t Axis::getDZ() {
  return (axisCenterP - axisCenterN) >> 1;
}

void Axis::setCenter(int16_t center) {
  autoCenter = false;
  int16_t dz = getDZ();
  axisCenterN = center - dz;
  axisCenterP = center + dz;
  updateRangeFactor();
}

void Axis::setDZ(int16_t dz) {
  autoCenter = false;
  int16_t center = getCenter();
  axisCenterN = center - dz;
  axisCenterP = center + dz;
  updateRangeFactor();
}

void Axis::setLimits(int16_t _min, int16_t _max, bool _auto) {
  axisMin = _min;
  axisMax = _max;

  if (!_auto)
    autoLimit = false;

  updateRangeFactor();
}

void Axis::setAutoLimits(bool _auto) {
  if (_auto) {
    axisMin = rawValue;
    axisMax = rawValue;
    updateRangeFactor();
  }
  autoLimit = _auto;
}

void Axis::updateRangeFactor() {
  if ((axisCenterN < axisMin) || (axisCenterP > axisMax))
    autoCenter = true;

  if (autoCenter) {
    axisCenterN = axisCenterP = (axisMin + axisMax) >> 1;
  }

  rangeFactorNeg = 32767.0 / (axisCenterN - axisMin);
  rangeFactorPos = 32767.0 / (axisMax - axisCenterP);
}


/***************************************** AxisWheel ******************************************************/

AxisWheel::AxisWheel()
  : Axis(MA_LEVEL_AXIS_ST_ANALOG) {
  filterVelocity.set_window(MA_LEVEL_WHEEL_VELOCITY);
  filterAcceleration.set_window(MA_LEVEL_WHEEL_ACCELERATION);
}

void AxisWheel::setValue(int32_t rawValue_) {
  rawValue_ = rawValue_ - 511;
  if (invertRotation == true) {
    rawValue_ = -1 * rawValue_;
  }
  rawValue_ = rawValue_ - getCenter();

  if (autoLimit) {
    if (rawValue_ < axisMin)
      setLimits(rawValue_, axisMax, true);
    if (rawValue_ > axisMax)
      setLimits(axisMin, rawValue_, true);
  }

#ifdef STEER_TM_RATIO_ENABLED
  rawValue = rawValue_ * ((float)STEER_TM_RATIO_MUL / (float)STEER_TM_RATIO_DIV);
#else
  rawValue = rawValue_;
#endif

  if (bitTrim) {
    if (value >= 0)
      value = value >> bitTrim << bitTrim;
    else
      value = -(-value >> bitTrim << bitTrim);
  }

  if (value < axisCenterN)
    value = (value - axisCenterN) * rangeFactorNeg;
  else if (value > axisCenterP)
    value = (value - axisCenterP) * rangeFactorPos;
  else
    value = 0;

  int32_t val = constrain(rawValue, axisMin, axisMax);
  value = val * rangeFactor;
  absValue = round(filter.add(value));
  int16_t tmpUs = micros();
  int16_t td = tmpUs - lastUs;
  lastUs = tmpUs;

  //Velocity and acceleration calculation for damper/friction/inertia effects
  //These parameters do not depend on wheel range
  int16_t newVelocity = round(filterVelocity.add(((absValue - lastPosition) << 15) / td));
  lastPosition = absValue;

  acceleration = round(filterAcceleration.add(((int32_t)(newVelocity - velocity) << 15) / td));
  velocity = newVelocity;
}

void AxisWheel::setRange(uint16_t _deg) {
  range = _deg;
  rangeFactor = (360.0 / (float)range) * 48;
}

void AxisWheel::setCenterZero() {
  value = 0;
  lastPosition = 0;
  velocity = 0;
  acceleration = 0;
}