#include "wheel.h"

Wheel_::Wheel_(void) {
  static HID_SubDescriptor node(wheelHIDDescriptor, sizeof(wheelHIDDescriptor));
  NEW_HID().AppendDescriptor(&node);

  ffbEngine.SetFfb(&NEW_HID().ffbReportHandler);

  axisWheel = new AxisWheel();

  analogAxes[AXIS_ACC] = new Axis(MA_LEVEL_AXIS_ACC);
  analogAxes[AXIS_BRAKE] = new Axis(MA_LEVEL_AXIS_BRAKE);
  analogAxes[AXIS_CLUTCH] = new Axis(MA_LEVEL_AXIS_CLUTCH);
  analogAxes[AXIS_AUX1] = new Axis(MA_LEVEL_AXIS_AUX1);
  analogAxes[AXIS_AUX2] = new Axis(MA_LEVEL_AXIS_AUX2);
  analogAxes[AXIS_AUX3] = new Axis(MA_LEVEL_AXIS_AUX3);
  analogAxes[AXIS_AUX4] = new Axis(MA_LEVEL_AXIS_AUX4);
  analogAxes[AXIS_ST_ANALOG] = new Axis(MA_LEVEL_AXIS_ST_ANALOG);
}

void Wheel_::update(void) {

  wheelData data;

#if STEER_TYPE == ST_ANALOG
  data.axes[0] = analogAxes[AXIS_ST_ANALOG]->value;
#else
  data.axes[0] = axisWheel->value;
#endif

  int8_t i;
  for (i = 0; i < AXIS_COUNT; i++) {
    if (!analogAxes[i]->outputDisabled)
      data.axes[i + 1] = analogAxes[i]->value;
  }

#ifdef HATSWITCH
  data.hat = getHatSwitch();
#endif

  data.buttons = buttons;

  NEW_HID().RecvFfbReport();
  NEW_HID().SendReport(0x01, &data, sizeof(data));

  if (USB_GUI_Report.command) {
    NEW_HID().SendReport(16, &USB_GUI_Report, sizeof(USB_GUI_Report));
    USB_GUI_Report.command = 0;
  }
}

//Hatswitch
#define HAT_UP 0b00000000          //(0)
#define HAT_UP_RIGHT 0b00000001    //(1)
#define HAT_RIGHT 0b00000010       //(2)
#define HAT_DOWN_RIGHT 0b00000011  //(3)
#define HAT_DOWN 0b00000100        //(4)
#define HAT_DOWN_LEFT 0b00000101   //(5)
#define HAT_LEFT 0b00000110        //(6)
#define HAT_UP_LEFT 0b00000111     //(7)
#define HAT_CENTER 0b00001000      //(8)

uint8_t Wheel_::getHatSwitch() {
  uint8_t hat = 0;

  bitWrite(hat, 0, bitRead(buttons, HAT_BTN_UP - 1));
  bitWrite(hat, 1, bitRead(buttons, HAT_BTN_DOWN - 1));
  bitWrite(hat, 2, bitRead(buttons, HAT_BTN_LEFT - 1));
  bitWrite(hat, 3, bitRead(buttons, HAT_BTN_RIGHT - 1));

#ifdef HAT_CLR_BTNS
  bitClear(buttons, HAT_BTN_UP - 1);
  bitClear(buttons, HAT_BTN_DOWN - 1);
  bitClear(buttons, HAT_BTN_LEFT - 1);
  bitClear(buttons, HAT_BTN_RIGHT - 1);
#endif

  switch (hat) {
    case 0b00000001:
      hat = HAT_UP;
      break;
    case 0b00000010:
      hat = HAT_DOWN;
      break;
    case 0b00000100:
      hat = HAT_LEFT;
      break;
    case 0b00001000:
      hat = HAT_RIGHT;
      break;
    case 0b00000101:
      hat = HAT_UP_LEFT;
      break;
    case 0b00001001:
      hat = HAT_UP_RIGHT;
      break;
    case 0b00000110:
      hat = HAT_DOWN_LEFT;
      break;
    case 0b00001010:
      hat = HAT_DOWN_RIGHT;
      break;
    default:
      hat = HAT_CENTER;
  };

  return hat;
}
