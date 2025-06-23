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
  analogAxes[AXIS_AUX5] = new Axis(MA_LEVEL_AXIS_AUX5);
}

void Wheel_::update(void) {

  wheelData data;
  data.axes[0] = axisWheel->value;

  int8_t i;
  for (i = 0; i < AXIS_REPORT_COUNT; i++) {
    if (!analogAxes[i]->outputDisabled)
      data.axes[i + 1] = analogAxes[i]->value;
  }

  data.buttons = buttons;

  NEW_HID().RecvFfbReport();
  NEW_HID().SendReport(0x01, &data, sizeof(data));

  if (USB_GUI_Report.command) {
    NEW_HID().SendReport(16, &USB_GUI_Report, sizeof(USB_GUI_Report));
    USB_GUI_Report.command = 0;
  }
}
