#include "wheel.h"

Wheel_::Wheel_(void) {
  static HID_SubDescriptor node(wheelHIDDescriptor, sizeof(wheelHIDDescriptor));
  NEW_HID().AppendDescriptor(&node);

  ffbEngine.SetFfb(&NEW_HID().ffbReportHandler);

  axisWheel = new AxisWheel();

  for (int i = 0; i < AXIS_COUNT; i++) {
    analogAxes[i] = new Axis(MA_LEVEL_AXIS);
  }
}

void Wheel_::update(void) {

  wheelData data;
  data.axes[0] = axisWheel->value;

  for (int i = 0; i < AXIS_REPORT_COUNT; i++) {
    if (i < AXIS_COUNT && !analogAxes[i]->outputDisabled) {
      data.axes[i + 1] = analogAxes[i]->value;
    }
  }

  data.buttons = buttons;

  NEW_HID().RecvFfbReport();
  NEW_HID().SendReport(0x01, &data, sizeof(data));

  if (USB_GUI_Report.command) {
    NEW_HID().SendReport(16, &USB_GUI_Report, sizeof(USB_GUI_Report));
    USB_GUI_Report.command = 0;
  }
}
