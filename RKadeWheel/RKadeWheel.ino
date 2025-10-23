#include <EEPROM.h>
#include "config.h"
#include "wheel.h"
#include "motor.h"
#include "settings.h"
#include <HID-Project.h>

//global variables
Wheel_ wheel;
Motor motor;
SettingsData settings;
uint32_t tempButtons;
uint8_t debounceCount = 0;
static const uint8_t dpb[] = { DPB_PINS };

void load(bool defaults = false);

//-------------------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(50);

  for (uint8_t i = 0; i < sizeof(dpb); i++) {
    pinMode(dpb[i], INPUT_PULLUP);
  }

  //load settings
  load();

  //motor setup
  motor.begin();

  //this helps firmware update
  delay(1000);
}

void loop() {

  readAnalogAxes();
  readButtons();
  processUsbCmd();
  wheel.update();
  processFFB();
  processSerial();

  delay(10);
}

void send(char keycode) {
  Keyboard.press(KeyboardKeycode(keycode));
  delay(40);
  Keyboard.release(KeyboardKeycode(keycode));
}

//Processing endstop and force feedback
void processFFB() {

  wheel.ffbEngine.constantSpringForce();

  int16_t force = wheel.ffbEngine.calculateForce(wheel.axisWheel);

  force = applyForceLimit(force);
  motor.setForce(force * FORCE_RATIO_MUL);
}

//scaling force to minForce & maxForce and cut at cutForce
int16_t applyForceLimit(int16_t force) {
  if (force == 0)
    return 0;

  if ((settings.minForce != 0) || (settings.maxForce < 16383)) {
    if (abs(force) < 1024)  //slope
    {
      int32_t v = (((settings.maxForce - settings.minForce)) >> 4) + settings.minForce;
      force = (force * v) >> 10;
    } else
      force = (int16_t)((int32_t)force * (settings.maxForce - settings.minForce) >> 14) + sign(force) * settings.minForce;
  }

  if (settings.cutForce >= 16383)
    return force;
  else
    return constrain(force, -settings.cutForce, settings.cutForce);
}

/*
  communicating with GUI:
*/
void processUsbCmd() {
  USB_GUI_Command *usbCmd = &wheel.ffbEngine.ffbReportHandler->usbCommand;

  //clear output report
  memset(&wheel.USB_GUI_Report, 0, sizeof(wheel.USB_GUI_Report));

  void *data = wheel.USB_GUI_Report.data;
  if (usbCmd->command) {
    //return data only for read commands
    if (usbCmd->command < 10) {
      wheel.USB_GUI_Report.command = usbCmd->command;
      wheel.USB_GUI_Report.arg = usbCmd->arg[0];
    }

    switch (usbCmd->command) {
      //get data
      case 1:  //return string model+version
        strcpy_P(((GUI_Report_Version *)data)->id, PSTR(FIRMWARE_TYPE));
        strcpy_P(((GUI_Report_Version *)data)->ver, PSTR(FIRMWARE_VER));
        break;
      case 2:  //return steering axis data
        ((GUI_Report_SteerAxis *)data)->rawValue = wheel.axisWheel->rawValue;
        ((GUI_Report_SteerAxis *)data)->value = wheel.axisWheel->value;
        ((GUI_Report_SteerAxis *)data)->range = wheel.axisWheel->range;
        ((GUI_Report_SteerAxis *)data)->velocity = wheel.axisWheel->velocity;
        ((GUI_Report_SteerAxis *)data)->acceleration = wheel.axisWheel->acceleration;

        ((GUI_Report_SteerAxis *)data)->axisMin = wheel.axisWheel->axisMin;
        ((GUI_Report_SteerAxis *)data)->axisMax = wheel.axisWheel->axisMax;
        ((GUI_Report_SteerAxis *)data)->center = wheel.axisWheel->getCenter();
        ((GUI_Report_SteerAxis *)data)->deadzone = wheel.axisWheel->getDZ();
        ((GUI_Report_SteerAxis *)data)->autoLimit = wheel.axisWheel->autoLimit;
        ((GUI_Report_SteerAxis *)data)->bitTrim = wheel.axisWheel->bitTrim;
        ((GUI_Report_SteerAxis *)data)->invertRotation = wheel.axisWheel->invertRotation;
        break;
      case 3:  //return analog axis data
        ((GUI_Report_AnalogAxis *)data)->rawValue = wheel.analogAxes[usbCmd->arg[0]]->rawValue;
        ((GUI_Report_AnalogAxis *)data)->value = wheel.analogAxes[usbCmd->arg[0]]->value;
        ((GUI_Report_AnalogAxis *)data)->axisMin = wheel.analogAxes[usbCmd->arg[0]]->axisMin;
        ((GUI_Report_AnalogAxis *)data)->axisMax = wheel.analogAxes[usbCmd->arg[0]]->axisMax;
        ((GUI_Report_AnalogAxis *)data)->center = wheel.analogAxes[usbCmd->arg[0]]->getCenter();
        ((GUI_Report_AnalogAxis *)data)->deadzone = wheel.analogAxes[usbCmd->arg[0]]->getDZ();
        ((GUI_Report_AnalogAxis *)data)->autoLimit = wheel.analogAxes[usbCmd->arg[0]]->autoLimit;
        ((GUI_Report_AnalogAxis *)data)->hasCenter = !wheel.analogAxes[usbCmd->arg[0]]->autoCenter;

        ((GUI_Report_AnalogAxis *)data)->outputDisabled = wheel.analogAxes[usbCmd->arg[0]]->outputDisabled;
        ((GUI_Report_AnalogAxis *)data)->bitTrim = wheel.analogAxes[usbCmd->arg[0]]->bitTrim;
        break;
      case 4:  //return buttons data
        ((GUI_Report_Buttons *)data)->buttons = wheel.buttons;
        ((GUI_Report_Buttons *)data)->shiftButton = settings.shiftButton;
        ((GUI_Report_Buttons *)data)->debounce = settings.debounce;
        ((GUI_Report_Buttons *)data)->mplexShifter = settings.mplexShifter;
        ((GUI_Report_Buttons *)data)->btn11ActionKey = settings.btn11ActionKey;
        ((GUI_Report_Buttons *)data)->btn12ActionKey = settings.btn12ActionKey;
        ((GUI_Report_Buttons *)data)->btn13ActionKey = settings.btn13ActionKey;
        ((GUI_Report_Buttons *)data)->btn14ActionKey = settings.btn14ActionKey;
        break;
      case 5:  //return gains
        memcpy(data, settings.gain, sizeof(settings.gain));
        break;
      case 6:  //return remaining settings
        //GUI_Report_Settings* repSettings=(GUI_Report_Settings*)(wheel.USB_GUI_Report.data);

        ((GUI_Report_Settings *)data)->maxvd = round(16384.0 / wheel.ffbEngine.maxVelocityDamperC);
        ((GUI_Report_Settings *)data)->maxvf = round(16384.0 / wheel.ffbEngine.maxVelocityFrictionC);
        ((GUI_Report_Settings *)data)->maxacc = round(16384.0 / wheel.ffbEngine.maxAccelerationInertiaC);

        ((GUI_Report_Settings *)data)->minForce = settings.minForce;
        ((GUI_Report_Settings *)data)->maxForce = settings.maxForce;
        ((GUI_Report_Settings *)data)->cutForce = settings.cutForce;

        ((GUI_Report_Settings *)data)->ffbBD = motor.bitDepth;

        ((GUI_Report_Settings *)data)->constantSpring = settings.constantSpring;
        break;

      // set
      case 10:  //set range for steering axis
        wheel.axisWheel->setRange(usbCmd->arg[0]);
        break;
      case 11:  //set limits for analog axis
        wheel.analogAxes[usbCmd->arg[0]]->setLimits(usbCmd->arg[1], usbCmd->arg[2]);
        break;
      case 12:  //set center for analog axis
        wheel.analogAxes[usbCmd->arg[0]]->setCenter(usbCmd->arg[1]);
        break;
      case 13:  //set deadzone for analog axis
        wheel.analogAxes[usbCmd->arg[0]]->setDZ(usbCmd->arg[1]);
        break;
      case 14:  //set autolimits for analog axis
        wheel.analogAxes[usbCmd->arg[0]]->setAutoLimits(usbCmd->arg[1] > 0);
        break;
      case 15:  //set center button
        settings.shiftButton = usbCmd->arg[0];
        break;
      case 16:  //set debounce value
        settings.debounce = usbCmd->arg[0];
        break;
      case 17:  //set gain
        settings.gain[usbCmd->arg[0]] = usbCmd->arg[1];
        break;
      case 18:  //set misc settings
        switch (usbCmd->arg[0]) {
          case 0:
            wheel.ffbEngine.maxVelocityDamperC = 16384.0 / usbCmd->arg[1];
            break;
          case 1:
            wheel.ffbEngine.maxVelocityFrictionC = 16384.0 / usbCmd->arg[1];
            break;
          case 2:
            wheel.ffbEngine.maxAccelerationInertiaC = 16384.0 / usbCmd->arg[1];
            break;
          case 3:
            settings.minForce = usbCmd->arg[1];
            break;
          case 4:
            settings.maxForce = usbCmd->arg[1];
            break;
          case 5:
            settings.cutForce = usbCmd->arg[1];
            break;
          case 6:
            motor.setBitDepth(usbCmd->arg[1]);
            break;
          case 8:
            settings.constantSpring = usbCmd->arg[1];
            break;
          case 10:
            settings.mplexShifter = usbCmd->arg[1];
            break;
        }
        break;
      case 19:  //set outputDisabled and bittrim for analog axis
        wheel.analogAxes[usbCmd->arg[0]]->outputDisabled = (usbCmd->arg[1] > 0);
        wheel.analogAxes[usbCmd->arg[0]]->bitTrim = usbCmd->arg[2];
        break;
      //commands
      case 20:  //load settings from EEPROM
        load();
        break;
      case 21:  //save settings to EEPROM
        save();
        break;
      case 22:  //load defaults
        load(true);
        break;
      case 24:  //wheel limits
        wheel.axisWheel->setLimits(usbCmd->arg[0], usbCmd->arg[1]);
        break;
      case 25:  //set center for wheel
        wheel.axisWheel->setCenter(usbCmd->arg[0]);
        break;
      case 26:  //set deadzone for wheel
        wheel.axisWheel->setDZ(usbCmd->arg[0]);
        break;
      case 27:  //set autolimits for wheel
        wheel.axisWheel->setAutoLimits(usbCmd->arg[0] > 0);
        break;
      case 28:  //set trim for wheel
        wheel.axisWheel->bitTrim = usbCmd->arg[0];
        break;
      case 29:  //set inversion for wheel
        wheel.axisWheel->invertRotation = usbCmd->arg[0];
        break;
      case 30:
        switch (usbCmd->arg[0]) {
          case 11:
            settings.btn11ActionKey = usbCmd->arg[1];
            break;
          case 12:
            settings.btn12ActionKey = usbCmd->arg[1];
            break;
          case 13:
            settings.btn13ActionKey = usbCmd->arg[1];
            break;
          case 14:
            settings.btn14ActionKey = usbCmd->arg[1];
            break;
        }
        break;
    }
  }
  usbCmd->command = 0;
}

//------------------------- Reading all analog axes ----------------------------------
void readAnalogAxes() {

  wheel.analogAxes[AXIS_ACC]->setValue(analogRead(PIN_ACC));
  wheel.analogAxes[AXIS_BRAKE]->setValue(analogRead(PIN_BRAKE));
  wheel.analogAxes[AXIS_CLUTCH]->setValue(analogRead(PIN_CLUTCH));

#ifdef PIN_AUX1
  wheel.analogAxes[AXIS_AUX1]->setValue(analogRead(PIN_AUX1));
#endif
#ifdef PIN_AUX2
  wheel.analogAxes[AXIS_AUX2]->setValue(analogRead(PIN_AUX2));
#endif
#ifdef PIN_ST_ANALOG
  wheel.axisWheel->setValue(analogRead(PIN_ST_ANALOG));
#endif
}

//-----------------------------------end analog axes------------------------------

//-----------------------------------reading buttons------------------------------

void doButtonAction(ButtonAction action) {
  if (action == ButtonAction::NONE) {
    return;
  } else if (action == ButtonAction::ESC) {
    send(KEY_ESC);
  } else if (action == ButtonAction::SHUTDOWN) {
    System.write(SYSTEM_POWER_DOWN);
  } else if (action == ButtonAction::PAUSE) {
    send(KEY_PAUSE);
  }
}

void readButtons() {
  uint32_t buttons = 0;

  uint8_t *d;
  if (settings.debounce) {
    d = (uint8_t *)&buttons;
  } else {
    wheel.buttons = 0;
    d = (uint8_t *)&wheel.buttons;
  }

  uint8_t i = 0;
  if (settings.mplexShifter > 0) {
    //first six buttons are for gear 1-6, but shifter only has 4 switches, multiplex to 1 of 6 buttons
    bool switch1 = (*portInputRegister(digitalPinToPort(dpb[GEAR_BTN_IDX_1])) & digitalPinToBitMask(dpb[GEAR_BTN_IDX_1])) == 0;
    bool switch2 = (*portInputRegister(digitalPinToPort(dpb[GEAR_BTN_IDX_2])) & digitalPinToBitMask(dpb[GEAR_BTN_IDX_2])) == 0;
    bool switch3 = (*portInputRegister(digitalPinToPort(dpb[GEAR_BTN_IDX_3])) & digitalPinToBitMask(dpb[GEAR_BTN_IDX_3])) == 0;
    bool switch4 = (*portInputRegister(digitalPinToPort(dpb[GEAR_BTN_IDX_4])) & digitalPinToBitMask(dpb[GEAR_BTN_IDX_4])) == 0;
    uint8_t gear = 255;
    if (switch3) {
      gear = GEAR_BTN_IDX_3;
      if (switch1) {
        gear = GEAR_BTN_IDX_1;
      } else if (switch2) {
        gear = GEAR_BTN_IDX_5;
      }
    } else if (switch4) {
      gear = GEAR_BTN_IDX_4;
      if (switch1) {
        gear = GEAR_BTN_IDX_2;
      } else if (switch2) {
        gear = GEAR_BTN_IDX_6;
      }
    }
    if (gear <= 31) {
      bitWrite(*((uint32_t *)d), gear, gear + 1);
    }
    i = 4;
  }

  bool shiftBtnPressed = false;
  if (settings.shiftButton > 0) {
    if ((*portInputRegister(digitalPinToPort(dpb[settings.shiftButton - 1])) & digitalPinToBitMask(dpb[settings.shiftButton - 1])) == 0) {
      shiftBtnPressed = true;
    }
  }

  bool btnPressed;
  bool otherBtnPressed = false;
  for (; i < sizeof(dpb); i++) {
    if (i != settings.shiftButton - 1) {
      btnPressed = (*portInputRegister(digitalPinToPort(dpb[i])) & digitalPinToBitMask(dpb[i])) == 0;
      if (btnPressed) {
        otherBtnPressed = true;
      }
      if (shiftBtnPressed) {
        bitWrite(*((uint32_t *)d), DPB_1ST_BTN - 1 + i + sizeof(dpb), btnPressed);
      } else {
        bitWrite(*((uint32_t *)d), DPB_1ST_BTN - 1 + i, btnPressed);
      }
    }
  }
  if (settings.shiftButton > 0) {
    bitWrite(*((uint32_t *)d), settings.shiftButton - 1, shiftBtnPressed && !otherBtnPressed);
  }

  //debounce
  if (settings.debounce) {
    if (tempButtons != buttons) {
      debounceCount = 0;
      tempButtons = buttons;
    } else if (debounceCount < settings.debounce)
      debounceCount++;
    else {
      wheel.buttons = buttons;
    }
  }

  if ((wheel.buttons & (uint32_t)pow(2, BTN_ACTION_1_INDEX)) != 0) {
    doButtonAction(settings.btn11ActionKey);
  } else if ((wheel.buttons & (uint32_t)pow(2, BTN_ACTION_2_INDEX)) != 0) {
    doButtonAction(settings.btn12ActionKey);
  } else if ((wheel.buttons & (uint32_t)pow(2, BTN_ACTION_3_INDEX)) != 0) {
    doButtonAction(settings.btn13ActionKey);
  } else if ((wheel.buttons & (uint32_t)pow(2, BTN_ACTION_4_INDEX)) != 0) {
    doButtonAction(settings.btn14ActionKey);
  }
}

//---------------------------------------- end buttons ----------------------------------------------

//load and save settings
void load(bool defaults) {
  SettingsEEPROM settingsE;
  uint8_t i;
  uint8_t checksum;
  EEPROM.get(0, settingsE);

  checksum = settingsE.calcChecksum();

  //Loading defaults
  if (defaults || (settingsE.checksum != checksum)) {

    for (i = 0; i < GAIN_COUNT; i++) {
      settingsE.data.gain[i] = 1024;
    }

    settingsE.data.gain[2] = 768;
    settingsE.data.gain[4] = 1280;
    settingsE.data.gain[9] = 410;
    settingsE.data.gain[11] = 256;
    //settingsE.data.gain[12] = 0;

    //wheel settings
    settingsE.range = WHEEL_RANGE_DEFAULT;
    settingsE.axisMin = -511;
    settingsE.axisMax = 511;
    settingsE.axisCenter = 0;
    settingsE.axisDZ = 0;
    settingsE.axisBitTrim = 0;
    settingsE.invertRotation = 0;

    settingsE.data.shiftButton = 0;  //no shift button

    for (i = 0; i < AXIS_COUNT; i++) {
      if (i < 3) {
        settingsE.axes[i].axisMin = DEFAULT_AA_MIN;
        settingsE.axes[i].axisMax = DEFAULT_AA_MAX;
      } else {
        settingsE.axes[i].axisMin = 0;
        settingsE.axes[i].axisMax = 1023;
      }
      if (i < 2) {
        settingsE.axes[i].axisOutputDisabled = 0;
      } else {
        settingsE.axes[i].axisOutputDisabled = 1;
      }
      settingsE.axes[i].axisCenter = SHRT_MIN;  //no center
      settingsE.axes[i].axisDZ = 0;
      settingsE.axes[i].axisBitTrim = 0;
    }
    settingsE.data.debounce = 0;
    settingsE.data.minForce = 0;
    settingsE.data.maxForce = 16383;
    settingsE.data.cutForce = 16383;
    settingsE.ffbBD = DEFAULT_FFB_BITDEPTH;

    settingsE.maxVelocityDamper = DEFAULT_MAX_VELOCITY;
    settingsE.maxVelocityFriction = DEFAULT_MAX_VELOCITY;
    settingsE.maxAcceleration = DEFAULT_MAX_ACCELERATION;

    settingsE.data.btn11ActionKey = ButtonAction::NONE;
    settingsE.data.btn12ActionKey = ButtonAction::NONE;
    settingsE.data.btn13ActionKey = ButtonAction::SHUTDOWN;
    settingsE.data.btn14ActionKey = ButtonAction::ESC;

    settingsE.data.constantSpring = 0;
    settingsE.data.afcOnStartup = 0;
    settingsE.data.mplexShifter = 0;
  }

  //settingsE.print();

  settings = settingsE.data;

  for (i = 0; i < AXIS_COUNT; i++) {
    wheel.analogAxes[i]->setLimits(settingsE.axes[i].axisMin, settingsE.axes[i].axisMax);
    wheel.analogAxes[i]->setCenter(settingsE.axes[i].axisCenter);
    if (!wheel.analogAxes[i]->autoCenter)
      wheel.analogAxes[i]->setDZ(settingsE.axes[i].axisDZ);

    wheel.analogAxes[i]->bitTrim = settingsE.axes[i].axisBitTrim;
    wheel.analogAxes[i]->outputDisabled = settingsE.axes[i].axisOutputDisabled;
  }

  //wheel settings
  wheel.axisWheel->setRange(settingsE.range);
  wheel.axisWheel->setLimits(settingsE.axisMin, settingsE.axisMax);
  wheel.axisWheel->setCenter(settingsE.axisCenter);
  if (!wheel.axisWheel->autoCenter) {
    wheel.axisWheel->setDZ(settingsE.axisDZ);
  }
  wheel.axisWheel->bitTrim = settingsE.axisBitTrim;
  wheel.axisWheel->invertRotation = settingsE.invertRotation;

  wheel.ffbEngine.maxVelocityDamperC = 16384.0 / settingsE.maxVelocityDamper;
  wheel.ffbEngine.maxVelocityFrictionC = 16384.0 / settingsE.maxVelocityFriction;
  wheel.ffbEngine.maxAccelerationInertiaC = 16384.0 / settingsE.maxAcceleration;
}

void save() {
  uint8_t i;
  SettingsEEPROM settingsE;
  settingsE.data = settings;

  //wheel settings
  settingsE.range = wheel.axisWheel->range;
  settingsE.axisMin = wheel.axisWheel->axisMin;
  settingsE.axisMax = wheel.axisWheel->axisMax;
  if (!wheel.axisWheel->autoCenter) {
    settingsE.axisCenter = wheel.axisWheel->getCenter();
  } else {
    settingsE.axisCenter = SHRT_MIN;
  }
  settingsE.axisDZ = wheel.axisWheel->getDZ();
  settingsE.axisBitTrim = wheel.axisWheel->bitTrim;
  settingsE.invertRotation = wheel.axisWheel->invertRotation;

  for (i = 0; i < AXIS_COUNT; i++) {
    settingsE.axes[i].axisMin = wheel.analogAxes[i]->axisMin;
    settingsE.axes[i].axisMax = wheel.analogAxes[i]->axisMax;
    if (!wheel.analogAxes[i]->autoCenter)
      settingsE.axes[i].axisCenter = wheel.analogAxes[i]->getCenter();
    else
      settingsE.axes[i].axisCenter = SHRT_MIN;
    settingsE.axes[i].axisDZ = wheel.analogAxes[i]->getDZ();

    settingsE.axes[i].axisBitTrim = wheel.analogAxes[i]->bitTrim;
    settingsE.axes[i].axisOutputDisabled = wheel.analogAxes[i]->outputDisabled;
  }

  settingsE.maxVelocityDamper = round(16384.0 / wheel.ffbEngine.maxVelocityDamperC);
  settingsE.maxVelocityFriction = round(16384.0 / wheel.ffbEngine.maxVelocityFrictionC);
  settingsE.maxAcceleration = round(16384.0 / wheel.ffbEngine.maxAccelerationInertiaC);

  settingsE.checksum = settingsE.calcChecksum();

  EEPROM.put(0, settingsE);
}

void processSerial() {
  if (Serial.available()) {
    char cmd[16];
    uint8_t cmdLength;
    int32_t arg1 = -32768;

    cmdLength = Serial.readBytesUntil(' ', cmd, 15);
    cmd[cmdLength] = 0;

    if (Serial.available())
      arg1 = Serial.parseInt(SKIP_WHITESPACE);

    if (strcmp_P(cmd, PSTR("spring")) == 0) {
      if (arg1 >= 0) {
        settings.constantSpring = arg1;
      }
    }
  }
}