#include <EEPROM.h>
#include <PCF8574.h>
#include "config.h"
#include "wheel.h"
#include "motor.h"
#include "settings.h"
#include <HID-Project.h>
#include "Keypad.h"
#include <ArduinoShrink.h>

#define KEYPAD_ROWS 4  //four rows
#define KEYPAD_COLS 3  //three columns
char keys[KEYPAD_ROWS][KEYPAD_COLS] = {
  { '1', '2', '3' },
  { '4', '5', '6' },
  { '7', '8', '9' },
  { '*', '0', '#' }
};
byte rowPins[KEYPAD_ROWS] = { 1, 6, 5, 3 };  //connect to the row pinouts of the kpd
byte colPins[KEYPAD_COLS] = { 2, 0, 4 };     //connect to the column pinouts of the kpd
long lastKeypadCheck = 0;
PCF8574 keypadIO(0x20);
Keypad keypad(&keypadIO, makeKeymap(keys), rowPins, colPins, KEYPAD_ROWS, KEYPAD_COLS);

//global variables
Wheel_ wheel;
Motor motor;
SettingsData settings;
uint32_t tempButtons;
uint8_t debounceCount = 0;
static const uint8_t dpb[] = { DPB_PINS };
bool keypadConnected = false;

void load(bool defaults = false);
int32_t getWheelPositionAnalog();

//------------------------ steering wheel sensor ----------------------------
#define GET_WHEEL_POS getWheelPositionAnalog()

//-------------------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(50);

  //Keyboard.begin();
  //System.begin();

  for (uint8_t i = 0; i < sizeof(dpb); i++) {
    pinMode(dpb[i], INPUT_PULLUP);
  }

  //load settings
  load();

  //motor setup
  motor.begin();

  //while (!Serial) {  // Wait for serial port to connect
  //  ;                // do nothing (loop until Serial is ready)
  //}
  Wire.begin();
  keypadConnected = keypadIO.begin() && keypadIO.isConnected();
}

void loop() {

  readAnalogAxes();
  readButtons();
  processUsbCmd();
  wheel.update();
  processFFB();
  processSerial();
  if (keypadConnected) {
    processKeypad();
  }

  delay(6);
}

void processKeypad() {
  if (millis() - lastKeypadCheck > 50) {
    lastKeypadCheck = millis();

    if (keypad.getKeys()) {
      for (int i = 0; i < LIST_MAX; i++) {  // Scan the whole key list.
        if (keypad.key[i].stateChanged) {   // Only find keys that have changed state.
          char key = keypad.key[i].kchar;
          char keycode = 0;
          switch (key) {
            case '*':
              keycode = KEYPAD_MULTIPLY;
              break;
            case '#':
              keycode = KEYPAD_DOT;
              break;
            case '1':
              keycode = KEYPAD_1;
              break;
            case '2':
              keycode = KEYPAD_2;
              break;
            case '3':
              keycode = KEYPAD_3;
              break;
            case '4':
              keycode = KEYPAD_4;
              break;
            case '5':
              keycode = KEYPAD_5;
              break;
            case '6':
              keycode = KEYPAD_6;
              break;
            case '7':
              keycode = KEYPAD_7;
              break;
            case '8':
              keycode = KEYPAD_8;
              break;
            case '9':
              keycode = KEYPAD_9;
              break;
            case '0':
              keycode = KEYPAD_0;
              break;
          }

          switch (keypad.key[i].kstate) {  // Report active key state : IDLE, PRESSED, HOLD, or RELEASED
            case PRESSED:
              if (key == '#' && (isHeld('*') || isPressed('*'))) {
                Keyboard.write(KEYPAD_ENTER);
              } else if (key == '6' && (isHeld('*') || isPressed('*'))) {
                //*6 will toggle numlock mode
                Keyboard.write(KEY_NUM_LOCK);
              } else {
                Keyboard.press(KeyboardKeycode(keycode));
              }
              break;
            case HOLD:
              break;
            case RELEASED:
              Keyboard.release(KeyboardKeycode(keycode));
              break;
            case IDLE:
              break;
          }
        }
      }
    }
  }
}

bool isHeld(char keyChar) {
  for (byte i = 0; i < LIST_MAX; i++) {
    if (keypad.key[i].kchar == keyChar) {
      if ((keypad.key[i].kstate == HOLD))
        return true;
    }
  }
  return false;  // Not held.
}

bool isPressed(char keyChar) {
  for (byte i = 0; i < LIST_MAX; i++) {
    if (keypad.key[i].kchar == keyChar) {
      if ((keypad.key[i].kstate == PRESSED))
        return true;
    }
  }
  return false;  // Not pressed.
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
      case 23:  //center wheel
        //center();
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
#ifdef PIN_AUX3
  wheel.analogAxes[AXIS_AUX3]->setValue(analogRead(PIN_AUX3));
#endif
#ifdef PIN_AUX4
  wheel.analogAxes[AXIS_AUX4]->setValue(analogRead(PIN_AUX4));
#endif
#ifdef PIN_AUX5
  wheel.analogAxes[AXIS_AUX5]->setValue(analogRead(PIN_AUX5));
#endif
#ifdef PIN_ST_ANALOG
  GET_WHEEL_POS;
#endif
}

//-----------------------------------end analog axes------------------------------

//-----------------------------------reading buttons------------------------------
void readButtons() {
  uint32_t buttons = 0;

  uint8_t *d;
  if (settings.debounce) {
    d = (uint8_t *)&buttons;
  } else {
    wheel.buttons = 0;
    d = (uint8_t *)&wheel.buttons;
  }

  int i = 0;
  if (settings.mplexShifter > 0) {
    //first six buttons are for gear 1-6, but shifter only has 4 switches, multiplex to 1 of 6 buttons
    bool switch1 = (*portInputRegister(digitalPinToPort(dpb[GEAR_BTN_IDX_1])) & digitalPinToBitMask(dpb[GEAR_BTN_IDX_1])) == 0;
    bool switch2 = (*portInputRegister(digitalPinToPort(dpb[GEAR_BTN_IDX_2])) & digitalPinToBitMask(dpb[GEAR_BTN_IDX_2])) == 0;
    bool switch3 = (*portInputRegister(digitalPinToPort(dpb[GEAR_BTN_IDX_3])) & digitalPinToBitMask(dpb[GEAR_BTN_IDX_3])) == 0;
    bool switch4 = (*portInputRegister(digitalPinToPort(dpb[GEAR_BTN_IDX_4])) & digitalPinToBitMask(dpb[GEAR_BTN_IDX_4])) == 0;
    uint8_t gear = 32767;
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
    if (gear >= 0 && gear <= 31) {
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


  if ((wheel.buttons & (uint32_t)pow(2, BTN_ESC_INDEX)) != 0) {
    Keyboard.press(KEY_ESC);
    delay(5);
    Keyboard.release(KEY_ESC);
  }
}
//---------------------------------------- end buttons ----------------------------------------------


int32_t getWheelPositionAnalog() {
  wheel.axisWheel->setValue(analogRead(PIN_ST_ANALOG));
  return wheel.axisWheel->value;
}

//Serial port - commands and output.
void processSerial() {

  if (Serial.available()) {
    char cmd[16];
    uint8_t cmdLength;
    int32_t arg1, arg2, arg3;

    arg1 = -32768;
    arg2 = -32768;
    arg3 = -32768;

    cmdLength = Serial.readBytesUntil(' ', cmd, 15);
    cmd[cmdLength] = 0;

    if (Serial.available())
      arg1 = Serial.parseInt(SKIP_WHITESPACE);
    if (Serial.available())
      arg2 = Serial.parseInt(SKIP_WHITESPACE);
    if (Serial.available())
      arg3 = Serial.parseInt(SKIP_WHITESPACE);

    //center
    //if (strcmp_P(cmd, PSTR("center")) == 0)
    // center();

    if (strcmp_P(cmd, PSTR("load")) == 0)
      load();

    if (strcmp_P(cmd, PSTR("defaults")) == 0)
      load(true);

    if (strcmp_P(cmd, PSTR("save")) == 0)
      save();

    /*if (strcmp_P(cmd, PSTR("shiftbtn")) == 0) {
      if ((arg1 >= 0) && (arg1 <= 32)) {
        settings.shiftButton = arg1 - 1;
      }
      Serial.print(F("Shift button: "));
      Serial.println(settings.shiftButton + 1);
    }*/

    /*
    if (strcmp_P(cmd, PSTR("range")) == 0) {
      if (arg1 > 0) {
        wheel.axisWheel->setRange(arg1);
      }
      //Serial.print(F("Wheel range: "));
      //Serial.println(wheel.axisWheel->range);
    }

    if (strcmp_P(cmd, PSTR("maxvd")) == 0) {
      if (arg1 > 0) {
        wheel.ffbEngine.maxVelocityDamperC = 16384.0 / arg1;
      }
      //Serial.print(F("max velocity damper: "));
      //Serial.println(round(16384.0 / wheel.ffbEngine.maxVelocityDamperC));
    }

    if (strcmp_P(cmd, PSTR("maxvf")) == 0) {
      if (arg1 > 0) {
        wheel.ffbEngine.maxVelocityFrictionC = 16384.0 / arg1;
      }
      //Serial.print(F("max velocity friction: "));
      //Serial.println(round(16384.0 / wheel.ffbEngine.maxVelocityFrictionC));
    }

    if (strcmp_P(cmd, PSTR("maxacc")) == 0) {
      if (arg1 > 0) {
        wheel.ffbEngine.maxAccelerationInertiaC = 16384.0 / arg1;
      }
      //Serial.print(F("max acceleration: "));
      //Serial.println(round(16384.0 / wheel.ffbEngine.maxAccelerationInertiaC));
    }

    if (strcmp_P(cmd, PSTR("forcelimit")) == 0) {
      if ((arg1 >= 0) && (arg1 <= 16383))
        settings.minForce = arg1;
      if ((arg2 >= 0) && (arg2 <= 16383))
        settings.maxForce = arg2;
      if ((arg3 >= 0) && (arg3 <= 16383))
        settings.cutForce = arg3;

      //Serial.print(F("MinForce: "));
      //Serial.print(settings.minForce);
      //Serial.print(F(" MaxForce: "));
      //Serial.print(settings.maxForce);
      //Serial.print(F(" CutForce: "));
      //Serial.println(settings.cutForce);
    }

    if (strcmp_P(cmd, PSTR("gain")) == 0) {
      if ((arg1 >= 0) && (arg1 <= 12)) {
        if ((arg2 >= 0) && (arg2 <= 32767)) {
          settings.gain[arg1] = arg2;
        }
        //Serial.print(F("Gain "));
        //Serial.print(arg1);
        //Serial.print(" ");
        //printEffect(arg1);
        //Serial.print(F(": "));
        //Serial.println(settings.gain[arg1]);
      }
    }

    //axisinfo <axis>
    if (strcmp_P(cmd, PSTR("axisinfo")) == 0) {
      if ((arg1 >= 0) && (arg1 <= AXIS_COUNT))
        axisInfo = arg1;
      else
        axisInfo = -1;
    }

    //limits <axis> <min> <max>
    if (strcmp_P(cmd, PSTR("limit")) == 0)
      if ((arg1 >= 1) && (arg1 <= AXIS_COUNT)) {
        if ((arg2 > -32768) && (arg3 > -32768))
          wheel.analogAxes[arg1 - 1]->setLimits(arg2, arg3);

        //Serial.print(F("Limits axis#"));
        //Serial.print(arg1);
        //Serial.print(F(": "));
        //Serial.print(wheel.analogAxes[arg1 - 1]->axisMin);
        //Serial.print(" ");
        //Serial.println(wheel.analogAxes[arg1 - 1]->axisMax);
      }

    //axiscenter <axis> <pos>
    if (strcmp_P(cmd, PSTR("axiscenter")) == 0)
      if ((arg1 >= 1) && (arg1 <= AXIS_COUNT)) {
        if (arg2 > -32768)
          wheel.analogAxes[arg1 - 1]->setCenter(arg2);
        //Serial.print(F("Axis#"));
        //Serial.print(arg1);
        //Serial.print(F(" center:"));
        //Serial.println(wheel.analogAxes[arg1 - 1]->getCenter());
      }

    //axisdz <axis> <pos>
    if (strcmp_P(cmd, PSTR("axisdz")) == 0)
      if ((arg1 >= 1) && (arg1 <= AXIS_COUNT)) {
        if (arg2 > -32768)
          wheel.analogAxes[arg1 - 1]->setDZ(arg2);
        //Serial.print(F("Axis#"));
        //Serial.print(arg1);
        //Serial.print(F(" Deadzone:"));
        //Serial.println(wheel.analogAxes[arg1 - 1]->getDZ());
      }

    //axisdisable <axis> <pos>
    if (strcmp_P(cmd, PSTR("axisdisable")) == 0)
      if ((arg1 >= 1) && (arg1 <= AXIS_COUNT)) {
        wheel.analogAxes[arg1 - 1]->outputDisabled = !wheel.analogAxes[arg1 - 1]->outputDisabled;

        //Serial.print(F("Axis#"));
        //Serial.print(arg1);
        //if (wheel.analogAxes[arg1 - 1]->outputDisabled)
        //  Serial.println(F(" disabled"));
        // else
        //  Serial.println(F(" enabled"));
      }

    //axistrim <axis> <level>
    if (strcmp_P(cmd, PSTR("axistrim")) == 0)
      if ((arg1 >= 1) && (arg1 <= AXIS_COUNT)) {
        if ((arg2 >= 0) && (arg2 < 8))
          wheel.analogAxes[arg1 - 1]->bitTrim = arg2;

        //Serial.print(F("Axis#"));
        //Serial.print(arg1);
        //Serial.print(F(" trim:"));
        //Serial.println(wheel.analogAxes[arg1 - 1]->bitTrim);
      }

    //autolimits <axis>
    if (strcmp_P(cmd, PSTR("autolimit")) == 0)
      if ((arg1 >= 1) && (arg1 <= AXIS_COUNT)) {
        wheel.analogAxes[arg1 - 1]->setAutoLimits(!wheel.analogAxes[arg1 - 1]->autoLimit);
        /*Serial.print(F("Axis #"));
        Serial.print(arg1);
        Serial.print(F(" autolimit"));
        if (wheel.analogAxes[arg1 - 1]->autoLimit)
          Serial.println(F(" on"));
        else
          Serial.println(F(" off"));
      }

    //FFB PWM bitdepth/frequency
    if (strcmp_P(cmd, PSTR("ffbbd")) == 0) {
      if (arg1 > 0)
        motor.setBitDepth(arg1);
      //Serial.print(F("FFB Bitdepth:"));
      // Serial.print(motor.bitDepth);
      // Serial.print(F(" Freq:"));
      // Serial.println(16000000 / ((uint16_t)1 << (motor.bitDepth + 1)));
    }

    //Debounce
    if (strcmp_P(cmd, PSTR("debounce")) == 0) {
      if (arg1 >= 0)
        settings.debounce = arg1;
      //Serial.print(F("Debounce:"));
      //Serial.println(settings.debounce);
    } 
*/
    if (strcmp_P(cmd, PSTR("spring")) == 0) {
      if (arg1 >= 0) {
        settings.constantSpring = arg1;
      }
      //Serial.print(F("spring:"));
      //Serial.println(settings.constantSpring);
    }

    /*if (strcmp_P(cmd, PSTR("version")) == 0) {
      Serial.print(F(FIRMWARE_TYPE));
      Serial.print(F(":"));
      Serial.println(F(FIRMWARE_VER));
    }*/
  }
}

//load and save settings
void load(bool defaults) {
  SettingsEEPROM settingsE;
  uint8_t i;
  uint8_t checksum;
  EEPROM.get(0, settingsE);

  checksum = settingsE.calcChecksum();

  //Loading defaults
  if (defaults || (settingsE.checksum != checksum)) {

    //Serial.println(F("Loading defaults"));

    settingsE.data.gain[0] = 1024;
    settingsE.data.gain[1] = 1024;
    settingsE.data.gain[2] = 768;
    settingsE.data.gain[3] = 1024;
    settingsE.data.gain[4] = 1280;
    settingsE.data.gain[5] = 1024;
    settingsE.data.gain[6] = 1024;
    settingsE.data.gain[7] = 1024;
    settingsE.data.gain[8] = 1024;
    settingsE.data.gain[9] = 410;
    settingsE.data.gain[10] = 1024;
    settingsE.data.gain[11] = 256;
    settingsE.data.gain[12] = 0;

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
      settingsE.axes[i].axisCenter = -32768;  //no center
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

    settingsE.data.constantSpring = 0;
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
    settingsE.axisCenter = -32768;
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
      settingsE.axes[i].axisCenter = -32768;
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