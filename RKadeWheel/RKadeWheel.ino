#include <EEPROM.h>
#include <digitalWriteFast.h>       //https://github.com/NicksonYap/digitalWriteFast
#include <avdweb_AnalogReadFast.h>  //https://github.com/avandalen/avdweb_AnalogReadFast
#include <AnalogIO.h>
#include <Smooth.h>

#include "config.h"
#include "wheel.h"
//#include "motor.h"
#include "settings.h"

//global variables
Wheel_ wheel;
//Motor motor;
SettingsData settings;
int16_t force;
int8_t axisInfo = -1;
uint32_t tempButtons;
uint8_t debounceCount = 0;
AnalogOut lShaker(9);
AnalogOut rShaker(10);
AnalogOut blower(11);
AnalogOut brakeLed(13);
Smooth smoothAcc(350);

#ifdef DPB
static const uint8_t dpb[] = { DPB_PINS };
#endif

//constants and definitions for analog axes
#if ((PEDALS_TYPE == PT_INTERNAL) || (PEDALS_TYPE == PT_HC164))
#define DEFAULT_AA_MIN 0
#define DEFAULT_AA_MAX 1023
#endif

void load(bool defaults = false);
int32_t getWheelPositionAnalog();

//------------------------ steering wheel sensor ----------------------------

#if STEER_TYPE == ST_ENCODER
#include <Encoder.h>  //https://github.com/PaulStoffregen/Encoder
Encoder encoder(ENCODER_PIN1, ENCODER_PIN2);
#define GET_WHEEL_POS (((int32_t)encoder.read() << STEER_BITDEPTH) / ENCODER_PPR)
#define CENTER_WHEEL encoder.write(0);
#define SET_WHEEL_POSITION(val) encoder.write((val * ENCODER_PPR) / (1 << STEER_BITDEPTH))
#endif

#if STEER_TYPE == ST_ANALOG
#define GET_WHEEL_POS getWheelPositionAnalog()
#endif


//-------------------------------------------------------------------------------------

void setup() {

  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(50);

  //set up analog axes
#ifdef AA_PULLUP
  analogReference(INTERNAL);  //2.56v reference to get more resolution.
#endif

//direct pin buttons
#ifdef DPB
  for (uint8_t i = 0; i < sizeof(dpb); i++) {
    pinMode(dpb[i], INPUT_PULLUP);
  }
  //Keyboard.begin();
#endif

  //motor setup
  //motor.begin();

  //load settings
  load();
}

void loop() {

  readAnalogAxes();

#if STEER_TYPE != ST_ANALOG
  //wheel.axisWheel->setValue(GET_WHEEL_POS);
#endif
  //#ifndef BT_NONE
  readButtons();
  //#endif
  processUsbCmd();
  wheel.update();
  processFFB();


#ifdef SERIAL_CMD
  processSerial();
#endif

  processBlower();
  processLights();

  delay(4);
}

void processBlower() {
  int16_t accVal = wheel.analogAxes[AXIS_ACC]->rawValue;
  accVal = round(smoothAcc.add(accVal));
  int16_t accMax = wheel.analogAxes[AXIS_ACC]->axisMax;
  int16_t accMin = wheel.analogAxes[AXIS_ACC]->axisMin;
  if (accVal < accMin) {
    accVal = accMin;
  }
  if (accVal > accMax) {
    accVal = accMax;
  }

  float accPct = (((float)accVal - accMin) / ((float)accMax - accMin));
  int16_t pwmValBlower = accPct * MAX_BLOWER_PWM;
  if (pwmValBlower < 0) {
    pwmValBlower = 0;
  } else if (pwmValBlower > MAX_BLOWER_PWM) {
    pwmValBlower = MAX_BLOWER_PWM;
  }
  blower.write(pwmValBlower);
}

void processLights() {
  int16_t accVal = wheel.analogAxes[AXIS_BRAKE]->rawValue;
  int16_t accMin = wheel.analogAxes[AXIS_BRAKE]->axisMin;
  if (accVal - accMin > 5) {
    brakeLed.write(MAX_LIGHT_PWM);
  } else {
    brakeLed.write(0);
  }
}

//Processing endstop and force feedback
void processFFB() {

  wheel.ffbEngine.constantSpringForce();

  force = wheel.ffbEngine.calculateForce(wheel.axisWheel);
  force = applyForceLimit(force);

  boolean isNegative = false;
  if (abs(force) < 2) {
    lShaker.write(0);
    rShaker.write(0);
    return;
  } else if (force < 0) {
    isNegative = true;
  }

  int16_t pwm = abs(force);
  float forcePct = (((float)pwm) / ((float)16383));
  pwm = forcePct * 255;
  if (isNegative) {
    rShaker.write(0);
    lShaker.write(pwm);
  } else {
    lShaker.write(0);
    rShaker.write(pwm);
  }
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
#if STEER_TYPE == ST_ANALOG
        ((GUI_Report_SteerAxis *)data)->rawValue = wheel.axisWheel->rawValue;
        ((GUI_Report_SteerAxis *)data)->value = wheel.axisWheel->value;
#else
        ((GUI_Report_SteerAxis *)data)->rawValue = wheel.axisWheel->rawValue;
        ((GUI_Report_SteerAxis *)data)->value = wheel.axisWheel->value;
#endif

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

        //((GUI_Report_Settings *)data)->ffbBD = motor.bitDepth;

        ((GUI_Report_Settings *)data)->endstopOffset = settings.endstopOffset;
        ((GUI_Report_Settings *)data)->endstopWidth = settings.endstopWidth;
        ((GUI_Report_Settings *)data)->constantSpring = settings.constantSpring;
        ((GUI_Report_Settings *)data)->afcOnStartup = settings.afcOnStartup;
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
            //motor.setBitDepth(usbCmd->arg[1]);
            break;
          case 7:
            settings.endstopOffset = usbCmd->arg[1];
            settings.endstopWidth = usbCmd->arg[2];
            break;
          case 8:
            settings.constantSpring = usbCmd->arg[1];
            break;
          case 9:
            settings.afcOnStartup = usbCmd->arg[1];
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

#if (PEDALS_TYPE == PT_INTERNAL)
#ifdef AA_PULLUP_LINEARIZE
  wheel.analogAxes[AXIS_ACC]->setValue(pullup_linearize(analogReadFast(PIN_ACC)));
  wheel.analogAxes[AXIS_BRAKE]->setValue(pullup_linearize(analogReadFast(PIN_BRAKE)));
  //wheel.analogAxes[AXIS_CLUTCH]->setValue(pullup_linearize(analogReadFast(PIN_CLUTCH)));
#else
  wheel.analogAxes[AXIS_ACC]->setValue(analogReadFast(PIN_ACC));
  wheel.analogAxes[AXIS_BRAKE]->setValue(analogReadFast(PIN_BRAKE));
  //wheel.analogAxes[AXIS_CLUTCH]->setValue(analogReadFast(PIN_CLUTCH));
#endif
#endif

//additional axes
#ifdef AA_PULLUP_LINEARIZE

#ifdef PIN_AUX1
  wheel.analogAxes[AXIS_AUX1]->setValue(pullup_linearize(analogReadFast(PIN_AUX1)));
#endif
#ifdef PIN_AUX2
  wheel.analogAxes[AXIS_AUX2]->setValue(pullup_linearize(analogReadFast(PIN_AUX2)));
#endif
#ifdef PIN_AUX3
  wheel.analogAxes[AXIS_AUX3]->setValue(pullup_linearize(analogReadFast(PIN_AUX3)));
#endif
#ifdef PIN_AUX4
  wheel.analogAxes[AXIS_AUX4]->setValue(pullup_linearize(analogReadFast(PIN_AUX4)));
#endif
#ifdef PIN_AUX5
  wheel.analogAxes[AXIS_AUX5]->setValue(pullup_linearize(analogReadFast(PIN_AUX5)));
#endif
#ifdef PIN_ST_ANALOG
  GET_WHEEL_POS;
#endif
#else
#ifdef PIN_AUX1
  wheel.analogAxes[AXIS_AUX1]->setValue(analogReadFast(PIN_AUX1));
#endif
#ifdef PIN_AUX2
  wheel.analogAxes[AXIS_AUX2]->setValue(analogReadFast(PIN_AUX2));
#endif
#ifdef PIN_AUX3
  wheel.analogAxes[AXIS_AUX3]->setValue(analogReadFast(PIN_AUX3));
#endif
#ifdef PIN_AUX4
  wheel.analogAxes[AXIS_AUX4]->setValue(analogReadFast(PIN_AUX4));
#endif
#ifdef PIN_AUX5
  wheel.analogAxes[AXIS_AUX5]->setValue(analogReadFast(PIN_AUX5));
#endif
#ifdef PIN_ST_ANALOG
  GET_WHEEL_POS;
#endif
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

//direct pin buttons
#ifdef DPB
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
#endif

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
}
//---------------------------------------- end buttons ----------------------------------------------


int32_t getWheelPositionAnalog() {

#ifdef AA_PULLUP_LINEARIZE
  wheel.axisWheel->setValue(pullup_linearize(analogReadFast(PIN_ST_ANALOG)));
#else
  wheel.axisWheel->setValue(analogReadFast(PIN_ST_ANALOG));
#endif

  return wheel.axisWheel->value;
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

    Serial.println(F("Loading defaults"));

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

    settingsE.data.endstopOffset = DEFAULT_ENDSTOP_OFFSET;
    settingsE.data.endstopWidth = DEFAULT_ENDSTOP_WIDTH;
    settingsE.data.constantSpring = 0;
    settingsE.data.afcOnStartup = 0;
    settingsE.data.mplexShifter = 0;
  }

  settingsE.print();

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

#ifdef AA_PULLUP_LINEARIZE
//Linearizing axis values, when using internal adc + pullup.
int16_t pullup_linearize(int16_t val) {
  //Assuming VCC=5v, ADCreference=2.56v, 0 < val < 1024
  //val = val * 1024 * (R_pullup / R_pot) / (1024 * 5 / 2.56 - val)
  //if Rpullup = R_pot...
  //val = val * 1024 / (2000 - val)
  //division is slow
  //piecewise linear approximation, 16 steps
  static const int16_t a[] = { 0, 33, 70, 108, 150, 195, 243, 295, 352, 414, 481, 556, 638, 729, 831, 945 };
  static const uint8_t b[] = { 33, 37, 38, 42, 45, 48, 52, 57, 62, 67, 75, 82, 91, 102, 114, 129 };

  uint8_t i = (val >> 6);

  return a[i] + (((val % 64) * b[i]) >> 6);
}
#endif

#ifdef SERIAL_CMD
//Serial port - commands and output.
void processSerial() {

  //output axis data
  if (axisInfo == 0) {
    Serial.print(F("Axis#0 Raw:"));
    Serial.print(wheel.axisWheel->rawValue);
    Serial.print(F("\tAbs: "));
    Serial.print(wheel.axisWheel->absValue);
    Serial.print(F("\tMax: "));
    Serial.print(wheel.axisWheel->axisMax);
    Serial.print(F("\tValue: "));
    Serial.print(wheel.axisWheel->value);
    Serial.print(F(" ("));
    Serial.print((int8_t)(wheel.axisWheel->value * (100.0 / 65534)) + 50);
    Serial.print(F("%)\tV:"));
    Serial.print(wheel.axisWheel->velocity);
    Serial.print(F("\tA:"));
    Serial.print(wheel.axisWheel->acceleration);
    Serial.print(F("\tFFB:"));
    Serial.println(force);
  } else if ((axisInfo > 0) && (axisInfo <= AXIS_COUNT)) {
    Serial.print(F("Axis#"));
    Serial.print(axisInfo);
    Serial.print(F("\tRaw: "));
    Serial.print(wheel.analogAxes[axisInfo - 1]->rawValue);
    Serial.print(F("\tAutoLimit: "));
    Serial.print(wheel.analogAxes[axisInfo - 1]->autoLimit);
    Serial.print(F("\tAutoCenter: "));
    Serial.print(wheel.analogAxes[axisInfo - 1]->autoCenter);
    Serial.print(F("\tDeadZone: "));
    Serial.print(wheel.analogAxes[axisInfo - 1]->getDZ());
    Serial.print(F("\tMin: "));
    Serial.print(wheel.analogAxes[axisInfo - 1]->axisMin);
    Serial.print(F("\tCenter: "));
    Serial.print(wheel.analogAxes[axisInfo - 1]->getCenter());
    Serial.print(F("\tMax: "));
    Serial.print(wheel.analogAxes[axisInfo - 1]->axisMax);
    Serial.print(F("\tValue: "));
    Serial.print(wheel.analogAxes[axisInfo - 1]->value);
    Serial.print(F(" ("));
    Serial.print((int8_t)(wheel.analogAxes[axisInfo - 1]->value * (100.0 / 65534)) + 50);
    Serial.println(F("%)"));
  }

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

    if (strcmp_P(cmd, PSTR("range")) == 0) {
      if (arg1 > 0) {
        wheel.axisWheel->setRange(arg1);
      }
      Serial.print(F("Wheel range: "));
      Serial.println(wheel.axisWheel->range);
    }

    if (strcmp_P(cmd, PSTR("maxvd")) == 0) {
      if (arg1 > 0) {
        wheel.ffbEngine.maxVelocityDamperC = 16384.0 / arg1;
      }
      Serial.print(F("max velocity damper: "));
      Serial.println(round(16384.0 / wheel.ffbEngine.maxVelocityDamperC));
    }

    if (strcmp_P(cmd, PSTR("maxvf")) == 0) {
      if (arg1 > 0) {
        wheel.ffbEngine.maxVelocityFrictionC = 16384.0 / arg1;
      }
      Serial.print(F("max velocity friction: "));
      Serial.println(round(16384.0 / wheel.ffbEngine.maxVelocityFrictionC));
    }

    if (strcmp_P(cmd, PSTR("maxacc")) == 0) {
      if (arg1 > 0) {
        wheel.ffbEngine.maxAccelerationInertiaC = 16384.0 / arg1;
      }
      Serial.print(F("max acceleration: "));
      Serial.println(round(16384.0 / wheel.ffbEngine.maxAccelerationInertiaC));
    }

    if (strcmp_P(cmd, PSTR("forcelimit")) == 0) {
      if ((arg1 >= 0) && (arg1 <= 16383))
        settings.minForce = arg1;
      if ((arg2 >= 0) && (arg2 <= 16383))
        settings.maxForce = arg2;
      if ((arg3 >= 0) && (arg3 <= 16383))
        settings.cutForce = arg3;

      Serial.print(F("MinForce: "));
      Serial.print(settings.minForce);
      Serial.print(F(" MaxForce: "));
      Serial.print(settings.maxForce);
      Serial.print(F(" CutForce: "));
      Serial.println(settings.cutForce);
    }

    if (strcmp_P(cmd, PSTR("gain")) == 0) {
      if ((arg1 >= 0) && (arg1 <= 12)) {
        if ((arg2 >= 0) && (arg2 <= 32767)) {
          settings.gain[arg1] = arg2;
        }
        Serial.print(F("Gain "));
        Serial.print(arg1);
        Serial.print(" ");
        //printEffect(arg1);
        Serial.print(F(": "));
        Serial.println(settings.gain[arg1]);
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

        Serial.print(F("Limits axis#"));
        Serial.print(arg1);
        Serial.print(F(": "));
        Serial.print(wheel.analogAxes[arg1 - 1]->axisMin);
        Serial.print(" ");
        Serial.println(wheel.analogAxes[arg1 - 1]->axisMax);
      }

    //axiscenter <axis> <pos>
    if (strcmp_P(cmd, PSTR("axiscenter")) == 0)
      if ((arg1 >= 1) && (arg1 <= AXIS_COUNT)) {
        if (arg2 > -32768)
          wheel.analogAxes[arg1 - 1]->setCenter(arg2);
        Serial.print(F("Axis#"));
        Serial.print(arg1);
        Serial.print(F(" center:"));
        Serial.println(wheel.analogAxes[arg1 - 1]->getCenter());
      }

    //axisdz <axis> <pos>
    if (strcmp_P(cmd, PSTR("axisdz")) == 0)
      if ((arg1 >= 1) && (arg1 <= AXIS_COUNT)) {
        if (arg2 > -32768)
          wheel.analogAxes[arg1 - 1]->setDZ(arg2);
        Serial.print(F("Axis#"));
        Serial.print(arg1);
        Serial.print(F(" Deadzone:"));
        Serial.println(wheel.analogAxes[arg1 - 1]->getDZ());
      }

    //axisdisable <axis> <pos>
    if (strcmp_P(cmd, PSTR("axisdisable")) == 0)
      if ((arg1 >= 1) && (arg1 <= AXIS_COUNT)) {
        wheel.analogAxes[arg1 - 1]->outputDisabled = !wheel.analogAxes[arg1 - 1]->outputDisabled;

        Serial.print(F("Axis#"));
        Serial.print(arg1);
        if (wheel.analogAxes[arg1 - 1]->outputDisabled)
          Serial.println(F(" disabled"));
        else
          Serial.println(F(" enabled"));
      }

    //axistrim <axis> <level>
    if (strcmp_P(cmd, PSTR("axistrim")) == 0)
      if ((arg1 >= 1) && (arg1 <= AXIS_COUNT)) {
        if ((arg2 >= 0) && (arg2 < 8))
          wheel.analogAxes[arg1 - 1]->bitTrim = arg2;

        Serial.print(F("Axis#"));
        Serial.print(arg1);
        Serial.print(F(" trim:"));
        Serial.println(wheel.analogAxes[arg1 - 1]->bitTrim);
      }

    //autolimits <axis>
    if (strcmp_P(cmd, PSTR("autolimit")) == 0)
      if ((arg1 >= 1) && (arg1 <= AXIS_COUNT)) {
        wheel.analogAxes[arg1 - 1]->setAutoLimits(!wheel.analogAxes[arg1 - 1]->autoLimit);
        Serial.print(F("Axis #"));
        Serial.print(arg1);
        Serial.print(F(" autolimit"));
        if (wheel.analogAxes[arg1 - 1]->autoLimit)
          Serial.println(F(" on"));
        else
          Serial.println(F(" off"));
      }

    //FFB PWM bitdepth/frequency
   /* if (strcmp_P(cmd, PSTR("ffbbd")) == 0) {
      if (arg1 > 0)
        motor.setBitDepth(arg1);
      Serial.print(F("FFB Bitdepth:"));
      Serial.print(motor.bitDepth);
      Serial.print(F(" Freq:"));
      Serial.println(16000000 / ((uint16_t)1 << (motor.bitDepth + 1)));
    }*/

    //Debounce
    if (strcmp_P(cmd, PSTR("debounce")) == 0) {
      if (arg1 >= 0)
        settings.debounce = arg1;
      Serial.print(F("Debounce:"));
      Serial.println(settings.debounce);
    }

    if (strcmp_P(cmd, PSTR("spring")) == 0) {
      if (arg1 >= 0) {
        settings.constantSpring = arg1;
      }
      Serial.print(F("spring:"));
      Serial.println(settings.constantSpring);
    }

    //Endstop
    if (strcmp_P(cmd, PSTR("endstop")) == 0) {
      if (arg1 >= 0)
        settings.endstopOffset = arg1;
      if (arg2 >= 0)
        settings.endstopWidth = arg2;
      Serial.print(F("Endstop: offset:"));
      Serial.print(settings.endstopOffset);
      Serial.print(F(" width:"));
      Serial.println(settings.endstopWidth);
    }

    if (strcmp_P(cmd, PSTR("version")) == 0) {
      Serial.print(F(FIRMWARE_TYPE));
      Serial.print(F(":"));
      Serial.println(F(FIRMWARE_VER));
    }
  }
}
#endif