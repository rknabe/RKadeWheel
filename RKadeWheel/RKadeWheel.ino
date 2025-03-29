#include <EEPROM.h>
#include <AnalogIO.h>
#include <Smooth.h>
#include <arduino-timer.h>
#include <PCF8574.h>
#include <HID-Project.h>

#include "config.h"
#include "wheel.h"
#include "settings.h"
#include "Keypad.h"
#include <ArduinoShrink.h>  //must be last

const byte KEYPAD_ROWS = 4;  //four rows
const byte KEYPAD_COLS = 3;  //three columns
char keys[KEYPAD_ROWS][KEYPAD_COLS] = {
  { '1', '2', '3' },
  { '4', '5', '6' },
  { '7', '8', '9' },
  { '*', '0', '#' }
};
byte rowPins[KEYPAD_ROWS] = { 1, 6, 5, 3 };  //connect to the row pinouts of the kpd
byte colPins[KEYPAD_COLS] = { 2, 0, 4 };     //connect to the column pinouts of the kpd
PCF8574 keypadIO(0x20);
Keypad keypad = Keypad(&keypadIO, makeKeymap(keys), rowPins, colPins, KEYPAD_ROWS, KEYPAD_COLS);

//global variables
Wheel_ wheel;
SettingsData settings;
float accelPct = 0;
bool keypadConnected = false;
uint32_t tempButtons;
uint8_t debounceCount = 0;
AnalogOut lShaker(LEFT_SHAKER_PIN);
AnalogOut rShaker(RIGHT_SHAKER_PIN);
AnalogOut blower(BLOWER_PIN);
AnalogOut brakeLed(BRAKE_LIGHT_PIN);
AnalogOut trak1Led(TRAK1_LIGHT_PIN);
AnalogOut trak2Led(TRAK2_LIGHT_PIN);
AnalogOut trak3Led(TRAK3_LIGHT_PIN);
bool trak1LedOn = false;
bool trak2LedOn = false;
bool trak3LedOn = false;
Smooth smoothAcc(350);
auto timer = timer_create_default();  // create a timer with default settings
static const uint8_t dpb[] = { BUTTON_PINS };

void load(bool defaults = false);

//-------------------------------------------------------------------------------------

void setup() {

  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(50);

  Keyboard.begin();
  System.begin();

  //direct pin buttons
  for (uint8_t i = 0; i < sizeof(dpb); i++) {
    pinMode(dpb[i], INPUT_PULLUP);
  }

  //load settings
  load();

  Wire.begin();
  keypadConnected = keypadIO.begin() && keypadIO.isConnected();
}

void loop() {
  timer.tick();

  readAnalogAxes();
  readButtons();
  processUsbCmd();
  wheel.update();
  processFFB();
  if (keypadConnected) {
    processKeypad();
  }

  calcAccelPct();
  processBlower();
  processLights();

  delay(6);
}

void calcAccelPct() {
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

  accelPct = (((float)accVal - accMin) / ((float)accMax - accMin));
}

void processBlower() {
  int16_t pwmValBlower = accelPct * MAX_BLOWER_PWM;
  if (pwmValBlower < 0) {
    pwmValBlower = 0;
  } else if (pwmValBlower > MAX_BLOWER_PWM) {
    pwmValBlower = MAX_BLOWER_PWM;
  }
  blower.write(pwmValBlower);
}

void processLights() {
  if (accelPct > 0.01) {
    if (!isTrakLedOn()) {
      turnOnTrakLed1(NULL);
    }
  } else {
    turnOffTrakLed1(NULL);
    turnOffTrakLed2(NULL);
    turnOffTrakLed3(NULL);
  }
  if ((wheel.buttons & (uint32_t)pow(2, BTN_BRAKE_INDEX)) != 0) {
    brakeLed.write(MAX_LIGHT_PWM);
  } else {
    brakeLed.write(0);
  }
}

bool turnOnTrakLed1(void *) {
  trak1LedOn = true;
  turnOffTrakLed2(NULL);
  turnOffTrakLed3(NULL);
  trak1Led.write(MAX_LIGHT_PWM);
  timer.in(getTrakLedDelay(), turnOnTrakLed2);
  return false;
}

bool turnOnTrakLed2(void *) {
  trak2LedOn = true;
  turnOffTrakLed1(NULL);
  turnOffTrakLed3(NULL);
  trak2Led.write(MAX_LIGHT_PWM);
  timer.in(getTrakLedDelay(), turnOnTrakLed3);
  return false;
}

bool turnOnTrakLed3(void *) {
  trak3LedOn = true;
  turnOffTrakLed1(NULL);
  turnOffTrakLed2(NULL);
  trak3Led.write(MAX_LIGHT_PWM);
  timer.in(getTrakLedDelay(), turnOffTrakLed3);
  return false;
}

bool turnOffTrakLed1(void *) {
  trak1LedOn = false;
  trak1Led.write(0);
  return false;
}

bool turnOffTrakLed2(void *) {
  trak2LedOn = false;
  trak2Led.write(0);
  return false;
}

bool turnOffTrakLed3(void *) {
  trak3LedOn = false;
  trak3Led.write(0);
  return false;
}

unsigned long getTrakLedDelay() {
  unsigned long delay = (1.0 - accelPct) * TRAK_LIGHT_DELAY;
  if (delay > TRAK_LIGHT_DELAY) {
    delay = TRAK_LIGHT_DELAY;
  } else if (delay < 40) {
    delay = 30;
  }
  return delay;
}

bool isTrakLedOn() {
  return trak1LedOn || trak2LedOn || trak3LedOn;
}

void processKeypad() {
  // Fills keypad.key[ ] array with up-to 10 active keys.
  // Returns true if there are ANY active keys.

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

void processFFB() {
  int16_t force = wheel.ffbEngine.calculateForce(wheel.axisWheel);
  force = applyForceLimit(force);
  if (abs(force) < 1200) {
    lShaker.write(0);
    rShaker.write(0);
    return;
  }

  float forcePct = (((float)abs(force)) / ((float)16383));
  int16_t pwm = forcePct * 255;
  if (force < 0) {
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
        break;
      case 5:  //return gains
        memcpy(data, settings.gain, sizeof(settings.gain));
        break;
      case 6:  //return remaining settings
        ((GUI_Report_Settings *)data)->maxvd = round(16384.0 / wheel.ffbEngine.maxVelocityDamperC);
        ((GUI_Report_Settings *)data)->maxvf = round(16384.0 / wheel.ffbEngine.maxVelocityFrictionC);
        ((GUI_Report_Settings *)data)->maxacc = round(16384.0 / wheel.ffbEngine.maxAccelerationInertiaC);

        ((GUI_Report_Settings *)data)->minForce = settings.minForce;
        ((GUI_Report_Settings *)data)->maxForce = settings.maxForce;
        ((GUI_Report_Settings *)data)->cutForce = settings.cutForce;
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
    }
  }
  usbCmd->command = 0;
}

//------------------------- Reading all analog axes ----------------------------------
void readAnalogAxes() {
  wheel.analogAxes[AXIS_ACC]->setValue(analogRead(PIN_ACC));
  getWheelPositionAnalog();
}

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
        bitWrite(*((uint32_t *)d), BTN_1 - 1 + i + sizeof(dpb), btnPressed);
      } else {
        bitWrite(*((uint32_t *)d), BTN_1 - 1 + i, btnPressed);
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
  if ((wheel.buttons & (uint32_t)pow(2, BTN_SHUTDOWN_INDEX)) != 0) {
    System.write(SYSTEM_POWER_DOWN);
  } else if ((wheel.buttons & (uint32_t)pow(2, BTN_ESC_INDEX)) != 0) {
    Keyboard.press(KeyboardKeycode(KEY_ESC));
    delay(5);
    Keyboard.release(KeyboardKeycode(KEY_ESC));
  }
}
//---------------------------------------- end buttons ----------------------------------------------


int32_t getWheelPositionAnalog() {
  wheel.axisWheel->setValue(analogRead(PIN_ST_ANALOG));
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

    settingsE.data.gain[0] = DEFAULT_GAIN;
    settingsE.data.gain[1] = DEFAULT_GAIN;
    settingsE.data.gain[2] = DEFAULT_GAIN;
    settingsE.data.gain[3] = DEFAULT_GAIN;
    settingsE.data.gain[4] = DEFAULT_GAIN;
    settingsE.data.gain[5] = DEFAULT_GAIN;
    settingsE.data.gain[6] = DEFAULT_GAIN;
    settingsE.data.gain[7] = DEFAULT_GAIN;
    settingsE.data.gain[8] = DEFAULT_GAIN;
    settingsE.data.gain[9] = DEFAULT_GAIN;
    settingsE.data.gain[10] = DEFAULT_GAIN;
    settingsE.data.gain[11] = DEFAULT_GAIN;
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
  }

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
