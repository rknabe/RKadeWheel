#ifndef KEYPAD_H
#define KEYPAD_H

#include "Key.h"
#include <PCF8574.h>

#ifndef INPUT_PULLUP
#warning "Using  pinMode() INPUT_PULLUP AVR emulation"
#define INPUT_PULLUP 0x2
#define pinMode(_pin, _mode) _mypinMode(_pin, _mode)
#define _mypinMode(_pin, _mode) \
  do { \
    if (_mode == INPUT_PULLUP) \
      pinMode(_pin, INPUT); \
    digitalWrite(_pin, 1); \
    if (_mode != INPUT_PULLUP) \
      pinMode(_pin, _mode); \
  } while (0)
#endif

#define OPEN LOW
#define CLOSED HIGH

typedef char KeypadEvent;
typedef unsigned int uint;
typedef unsigned long ulong;

typedef struct {
  byte rows;
  byte columns;
} KeypadSize;

#define LIST_MAX 3  // Max number of keys on the active list.
#define MAPSIZE 4   // MAPSIZE is the number of rows (times 16 columns)
#define makeKeymap(x) ((char *)x)

//class Keypad : public Key, public HAL_obj {
class Keypad : public Key {
public:

  Keypad(PCF8574 *keypadIO, char *userKeymap, byte *row, byte *col, byte numRows, byte numCols);

  virtual void pin_mode(byte pinNum, byte mode) {
    pinMode(pinNum, mode);
  }
  virtual void pin_write(byte pinNum, boolean level) {
    digitalWrite(pinNum, level);
  }
  virtual int pin_read(byte pinNum) {
    return digitalRead(pinNum);
  }

  uint bitMap[MAPSIZE];  // 10 row x 16 column array of bits. Except Due which has 32 columns.
  Key key[LIST_MAX];
  unsigned long holdTimer;

  char getKey();
  bool getKeys();
  KeyState getState();
  void begin(char *userKeymap);
  bool isPressed(char keyChar);
  void setDebounceTime(uint);
  void setHoldTime(uint);
  //void addEventListener(void (*listener)(char));
  int findInList(char keyChar);
  int findInList(int keyCode);
  //char waitForKey();
  bool keyStateChanged();
  byte numKeys();

private:
  unsigned long startTime;
  char *keymap;
  byte *rowPins;
  byte *columnPins;
  PCF8574 *keypadIO;
  KeypadSize sizeKpd;
  uint debounceTime;
  uint holdTime;
  bool single_key;

  void scanKeys();
  bool updateList();
  void nextKeyState(byte n, boolean button);
  void transitionTo(byte n, KeyState nextState);
  //void (*keypadEventListener)(char);
};

#endif
