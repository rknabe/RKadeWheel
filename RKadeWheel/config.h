#pragma once
//---------------------------Constants, do not change--------------------
#define ST_ENCODER 0
#define ST_ANALOG 4

#define PT_INTERNAL 0

//-----------------------------------------------------------------------

//---------------------------Configuration-------------------------------

#define SERIAL_BAUDRATE 2000000

//---------------------------Steering axis-------------------------------
//different types of wheel sensor. Choose only one!
//#define STEER_TYPE ST_ENCODER
#define STEER_TYPE ST_ANALOG
//#define STEER_TYPE ST_TLE5010
//#define STEER_TYPE ST_AS5600
//#define STEER_TYPE ST_MLX90316

//settings for encoder
#if STEER_TYPE == ST_ENCODER
#define ENCODER_PIN1 0  //encoder pins must be interrupt pins:[0, 1, 2, 3]
#define ENCODER_PIN2 1
#define ENCODER_PPR 600  //PPR = CPR/4
#endif

//settings for TLE5010
#define TLE5010_PIN_CS 1

//settings for MLX90316
#define MLX90316_PIN_CS 1

//wheel sensor bitdepth. Not supposed to be changed.
#define STEER_BITDEPTH 13
//default wheel range in degrees.
#define WHEEL_RANGE_DEFAULT 270

//transmission ratio (see readme)
#define STEER_TM_RATIO_ENABLED  //Uncomment to enable feature
#if STEER_TYPE == ST_ANALOG
#define STEER_TM_RATIO_MUL -1  //Multiplication factor
#define STEER_TM_RATIO_DIV 1   //Division factor
#define FORCE_RATIO_MUL -1
#else
#define STEER_TM_RATIO_MUL -1  //Multiplication factor
#define STEER_TM_RATIO_DIV 4   //Division factor
#define FORCE_RATIO_MUL -1
#endif

//---------------------------I2C----------------------------------------
//bitbang I2С pins - for MCP23017 and ADS1015
#define I2C_PIN_SDA 2  //any free pins
#define I2C_PIN_SCL 7

#define I2C_DELAY 1
//---------------------------analog axes---------------------------
//analog axes pins
#define PIN_ACC A0
#define PIN_BRAKE A1
#define PIN_CLUTCH A2
//aux analog axes pins
//If aux axis is not needed, comment out corresponding line.
#define PIN_AUX1 A3
#define PIN_AUX2 A4
//#define PIN_AUX3 A5
//#define PIN_AUX4 A6
//#define PIN_AUX5 A7
#if STEER_TYPE == ST_ANALOG
#define PIN_ST_ANALOG A5
#endif

//different ways of connecting pedals. Choose only one!
#define PEDALS_TYPE PT_INTERNAL  //use internal ADC

//settings for internal ADC


//#define AA_PULLUP  //internal ADC with pullups for analog axes
//#define AA_PULLUP_LINEARIZE    //uncomment if need to linearize

//settings for analog multiplexer + 74HC164
#define MP_HC164_PIN_ADATA A0  //analog pin
#define MP_HC164_PIN_SCK 15

//settings for MCP3204
#define MCP3204_CH_ACC 0  //channels for axes
#define MCP3204_CH_BRAKE 1
#define MCP3204_CH_CLUTCH 2

//settings for MCP3204 (SPI)
#define MCP3204_PIN_CS A0

//settings for MCP3204 (4wire)
//v1
#define MCP3204_4W_PIN_SCK A0
#define MCP3204_4W_PIN_MOSI 16
#define MCP3204_4W_PIN_MISO 14

//v2
//#define MCP3204_4W_PIN_SCK  15
//#define MCP3204_4W_PIN_MOSI A0
//#define MCP3204_4W_PIN_MISO A0

//v3
//#define MCP3204_4W_PIN_SCK  A1
//#define MCP3204_4W_PIN_MOSI A0
//#define MCP3204_4W_PIN_MISO A0

//settings for ADS1015
#define ADS1015_CH_ACC 0  //channels for axes
#define ADS1015_CH_BRAKE 1
#define ADS1015_CH_CLUTCH 2

//---------------------------Smoothing-----------------------------------
/*
 * Smoothing is performed with moving average filter.
 * Level means filter window size as power of 2.
 * 2 means averaging 4 values, 3 - 8 values and so on.
 */
//Smoothing for wheel axis.
#define MA_LEVEL_WHEEL_POSITION 4
#define MA_LEVEL_WHEEL_VELOCITY 4
#define MA_LEVEL_WHEEL_ACCELERATION 4

//Level of smoothing for analog axes.
#define MA_LEVEL_AXIS_ACC 4
#define MA_LEVEL_AXIS_BRAKE 4
#define MA_LEVEL_AXIS_CLUTCH 4
#define MA_LEVEL_AXIS_AUX1 4
#define MA_LEVEL_AXIS_AUX2 4
#define MA_LEVEL_AXIS_AUX3 4
#define MA_LEVEL_AXIS_AUX4 4
#define MA_LEVEL_AXIS_AUX5 4
#define MA_LEVEL_AXIS_ST_ANALOG 4

//----------------------------Buttons-------------------------------------
//different ways of connecting buttons. Choose only one!
//#define BUTTONS_TYPE BT_74HC165       //Use 74HC165 shift registers
//#define BUTTONS_TYPE BT_MCP23017    //Use MCP23017 I2C port expanders
//#define BUTTONS_TYPE BT_CD4021B     //Use CD4021B shift registers
//#define BUTTONS_TYPE BT_PCF857x     //Use PCF857x I2C port expanders
#define BUTTONS_TYPE BT_NONE  //No buttons


//settings for 74HC165
#define HC165_PIN_SCK 15
#define HC165_PIN_DATA1 2  //pin for DATA#1
#define HC165_PIN_DATA2 7  //pin for DATA#2
#define HC165_PIN_PL 3     //pin for PL (comment this line if using RC to omit PL line)

//settings for CD4021
#define CD4021_PIN_SCK 15
#define CD4021_PIN_DATA1 2  //pin for DATA#1
#define CD4021_PIN_DATA2 7  //pin for DATA#2
#define CD4021_PIN_PL 3     //pin for PL (comment this line if using RC to omit PL line)

//settings for MCP23017
#define MCP23017_ADDR1 0x20
#define MCP23017_ADDR2 0x21

//settings for PCF857x
#define PCF857x_L1_TYPE PCF8575
#define PCF857x_L1_ADDR1 0x20
#define PCF857x_L1_ADDR2 0x21

#define PCF857x_L2_TYPE PCF8574
#define PCF857x_L2_ADDR1 0x22
#define PCF857x_L2_ADDR2 0x23

//buttons directly connected to pins
#define DPB  //Enable
#if STEER_TYPE == ST_ANALOG
#define DPB_PINS 0, 1, 2, 3, 4, 6, 7, 8, 11, 12, 13, 14, 15, 16
#else
#define DPB_PINS 2, 3, 4, 6, 7, 8, 11, 12, 13, 14, 15, 16
#endif
#define DPB_1ST_BTN 1
#define GEAR_BTN_IDX_1 0
#define GEAR_BTN_IDX_2 1
#define GEAR_BTN_IDX_3 2
#define GEAR_BTN_IDX_4 3
#define GEAR_BTN_IDX_5 30
#define GEAR_BTN_IDX_6 31

#define BTN_ESC_INDEX 12       //zero-based index, 15
#define BTN_SHUTDOWN_INDEX 13  //zero-based index, 16

//Hat switch
//#define HATSWITCH
#define HAT_BTN_UP 13
#define HAT_BTN_DOWN 15
#define HAT_BTN_LEFT 14
#define HAT_BTN_RIGHT 16
#define HAT_CLR_BTNS  //clear buttons state

//----------------------------FFB settings-------------------------------
//rknabe, use motor option 2, enabled next line
#define MOTOR_ENABLE_PIN 5  //if is set, selected pin will output 1 when FFB is active and 0 otherwise.
#define MOTOR_OUTPUT_PIN1 9
#define MOTOR_OUTPUT_PIN2 10

//default FFB PWM bitdepth
#define DEFAULT_FFB_BITDEPTH 9  //15.6 KHz

//Effect parameters
#define DEFAULT_MAX_VELOCITY 2500
#define DEFAULT_MAX_ACCELERATION 2500

#define DEFAULT_ENDSTOP_OFFSET 0    //force level endstop effect will start from (0-16383). Increasing will make endstop harder.
#define DEFAULT_ENDSTOP_WIDTH 1024  //length of excess position where endstop effect will rise to maximum level. Decreasing makes endstop harder.

//Auto find center
//#define AFC_ON           //Uncomment to enable autofind center at start. Requires presence of mechanical limiters (see description)
#define AFC_FORCE 4200   //Force [0...16383] to exert when finding center.
#define AFC_PERIOD 80    //default 50         //Position check period in milliseconds
#define AFC_TRESHOLD 10  //default 10         //Minimum position change to detect movement
#define AFC_NORANGE      //Uncomment to disable range setting
#define AFC_RANGE_FIX 5  //range will be decreased by this value (in degrees), to prevent wheel kicking on limiters.
