#pragma once

#define SERIAL_BAUDRATE 2000000

//---------------------------Steering axis-------------------------------

//default wheel range in degrees.
#define WHEEL_RANGE_DEFAULT 270

//transmission ratio (see readme)
#define STEER_TM_RATIO_ENABLED  //Uncomment to enable feature
#define STEER_TM_RATIO_MUL -1   //Multiplication factor
#define STEER_TM_RATIO_DIV 1    //Division factor
#define FORCE_RATIO_MUL -1

//---------------------------pins---------------------------
//analog axes pins
#define PIN_ACC A0
#define PIN_ST_ANALOG A5

//----------------------------Buttons-------------------------------------
#define BUTTON_PINS 0, 1, 2, 4, 7, 8, 12, 14, 15, 16, A1, A2, A3, A4
#define BTN_1 1
#define BTN_ESC_INDEX 12       //zero-based index
#define BTN_SHUTDOWN_INDEX 13  //zero-based index

#define LEFT_SHAKER_PIN 9
#define RIGHT_SHAKER_PIN 10
#define BLOWER_PIN 11
#define BRAKE_LIGHT_PIN 3
#define TRAK1_LIGHT_PIN 5
#define TRAK2_LIGHT_PIN 6
#define TRAK3_LIGHT_PIN 13

//different ways of connecting pedals. Choose only one!
#define DEFAULT_AA_MIN 0
#define DEFAULT_AA_MAX 1023
#define DEFAULT_GAIN 1024

//---------------------------Smoothing-----------------------------------
/*
 * Smoothing is performed with moving average filter.
 * Level means filter window size as power of 2.
 * 2 means averaging 4 values, 3 - 8 values and so on.
 */
//Smoothing for wheel axis.
#define MA_LEVEL_WHEEL_VELOCITY 4
#define MA_LEVEL_WHEEL_ACCELERATION 4

//Level of smoothing for analog axes.
#define MA_LEVEL_AXIS_ACC 4
#define MA_LEVEL_AXIS_ST_ANALOG 4

#define MAX_BLOWER_PWM 126
#define MAX_LIGHT_PWM 126
#define TRAK_LIGHT_DELAY 250

//----------------------------FFB settings-------------------------------

//default FFB PWM bitdepth
#define DEFAULT_FFB_BITDEPTH 9  //15.6 KHz

//Effect parameters
#define DEFAULT_MAX_VELOCITY 2500
#define DEFAULT_MAX_ACCELERATION 2500

#define DEFAULT_ENDSTOP_OFFSET 0    //force level endstop effect will start from (0-16383). Increasing will make endstop harder.
#define DEFAULT_ENDSTOP_WIDTH 1024  //length of excess position where endstop effect will rise to maximum level. Decreasing makes endstop harder.
