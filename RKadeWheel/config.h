#pragma once

//---------------------------Configuration-------------------------------

#define SERIAL_BAUDRATE 115200

//---------------------------Steering axis-------------------------------
//wheel sensor bitdepth. Not supposed to be changed.
#define STEER_BITDEPTH 13
//default wheel range in degrees.
#define WHEEL_RANGE_DEFAULT 270

//transmission ratio (see readme)
#define STEER_TM_RATIO_ENABLED  //Uncomment to enable feature
#define STEER_TM_RATIO_MUL -1   //Multiplication factor
#define STEER_TM_RATIO_DIV 1    //Division factor
#define FORCE_RATIO_MUL -1

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
#define PIN_ST_ANALOG A5

#define DEFAULT_AA_MIN 0
#define DEFAULT_AA_MAX 1023

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

//#define DPB_PINS 0, 1, 2, 3, 4, 6, 7, 8, 11, 12, 13, 14, 15, 16
#define DPB_PINS 0, 1, 4, 6, 7, 8, 11, 12, 13, 14, 15, 16
#define DPB_1ST_BTN 1
#define GEAR_BTN_IDX_1 0
#define GEAR_BTN_IDX_2 1
#define GEAR_BTN_IDX_3 2
#define GEAR_BTN_IDX_4 3
#define GEAR_BTN_IDX_5 30
#define GEAR_BTN_IDX_6 31

#define BTN_ESC_INDEX 11  //zero-based index, 16

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
