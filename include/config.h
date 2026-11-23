#pragma once

#define TURNING_PULSES 15//value of turning pulses to control turn left, right and uturn
#define DELAY 120//Time to collect measurement for PID calculation in millis
#define STATE_MEASURE 0//value of main state control integer to enter measuring state
#define STATE_FORWARD 1//value of main state control integer to enter moving forward state
#define STATE_LEFT 3//value of main state control integer to enter turning left state
#define STATE_RIGHT 4//value of main state control integer to enter turning right state
#define STATE_UTURN 5//value of main state control integer to enter uturn state
#define SUB_INIT 0//value of sub state control integer to enter initialization state
#define STOPPING_DISTANCE 50//front stopping distance
#define RST_PIN -1//defining reset pin
#define I2C_ADDRESS 0x3C//address of dispaly for i2c comunication
