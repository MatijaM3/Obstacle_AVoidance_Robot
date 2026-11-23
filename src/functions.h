#pragma once

#include <Arduino.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#include <HCSR04.h>
#include "simplePID.h"

extern const byte leftForward;
extern const byte leftBackward;
extern const byte rightBackward;
extern const byte rightForward;

extern unsigned long previousMillisPID;
extern volatile unsigned int counterLeft;
extern volatile unsigned int counterRight;
extern float leftRPM;
extern float rightRPM;
extern int stateMain;
extern int stateSub;
extern const int timeDelay;
extern unsigned long previousMillisServo;
extern int distances[3];
extern const int size;

extern SSD1306AsciiAvrI2c oled;
extern HCSR04 hc;
extern PID leftPID;
extern PID rightPID;

void countLeft();//interrupt function to count pulses on right wheel encoder
void countRight();//interrupt function to count pulses on left wheel encoder
unsigned long timeCountPID();//function to ensure PID calculations happen in regular intervals
void calcRPM(int curCounterLeft, int curCounterRight, float measuredTime);//calculates current RPM on left and right wheel
bool timeCountServo(unsigned long millisATM, unsigned int timeDelay);//ensures measurements ahppen after a delay so that the servo settles
int compareDistance(int distances[], int size);//compares measured distances
int measureDistance();//returns measured distance of the ultrasonic sensor
int moveForward();//a transition function to ensure counters, timers and PID are reset when entering move forward state
int turnLeft();//ensures counters are reset before turning left
int turnRight();//ensures counters are reset before turning right
void enterMeasure();//ensures counters and timers are reset before entering measure state
void showOnDisplay(int curState);//updates display
int uTurn();//ensures counters are reset before making a uturn
