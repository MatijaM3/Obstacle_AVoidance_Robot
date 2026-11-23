#include <Arduino.h>
#include "config.h"
#include "functions.h"
#define DEBOUNCE_TIME 5//minimum time between wheel encoder pulses

bool firstPulseLeft=true;//flag to check whether first pulse has already happened on left wheel encoder
bool firstPulseRight=true;//flag to check whether first pulse has already happened on right wheel encoder
unsigned long lastMillisPulseLeft=0;//Time when last left encoder pulse has happened
unsigned long lastMillisPulseRight=0;//Time when last right encoder pulse has happened

void countLeft()
{
    unsigned long curMillis=millis();

    if(firstPulseLeft||(curMillis-lastMillisPulseLeft>=DEBOUNCE_TIME)) {
        counterLeft++;
        lastMillisPulseLeft=curMillis;
        firstPulseLeft=false;
    }
}

void countRight()
{
    unsigned long curMillis=millis();

    if(firstPulseRight||(curMillis-lastMillisPulseRight>=DEBOUNCE_TIME)) {
        counterRight++;
        lastMillisPulseRight=curMillis;
        firstPulseRight=false;
    }
}

unsigned long timeCountPID()
{
    return millis()-previousMillisPID;
}

void calcRPM(int curCounterLeft, int curCounterRight, float measuredTime)
{
    leftRPM=(curCounterLeft*3000.0f)/measuredTime;
    rightRPM=(curCounterRight*3000.0f)/measuredTime;

    Serial.print("Right RPM: ");
    Serial.println(rightRPM);
}

bool timeCountServo(unsigned long millisATM, unsigned int timeDelay)
{
    if(millisATM-previousMillisServo>=timeDelay) {
        previousMillisServo=millis();
        return true;
    }

    return false;
}

int compareDistance(int distances[], int size)
{
    int furthest=-1;

    for(int i=0; i<size; i++) {
        if(distances[i]>STOPPING_DISTANCE&&
           (furthest == -1 || distances[i] > distances[furthest])) {
            furthest = i;
        }
    }
    switch(furthest) 
    {
        case 0:
            return moveForward();

        case 1:
            return turnLeft();

        case 2:
            return turnRight();
    }

    return uTurn();
}

int measureDistance()
{
        return hc.dist();
}

int moveForward()
{
    noInterrupts();
    counterLeft=0;
    counterRight=0;
    interrupts();

    leftPID.reset();
    rightPID.reset();

    previousMillisPID=millis();

    return STATE_FORWARD;
}

int turnLeft()
{
    noInterrupts();
    counterLeft=0;
    interrupts();

    digitalWrite(leftBackward,HIGH);

    return STATE_LEFT;
}

int turnRight()
{
    noInterrupts();
    counterRight=0;
    interrupts();

    digitalWrite(rightBackward,HIGH);

    return STATE_RIGHT;
}

void enterMeasure()
{
    noInterrupts();
    counterLeft=0;
    counterRight=0;
    interrupts();
    stateMain=STATE_MEASURE;
    stateSub=SUB_INIT;

    previousMillisServo=millis();

    Serial.println("Entered Measure");
}
void showOnDisplay(int curState)
{
    oled.clear();
    if(curState==STATE_FORWARD){
        oled.print("Going forward!");
    }
    else if(curState==STATE_LEFT){
        oled.print("Turning left!");
    }
    else if(curState==STATE_RIGHT){
        oled.print("Turning right!");
    }
    else {
        oled.print("Measuring!");
    }
}
int uTurn()
{
  noInterrupts();
  counterLeft=0;
  counterRight=0;
  interrupts();
  digitalWrite(rightBackward, HIGH);
  digitalWrite(leftForward, HIGH);
  return STATE_UTURN;
}