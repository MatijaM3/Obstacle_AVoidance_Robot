#include <Arduino.h>
#include "simplePID.h"
#include <Servo.h>
#include <HCSR04.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#include "functions.h"
#include "config.h"

#define STATE_STOP 2//value of state control integer to enter stopping state
#define SUB_FORWARD 1//value of sub state control integer to enter measure forward state
#define SUB_LEFT 2//value of sub state control integer to enter measure left state
#define SUB_RIGHT 3//value of sub state control integer to enter measure right state
#define SUB_COMPARE 4//value of sub state control integer to enter compare state

const byte interruptPinLeft = 3;//interrupt pin for left wheel encoder
const byte interruptPinRight = 2;//interrupt pin for right wheel encoder
const byte leftForward=6;//output pin to control left forward motion
const byte leftBackward=5;//output pin to control left backwards motion
const byte rightForward=11;//output pin to control right forward motion
const byte rightBackward=10;//output pin to control right backwards motion

unsigned long previousMillisPID=0;//previous pid calculation time

volatile unsigned int counterLeft=0;//pulse counter for left wheel encoder
volatile unsigned int counterRight=0;//pulse counter for left wheel encoder

float leftRPM;//current rpm on left wheel
float rightRPM;//current rpm on right wheel
const float setpoint = 140.0f;//desired rpm 

int stateMain=STATE_MEASURE;//main state control integer
int stateSub=SUB_INIT;//sub state control integer
int previousState=-1;//previous main state

const int servoLeft=110;//servo turn left angle
const int servoFront=65;//servo turn front angle
const int servoRight=20;//servo turn right angle
const int timeDelay=2000;//dleay between servo moving and distance sensor measuring

unsigned long previousMillisServo=0;//previous distance measurement time 
int distances[3];//array to store measured distances
const int size=3;//array size


SSD1306AsciiAvrI2c oled;//object to set up oled display
Servo myservo;//object to control servo
HCSR04 hc(13, 12);//attaching IO pins to receive and send signal through ultrasonic sensors

PID leftPID(1.0f, 0.2f, 0.0f);//external
PID rightPID(1.0f, 0.2f, 0.0f);//external

void setup() {
    Serial.begin(9600);
  //pin setup
    pinMode(leftForward, OUTPUT);
    pinMode(leftBackward, OUTPUT);
    pinMode(rightForward, OUTPUT);
    pinMode(rightBackward, OUTPUT);

    pinMode(interruptPinLeft, INPUT_PULLUP);
    pinMode(interruptPinRight, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(interruptPinLeft), countLeft, FALLING);
    attachInterrupt(digitalPinToInterrupt(interruptPinRight), countRight, FALLING);

    myservo.attach(9);
    //oled setup
    #if RST_PIN >= 0
    oled.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);
    #else // RST_PIN >= 0
    oled.begin(&Adafruit128x32, I2C_ADDRESS);
    #endif // RST_PIN >= 0
    oled.setI2cClock(100);
    oled.setFont(Adafruit5x7);
    oled.clear();
    oled.setCursor(10, 1);
}

void loop() {

     if (stateMain!= previousState) {
        showOnDisplay(stateMain);
        previousState = stateMain;
    }

    if(stateMain==STATE_FORWARD) {

        float measuredTime=timeCountPID();

        if(measuredTime>=DELAY) {

            noInterrupts();
            int currentCountLeft = counterLeft;
            int currentCountRight = counterRight;

            counterLeft=0;
            counterRight=0;
            interrupts();

            previousMillisPID=millis();

            calcRPM(currentCountLeft, currentCountRight, measuredTime);

            int leftPWM=constrain(leftPID.compute(setpoint, leftRPM, measuredTime/1000.0f), 0, 255);
            int rightPWM=constrain(rightPID.compute(setpoint, rightRPM, measuredTime/1000.0f), 0, 255);

            analogWrite(leftForward, leftPWM);
            analogWrite(rightForward, rightPWM);
        }
    }

    switch(stateMain) {

        case STATE_MEASURE:
        {
            switch(stateSub) {

                case SUB_INIT:
                {
                    myservo.write(servoFront);

                    previousMillisServo=millis();

                    stateSub=SUB_FORWARD;

                    break;
                }

                case SUB_FORWARD:
                {
                    

                    myservo.write(servoFront);

                    if(timeCountServo(millis(),timeDelay)) {
                        distances[0]=hc.dist();

                        stateSub=SUB_LEFT;

                        Serial.print("Distance front: ");
                        Serial.println(distances[0]);
                    }

                    break;
                }

                case SUB_LEFT:
                {
                   

                    myservo.write(servoLeft);

                    if(timeCountServo(millis(),timeDelay)) {
                        distances[1]=hc.dist();

                        stateSub=SUB_RIGHT;

                        //Serial.print("Distance left: ");
                        Serial.println(distances[1]);
                    }

                    break;
                }

                case SUB_RIGHT:
                {
                    myservo.write(servoRight);

                    if(timeCountServo(millis(),timeDelay)) {
                        distances[2]=hc.dist();

                        stateSub=SUB_COMPARE;

                        //Serial.print("Distance right: ");
                        //Serial.println(distances[2]);
                    }

                    break;
                }

                case SUB_COMPARE:
                {
                    myservo.write(servoFront);

                    if(timeCountServo(millis(),timeDelay)) {
                        stateSub=SUB_FORWARD;
                        stateMain=compareDistance(distances, size);
                        //Serial.println(stateMain);
                    }

                    break;
                }
            }

            break;
        }

        case STATE_FORWARD:
        {
            int distance=measureDistance();

            if(distance>0&&distance<=STOPPING_DISTANCE) {
                stateMain=STATE_STOP;
            }

            /*Serial.print("Distance front: ");
            Serial.println(distances[0]);
            Serial.println("In state_forward");
            Serial.print("Next state");
            Serial.println(stateMain);*/

            break;
        }

        case STATE_STOP:
        {
            analogWrite(leftForward,0);
            analogWrite(rightForward,0);

            enterMeasure();

            break;
        }

        case STATE_LEFT:
        {
            unsigned int pulses;

            noInterrupts();
            pulses=counterLeft;
            interrupts();

            if(pulses>=TURNING_PULSES) {
                digitalWrite(leftBackward, LOW);

                enterMeasure();
            }

            break;
        }

        case STATE_RIGHT:
        {
            unsigned int pulses;

            noInterrupts();
            pulses=counterRight;
            interrupts();

            if(pulses>=TURNING_PULSES) {
                digitalWrite(rightBackward, LOW);

                enterMeasure();
            }

            break;
        }
        case STATE_UTURN:
        {
          unsigned int pulsesRight;
          unsigned int pulsesLeft;
          noInterrupts();
          pulsesRight=counterRight;
          pulsesLeft=counterLeft;
          interrupts();
          if(pulsesRight>=TURNING_PULSES*2&&pulsesLeft>=TURNING_PULSES*2)
          {
          digitalWrite(rightBackward, LOW);
          digitalWrite(leftForward, LOW);

          enterMeasure();
          }
          break;
        }
    }
}
