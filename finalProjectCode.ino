#include <Wire.h>

#include <LiquidCrystal.h>

#include <L3G.h>

/**
 * RBE 2002 Team 8 Project Code
 * Team Members: Everett Johnson, Leif Sahyun, Orion Strickland
 * Date Created: Nov 12, 2017
 * Last Modified: Nov 25, 2017 by Leif Sahyun
 * Description:
 * 
 * Input Pins:
 * Output Pins:
 */

#include <TimerThree.h>
//#include "OdometryProject.h"
#include "PidControl.h"
#include "Ultrasonic.h"
#define FLAME_DIFF A0

enum State
  {Calibration,
  WallFollowing,
  TurnRight,
  TurnLeft,
  Align,
  TurnToFlame,
  ApproachFlame,
  FanOn,
  TurnToStart,
  MoveForward,
  Reverse,
  TurnToWall,
  WallFollowingStage2,
  TurnLeftStage2,
  Finish};

struct odometer{
  float x;
  float y;
  float heading;
} pos;

State currentState = Calibration;
State lastState = Calibration;
const int buttonPin = 12;
const int motorRightForwardPin = 4;
const int motorRightReversePin = 5;
const int motorLeftForwardPin = 6;
const int motorLeftReversePin = 7;
const int FORWARD = 254;
const int STALL = 127;
const int REVERSE = 0;
int motorRightSpeed = 0;
int motorLeftSpeed = 0;
//Servo motorLeft;  //attached to pin 4
//Servo motorRight; //attached to pin 5
const int fanPin = 8;
const int leftEncPin1 = 50;
const int leftEncPin2 = 51;
const int leftEncInt1 = PCINT3;
const int leftEncInt2 = PCINT2;
long leftEncCount = 0;
const int rightEncPin1 = 52;
const int rightEncPin2 = 53;
const int rightEncInt1 = PCINT1;
const int rightEncInt2 = PCINT0;
long rightEncCount = 0;
byte lastReadings = 0;
L3G gyro;
LiquidCrystal lcd(40,41,42,43,44,45);
Ultrasonic frontUltra(22, 18);
Ultrasonic sideUltra(23, 19);
Ultrasonic rearUltra(24, 2);
float forwardInput=0;
float distInput=0;
float alignInput=0;
float flameInput=0;
float turningInput=0;
float approachInput=0;
float gyroError=0;
float G_gain = .002625;
float gyroHeading = 0;
const float wallDist = 5;
const int hallWidth = 20;
const int gapWidth = 20;
PidControl distController(&distInput, wallDist);
PidControl alignController(&alignInput, 0);
PidControl turningController(&gyroHeading,0);
PidControl forwardController(&forwardInput,0);
PidControl flameController(&flameInput,0);
PidControl approachController(&approachInput,0);
const int lightSensorPin = 8;
const int topFlamePin = 3;
int motorDiff;
int motorSpeed;
unsigned long timer = 0;
const float encTicksPerInch = -694.5;
bool odometryEnabled = true;
unsigned long calibrationTimer = 0;
bool stopped=true;
int gyroCounter =0;
bool flameFlag = false;

void flameInterrupt(){
  flameFlag = true;
}

ISR(PCINT0_vect){
  byte readings = 0;
  if(digitalRead(leftEncPin1))
    readings |= bit(leftEncInt1);
  if(digitalRead(leftEncPin2))
    readings |= bit(leftEncInt2);
  if(digitalRead(rightEncPin1))
    readings |= bit(rightEncInt1);
  if(digitalRead(rightEncPin2))
    readings |= bit(rightEncInt2);
  byte changes = lastReadings^readings;
  switch (changes){
    case bit(leftEncInt1):
      if(readings & bit(leftEncInt1)){
        if(readings & bit(leftEncInt2))
          leftEncCount++;
        else
          leftEncCount--;
      }else {
        if(readings & bit(leftEncInt2))
          leftEncCount--;
        else
          leftEncCount++;
      }
    break;
    case bit(leftEncInt2):
      if(readings & bit(leftEncInt2)){
        if(readings & bit(leftEncInt1))
          leftEncCount++;
        else
          leftEncCount--;
      }else {
        if(readings & bit(leftEncInt1))
          leftEncCount--;
        else
          leftEncCount++;
      }
    break;
    case bit(rightEncInt1):
      if(readings & bit(rightEncInt1)){
        if(readings & bit(rightEncInt2))
          leftEncCount++;
        else
          leftEncCount--;
      }else {
        if(readings & bit(rightEncInt2))
          leftEncCount--;
        else
          leftEncCount++;
      }
    break;
    case bit(rightEncInt2):
      if(readings & bit(rightEncInt2)){
        if(readings & bit(rightEncInt1))
          leftEncCount++;
        else
          leftEncCount--;
      }else {
        if(readings & bit(leftEncInt1))
          leftEncCount--;
        else
          leftEncCount++;
      }
    break;
  }
}

void setup() {
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(43, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(50, INPUT);
  pinMode(51, INPUT);
  pinMode(52, INPUT);
  pinMode(53, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);
  lcd.begin(16,2);
  Wire.begin();
  gyro.init();
  gyro.enableDefault();
  /*distController.toPrint = true;
  alignController.toPrint = true;
  turningController.toPrint = true;
  forwardController.toPrint = true;*/
  forwardController.kp = 25;
  forwardController.ki = -0.00001;
  forwardController.defaultTolerance = 1;
  approachController.defaultTolerance = 1;
  turningController.ki = 0;
  gyro.read();
  gyroError += gyro.g.z;
  PCMSK0 |= bit(leftEncInt1) | bit (leftEncInt2) | bit(rightEncInt1) | bit(rightEncInt2); //enable bits corresponding to encoder pins in the interrupt enable mask register
  PCICR  |= bit (PCIE0); //enable bit corresponding to pins 10-13 and 50-53 in the interrupt enable register.
  attachInterrupt(digitalPinToInterrupt(topFlamePin), flameInterrupt, FALLING);
  //Timer3.initialize();
  //Timer3.attachInterrupt(synchronousUpdate, 100);
  //OdometryProject::zero();
  //motorLeft.attach(4);
  //motorRight.attach(5);
}

void loop() {
  Serial.println("here");
  Serial.println(currentState);
  Serial.println(digitalRead(buttonPin));
  synchronousUpdate();
  if(currentState==Calibration)
    return;
  safeTriggerAllUltras();
  updateGyro();
  //Serial.print("DISTANCE FRONT: ");
  //Serial.println(frontUltra.getLastDist());
  //Serial.print("DISTANCE SIDE: ");
  //Serial.println(sideUltra.getLastDist());
  //Serial.print("DISTANCE REAR: ");
  //Serial.println(rearUltra.getLastDist());
}

void motorRightWrite(int i){
  analogWrite(motorRightForwardPin, i);
  analogWrite(motorRightReversePin, FORWARD-i);
}

void motorLeftWrite(int i){
  analogWrite(motorLeftForwardPin, i);
  analogWrite(motorLeftReversePin, FORWARD-i);
}

void scaleToMaxSpeed(){
  double scalingFactor;
  if(motorRightSpeed>motorLeftSpeed){
    scalingFactor = 128.0/(motorRightSpeed-STALL);
    motorRightSpeed = FORWARD;
    motorLeftSpeed = (int)(STALL+(motorLeftSpeed-STALL)*scalingFactor);
  } else {
    scalingFactor = 128.0/(motorLeftSpeed-STALL);
    motorLeftSpeed = FORWARD;
    motorRightSpeed = (int)(STALL+(motorRightSpeed-STALL)*scalingFactor);
  }
}

void updateGyro(){
  gyro.read();
  gyroHeading += (gyro.g.z - gyroError)*G_gain;
  lcd.clear();
  lcd.print((int)gyroHeading);
  lcd.print(" ");
  lcd.print(currentState);
  lcd.print(" ");
  lcd.print((int)turningController.getError());
}

int gyroSnapToWall(int input){
  int actualHeading = input%360;
  if(actualHeading<0)
    actualHeading += 360;
  if(actualHeading>315||actualHeading<=45)
    return 0;
  else if(actualHeading>45&&actualHeading<=135)
    return 90;
  else if(actualHeading>135&&actualHeading<=225)
    return 180;
  else if(actualHeading>225&&actualHeading<=315)
    return 270;
}

void positionUpdate(){
  float dist = (leftEncCount + rightEncCount)/2/encTicksPerInch;
  forwardInput+=dist;
  pos.x+=dist*cos(pos.heading);
  pos.y+=dist*sin(pos.heading);
  pos.heading = gyroHeading;
  leftEncCount = 0;
  rightEncCount = 0;
}

void synchronousUpdate(){
  if(odometryEnabled)
    positionUpdate();
  if(checkStateChange())
  {
    endState();
    beginState();
  }
  lastState = currentState;
  continueState();
  distController.updatePid();
  alignController.updatePid();
  turningController.updatePid();
  forwardController.updatePid();
  if(!stopped){
    motorRightWrite(constrain(motorRightSpeed, REVERSE, FORWARD));
    motorLeftWrite(constrain(motorLeftSpeed, REVERSE, FORWARD));
  } else {
    motorRightWrite(127);
    motorLeftWrite(127);
  }
}

/**
 * This method requires at least 300 milliseconds to function
 */
void safeTriggerAllUltras(){
  int failCount;
  failCount = 0;
  timer = millis();
  while(millis()<timer+100);
  while(!frontUltra.trigger()){
    failCount++;
    if(failCount>10){
      Ultrasonic::triggeredObj = NULL;
    }
  }
  failCount = 0;
  timer = millis();
  while(millis()<timer+100);
  while(!sideUltra.trigger()){
    failCount++;
    if(failCount>10){
      Ultrasonic::triggeredObj = NULL;
    }
  }
  failCount = 0;
  timer = millis();
  while(millis()<timer+100);
  while(!rearUltra.trigger()){
    failCount++;
    if(failCount>10){
      Ultrasonic::triggeredObj = NULL;
    }
  }
}

void wallFollowingSequence(){
  noInterrupts();
  distInput = (sideUltra.getLastDist() + rearUltra.getLastDist())/2;
  alignInput = sideUltra.getLastDist()-rearUltra.getLastDist();
  interrupts();
  int distOut = constrain(distController.outSpeed, -48, 48);
  int alignOut = constrain(alignController.outSpeed, -16, 16);
  if(alignController.isInTolerance(1))
    gyroHeading = gyroSnapToWall(gyroHeading);
  motorRightSpeed = STALL + 64+distOut+alignOut;
  motorLeftSpeed = STALL + 64-distOut-alignOut;
  scaleToMaxSpeed(); 
}

void forwardSequence(){
  motorRightSpeed = STALL + constrain(forwardController.outSpeed,-STALL,STALL);
  motorLeftSpeed = motorRightSpeed;
}

void turningSequence(){
  motorRightSpeed = STALL-(int)constrain(turningController.outSpeed, -STALL, STALL);
  motorLeftSpeed = STALL+(int)constrain(turningController.outSpeed, -STALL, STALL);
}

void flameTurningSequence(){
  flameInput = analogRead(FLAME_DIFF);
  motorRightSpeed = STALL-(int)constrain(flameController.outSpeed, -STALL, STALL);
  motorLeftSpeed = STALL+(int)constrain(flameController.outSpeed, -STALL, STALL);
}

void flameApproachSequence(){
  approachInput = frontUltra.getLastDist();
  motorRightSpeed = STALL-(int)constrain(approachController.outSpeed, -STALL, STALL);
  motorLeftSpeed = STALL-(int)constrain(approachController.outSpeed, -STALL, STALL);
}

void continueState(){
  switch (currentState)
    {
      case Calibration:
      if(millis() < calibrationTimer+2000){
        gyro.read();
        gyroError += gyro.g.z;
        gyroCounter++;
      }
      break;
      case WallFollowing:
        wallFollowingSequence();
      break;
      case TurnRight:
        turningSequence();
      break;
      case TurnLeft:
        turningSequence();
      break;
      /*case Align:
        turningSequence();
      break;*/
      case TurnToFlame:
        flameTurningSequence();
      break;
      case ApproachFlame:
        flameApproachSequence();
      break;
      case FanOn:

      break;
      case TurnToStart:

      break;
      case MoveForward:
        forwardSequence();
      break;
      case Reverse:
        forwardSequence();
      break;
      case TurnToWall:

      break;
      case WallFollowingStage2:
        wallFollowingSequence();
      break;
      case TurnLeftStage2:

      break;
      case Finish:

      break;
    }
}

bool checkStateChange(){
  if(!digitalRead(lightSensorPin)){
    currentState = Reverse;
    return true;
  } else if (flameFlag){
    currentState = TurnToFlame;
    return true;
  }
  switch (currentState)
    {
      case Calibration:
        if(digitalRead(buttonPin)==0 && millis()>calibrationTimer+2000){
          currentState = WallFollowing;
          return true;
        }
      break;
      case WallFollowing:
        if(frontUltra.getLastDist()<wallDist+1){
          currentState = TurnLeft;
          return true;
        }
        else if(sideUltra.getLastDist()>hallWidth){
          currentState = MoveForward; //Align
          return true;
        }
      break;
      case TurnRight:
        if(turningController.isInTolerance()){
          currentState = WallFollowing;
          return true;
        }
      break;
      case TurnLeft:
        if(turningController.isInTolerance()){
          if(sideUltra.getLastDist()>hallWidth/2)
            currentState = MoveForward;
          currentState = WallFollowing;
          return true;
        }
      break;
      /*case Align:
        if(turningController.isInTolerance()){
          currentState = MoveForward;
          return true;
        }
      break;*/
      case TurnToFlame:
        if(flameController.isInTolerance()){
          currentState = ApproachFlame;
          return true;
        }
      break;
      case ApproachFlame:
        if(approachController.isInTolerance()){
          currentState = FanOn;
          return true;
        }
      break;
      case FanOn:

      break;
      case TurnToStart:

      break;
      case MoveForward:
        if(forwardController.isInTolerance()){
          if(sideUltra.getLastDist()<hallWidth/2){
            currentState = WallFollowing;
          }else{
            currentState = TurnRight;
          }
          return true;
        }
      break;
      case Reverse:
        if(forwardController.isInTolerance()){
          currentState = TurnLeft;
          return true;
        }
      break;
      case TurnToWall:

      break;
      case WallFollowingStage2:

      break;
      case TurnLeftStage2:

      break;
      case Finish:

      break;
    }
  return false;
}

void beginState(){
  switch (currentState)
    {
      case Calibration:
        calibrationTimer = millis();
      break;
      case WallFollowing:
        distController.setGoal(wallDist);
        alignController.setGoal(0);
      break;
      case TurnRight:
        turningController.setGoal(gyroHeading+90);
      break;
      case TurnLeft:
        turningController.setGoal(gyroHeading-90); //maybe want this to be 100
      break;
      /*case Align:
        turningController.setGoal(gyroSnapToWall(gyroHeading));
      break;*/
      case TurnToFlame:
        flameController.setGoal(0);
      break;
      case ApproachFlame:
        approachController.setGoal(2);
      break;
      case FanOn:

      break;
      case TurnToStart:

      break;
      case MoveForward:
        forwardInput = 0;
        forwardController.setGoal(12.0);
      break;
      case Reverse:
        forwardInput = 0;
        forwardController.kp = -forwardController.kp;
        forwardController.ki = -forwardController.ki;
        forwardController.kd = -forwardController.kd;
        forwardController.setGoal(6.0);
      break;
      case TurnToWall:

      break;
      case WallFollowingStage2:
        distController.setGoal(wallDist);
        alignController.setGoal(0);
      break;
      case TurnLeftStage2:
        turningController.setGoal(gyroHeading-90);
      break;
      case Finish:

      break;
    }
}

void endState(){
  switch (lastState)
    {
      case Calibration:
        gyroError = gyroError/gyroCounter;
        stopped = false;
      break;
      case WallFollowing:

      break;
      case TurnRight:

      break;
      case TurnLeft:

      break;
      case TurnToFlame:

      break;
      case ApproachFlame:

      break;
      case FanOn:

      break;
      case TurnToStart:

      break;
      case MoveForward:

      break;
      case Reverse:
        forwardController.kp = -forwardController.kp;
        forwardController.ki = -forwardController.ki;
        forwardController.kd = -forwardController.kd;
      break;
      case TurnToWall:

      break;
      case WallFollowingStage2:

      break;
      case TurnLeftStage2:

      break;
      case Finish:

      break;
    }
}

