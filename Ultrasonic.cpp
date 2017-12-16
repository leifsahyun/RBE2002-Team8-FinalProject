/**
 * Ultrasonic.cpp
 * 
 * Created On: 11/11/2017
 * Last Modified: 12/11/2017
 * Author: Leif Sahyun
 */

#include "Ultrasonic.h"

Ultrasonic *Ultrasonic::triggeredObj = NULL;

Ultrasonic::Ultrasonic(int trigPin, int echoPin){
  this->trigPin = trigPin;
  this->echoPin = echoPin;
}

bool Ultrasonic::trigger(){
  if(triggeredObj){ //if there is a currently triggered ultrasonic object, do not trigger this object
    return false;
  } //if there is no currently triggered object, proceed
  triggeredObj = this;
  attachInterrupt(digitalPinToInterrupt(echoPin), echoInterruptRising, RISING);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return true;
}

void Ultrasonic::echoInterruptRising(){
  if(!triggeredObj)
    return;
  triggeredObj->triggerTime = micros();
  attachInterrupt(digitalPinToInterrupt(triggeredObj->echoPin), echoInterruptFalling, FALLING);
}

void Ultrasonic::echoInterruptFalling(){
  if(!triggeredObj)
    return;
  detachInterrupt(digitalPinToInterrupt(triggeredObj->echoPin));
  unsigned long echoTime = micros();
  triggeredObj->timeDiff = echoTime - triggeredObj->triggerTime;
  digitalWrite(triggeredObj->trigPin, LOW);
  triggeredObj->echoFlag = true;
  triggeredObj = NULL;
}

unsigned long Ultrasonic::getLastTime(){
  return timeDiff;
}

double Ultrasonic::getLastDist(){
  return (double)((timeDiff-timeDistOffset)/timeDistConvFactor);
}

bool Ultrasonic::checkEcho(){
  //return echoFlag;
  if(echoFlag){
    echoFlag = false;
    return true;
  } else {
    return false;
  }
}

void Ultrasonic::calibrate(int timeSense, int constDist){
  timeDistOffset = timeSense - constDist*timeDistConvFactor;
}
    
static void Ultrasonic::resetTriggers(){
  triggeredObj = NULL;
}

