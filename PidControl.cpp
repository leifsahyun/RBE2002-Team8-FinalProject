/**
 * PidControl.cpp
 * 
 * Created On: 9/28/2017
 * Last Modified: 12/9/2017
 * Author: Leif Sahyun
 */

#include "PidControl.h"

/**
 * Constructors initialize the PidControl object with a pointer to the input and an optional initial setpoint.
 */
PidControl::PidControl(float *input, float goal){
  processVariable = input;
  setpoint = goal;
  lastTime = micros();
}

PidControl::PidControl(float *input){
  PidControl::PidControl(input, 0);
}


/**
 * Sets the value of setpoint and clears sumError
 */
void PidControl::setGoal(float goal){
  setpoint = goal;
  sumError = 0;
}

/**
 * Returns current error
 */
float PidControl::getError(){
  noInterrupts();
  float err = setpoint - *processVariable;
  interrupts();
  return err;
}

/**
 * Returns true if the current error is within an acceptable tolerance of the setpoint
 */
bool PidControl::isInTolerance(float errorTolerance){
  float err = getError();
  return (err>-errorTolerance)&&(err<errorTolerance);
}

/**
 * The polymorphic version of isInTolerance with no inputs returns true if the error is within the object's default tolerance
 */
bool PidControl::isInTolerance(){
  return isInTolerance(defaultTolerance);
}

/**
 * Updates the PID control.
 * This is not required to be done on a specific interval so the time difference since the last update is measured and used in the integral and derivative calculations
 * If toPrint is set to true, updatePid will print a summary of the current PidControl to the Serial output for debugging
 */
void PidControl::updatePid(){
  unsigned long now = micros();
  int deltaTime = now - lastTime; 
  float error = getError();
  float deltaError = error - lastError; 
  sumError += (error * deltaTime); //Integral(error) = Sum(error*deltaTime) for a given time interval as deltaTime approaches 0
  derivError = deltaError/deltaTime; //Derivative(error) = deltaError/deltaTime;
  double motorValue = kp*error + ki*sumError + kd*derivError; //Calculation of motorValue with PID constants
  motorValue = motorValue/scale; //The motorValue yielded above is a positive or negative number that may be large depending on the domain of the process variable and the kp, ki, and kd constants.
  //Dividing motorValue by a constant scale makes it easier to adjust the range of output values without changing the pid constants individually.
  outSpeed = motorValue;
  outPosition += outSpeed;
  if(toPrint){ //If toPrint is true, print a summary of the PID control for debugging
    Serial.print("position: ");
    Serial.println(*processVariable);
    Serial.print("error: ");
    Serial.println(error);
    Serial.print("output speed: ");
    Serial.println(outSpeed);
    Serial.print("output position: ");
    Serial.println(outPosition);
  }
  lastTime = now;
  lastError = error;
}

