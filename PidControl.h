/**
 * PidControl.h
 * 
 * Created On: 9/28/2017
 * Last Modified: 12/9/2017
 * Author: Leif Sahyun
 * 
 * Handles PID control for a process variable and an output either directly related to the variable or related to the first derivative of the variable.
 * This class keeps track of error values and has public variables for the constants kp, ki, and kd.
 * Calling updatePid on a PidControl object is not required to be synchronous
 */
#include "Arduino.h"

class PidControl{
  public:
    PidControl(float *input);
    PidControl(float *input, float goal);
    void setGoal(float goal);
    float getError();
    bool isInTolerance(float errorTolerance);
    bool isInTolerance();
    void updatePid();

    //PID constants are public member variables so they can be modified by the main code easily
    double kp=50;
    double ki=0.0001;
    double kd=0.01;
    //Other public member variables include the scale and defaultTolerance
    float scale = 1;
    float defaultTolerance = 5;
    //If toPrint is true, PidControl will print a summary of its control each time updatePid is called to aid debugging
    bool toPrint = false;
    float outSpeed;    //output for use when controlling the first derivative of the input value. For example, if the input is angular displacement in encoder ticks, outSpeed could be the speed a motor should be running at to achieve a specific angular displacement
    float outPosition; //output for use when controlling a value directly related to the input value. For example, if the input is current through a motor, outPosition could be the speed the motor should be running at to achieve a desired current. This is an integral of outSpeed
    
  private:
    float setpoint = 0;
    float *processVariable;
    float lastError;
    float sumError;
    float derivError;
    unsigned long lastTime;
};
