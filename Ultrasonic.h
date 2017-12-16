/**
 * Ultrasonic.h
 * 
 * Created On: 11/11/2017
 * Last Modified: 12/11/2017
 * Author: Leif Sahyun
 * 
 * Controls any number of HC-SR04 ultrasonic sensors where the sensor echo pins are attached to interrupt
 * pins on the Arduino. Each ultrasonic sensor must be triggered before it can be read and only one
 * ultrasonic can be triggered at a time. This is because Arduino ISRs must be static and because
 * triggering multiple ultrasonics can result in interference.
 */

#include "Arduino.h"

class Ultrasonic {
  
  public:
    Ultrasonic(int trigPin, int echoPin); //Creates an Ultrasonic object. The echo pin must be an interrupt pin.
    double getLastDist();                 //Gets the distance read by the sensor
    unsigned long getLastTime();          //Gets the raw time delay read by the sensor in microseconds
    bool trigger();                       //Triggers the sensor. Causes the sensor to send ultrasonic pulses. This method takes at least 10 microseconds to run.
    bool checkEcho();                     //Checks whether sensor measurements have changed since the last time checkEcho was called. Recommended to call checkEcho before calling getLastDist or getLastTime.
    void calibrate(int timeSense, int constDist);
    void resetTriggers();
        static Ultrasonic *triggeredObj;      //Currently active/triggered Ultrasonic object. Used for static ISRs so they can apply to specific objects.


  private:
    static void echoInterruptRising();    //Measures the time at the rising edge of the echo signal.
    static void echoInterruptFalling();   //Measures the time at the falling edge of the echo signal and calculates the delay. Also sets echoFlag to true.
    const double timeDistConvFactor = 150;//Divide the time delay by this constant to get the distance. Value determined experimentally.
    int timeDistOffset = 0;       //Subtract this constant from the time delay to get the distance. Value determined experimentally.
    int trigPin;                          //Sensor trigger pin
    int echoPin;                          //Sensor echo pin
    unsigned long triggerTime;            //Time measured by echoInterruptRising
    volatile long timeDiff;      //Delay of echo signal in microseconds
    bool echoFlag;                        //Whether the sensor measurements have changed since the last time checkEcho was called
};

