#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <L3G.h>
#include "PidControl.h"
#include "Ultrasonic.h"

/**
 * RBE 2002 Team 8 Project Code
 * Team Members: Everett Johnson, Leif Sahyun, Orion Strickland
 * Date Created: Nov 12, 2017
 * Last Modified: Dec 15, 2017 by Leif Sahyun
 * Dependencies: Servo.h, Wire.h, LiquidCrystal.h, L3G.h, PidControl.h, Ultrasonic.h
 * Description:
 * Main Arduino code to complete the RBE 2002 final project. Tasks include:
 * Extinguishing a flame of unknown location
 * Reporting that location
 * Avoiding cliffs with black lines in front of them
 * 
 * This code accomplishes the above tasks by following a wall with three ultrasonic
 * sensors until it detects the flame with its flame sensors or a cliff with its
 * light sensor. Upon detecting a cliff, the robot will reverse and turn away from
 * the cliff. Upon detecting a flame, the robot will use the input from a differential
 * amplifier to turn towards the flame; then it will approach the flame until it is
 * a set distance away determined with the front ultrasonic. At this point, it will
 * turn the fan on and report the location of the flame on an lcd.
 * 
 * Input Pins: A0, A1, A2, 2, 3, 12, 18, 19, 50, 51, 52, 53
 * Output Pins: 4, 5, 6, 7, 11, 22, 23, 24, 49
 */

/**
 * All input and output pins are defined or declared here.
 * The related values of the pin change interrupt bit numbers are also declared here.
 */
#define FLAME_DIFF A0               //The input for the flame sensor differential amplifier
#define LEFT_FLAME A1               //The input for the left flame sensor raw data
#define TOP_FLAME A2                //The input for the top flame sensor
const int buttonPin = 12;           //The input for the start button
const int motorRightForwardPin = 4; //These four pins are the motor signal outputs
const int motorRightReversePin = 5;
const int motorLeftForwardPin = 6;
const int motorLeftReversePin = 7;
const int fanPin = 49;              //Controls the fan
const int leftEncPin1 = 50;         //These eight values are the encoder input pins and
const int leftEncPin2 = 51;         //their pin change interrupt bit numbers
const int leftEncInt1 = PCINT3;
const int leftEncInt2 = PCINT2;
const int rightEncPin1 = 52;
const int rightEncPin2 = 53;
const int rightEncInt1 = PCINT1;
const int rightEncInt2 = PCINT0;
const int frontUltraTriggerPin = 22;//These three pins are the trigger outputs for the
const int sideUltraTriggerPin = 23; //ultrasonic sensors
const int rearUltraTriggerPin = 24;
const int frontUltraEchoPin = 18;   //These three pins are the echo inputs for the
const int sideUltraEchoPin = 19;    //ultrasonic sensors
const int rearUltraEchoPin = 2;
const int flameAlertPin = 11;       //This is an LED alert for when the flame is detected
const int lightSensorPin = 3;       //This is a digital light sensor for detecting cliffs

/**
 * These are some more constant values for various controls
 */
const float G_gain = .00335;        //This value converts the gyroscope reading to degrees given our update frequency
const float encTicksPerInch = -500; //Originally calculated as -694.5; tests showed that value was too high 
const int FORWARD = 254;            //Value to write to the motors for the named effect - initially, we used 255 for FORWARD
const int STALL = 127;              //however, our motors did not run properly with FORWARD=255
const int REVERSE = 0;
const float wallDist = 4.5;         //Desired distance away from the wall to wall follow at
const int hallWidth = 20;           //Minimum hallway width on the field

enum State
  {Calibration,   //Calibrates robot
  WallFollowing,  //Follows a wall
  TurnRight,      //Turns right 90 degrees to the closest of the following headings: 0, 90, 180, 270 degrees
  TurnLeft,       //Turns left 90 degrees
  TurnToFlame,    //Turns towards the flame
  ApproachFlame,  //Approaches the flame with the front ultrasonic while maintaining alignment with the flame
  FanOn,          //Turns fan on
  MoveForward,    //Moves forward a set distance before and after making right turns in order to avoid walls
  MoveToWall,     //Moves forward until the ultrasonic on the front or those on the sides pick up a wall
  Reverse,        //Moves back 6 inches
  Finish};        //End

struct odometer{
  float x;
  float y;
  float heading;
} pos;

State currentState = Calibration;

/**
 * Objects from included libraries control the gyro, lcd, fan, and ultrasonics
 */
L3G gyro;
LiquidCrystal lcd(40,41,42,43,44,45);
Servo esc;
Ultrasonic frontUltra(frontUltraTriggerPin, frontUltraEchoPin);
Ultrasonic sideUltra(sideUltraTriggerPin, sideUltraEchoPin);
Ultrasonic rearUltra(rearUltraTriggerPin, rearUltraEchoPin);

//gyro reading variables
float gyroError=0;
float gyroHeading = 0;
int calibrationCounter =0;

/**
 * PidControl objects and their associated process variables
 */
float forwardInput=0;
float distInput=0;
float alignInput=0;
float flameInput=0;
float turningInput=0;
float approachInput=0;
PidControl distController(&distInput, wallDist);  //controls distance between robot and wall
PidControl alignController(&alignInput, 0);       //controls alignment of robot with wall
PidControl turningController(&gyroHeading,0);     //controls specific degree turns (uses gyroHeading directly without an intermediate process variable)
PidControl forwardController(&forwardInput,0);    //controls the robot when travelling a set distance forward
PidControl flameController(&flameInput,0);        //controls turning toward the flame
PidControl approachController(&approachInput,0);  //controls the robot approaching the candle (based on the front ultrasonic reading)

/**
 * Program variables
 */
int motorRightSpeed = 0;  //Motor speeds; each loop, these values will be written to the motors
int motorLeftSpeed = 0;
long leftEncCount = 0;    //Encoder tick counts
long rightEncCount = 0;
byte lastReadings = 0;    //This last readings of the PCINT0 interrupt pins, see below
int rightTurnCount = 0;   //Counts the number of right turns so we don't go in circles
int flameError = 1024;    //Minimum value read by the flame sensor when there is no flame - set in calibration
int flameDiffSetpoint = 650; //Op-amp value when flame sensors have an equal reading - set in calibration

bool stopped = true;
bool finalDist = false;   //Whether to print final co-ordinate output
bool calibrating = true;

/**
 * These variables are for timing various parts of the program using millis()
 */
unsigned long timer = 0;
unsigned long calibrationTimer = 0;
unsigned long fanTimer = 0;

/**
 * This is a pin-change interrupt interrupt service routine. When the arduino detects
 * an interrupt on any pin in the PCINT0 port, it will jump to a section of code
 * where the PCINT0 vector is present on an assembly level. The pins attached to PCINT0
 * are: 10, 11, 12, 13, 50, 51, 52, 53. We use pins 50-53 for our encoders. Each time
 * there is an interrupt on one of those pins, this function will be called. This
 * function determines which pin triggered the interrupt by comparing the current pin
 * readings to previous pin readings and using appropriate logic based on quadrature
 * signals to determine whether to add or subtract from each encoder count variable.
 */
ISR(PCINT0_vect){
  byte readings = 0;
  //Read the current values of the encoder pins to readings
  if(digitalRead(leftEncPin1))
    readings |= bit(leftEncInt1);
  if(digitalRead(leftEncPin2))
    readings |= bit(leftEncInt2);
  if(digitalRead(rightEncPin1))
    readings |= bit(rightEncInt1);
  if(digitalRead(rightEncPin2))
    readings |= bit(rightEncInt2);
  //bitwise xor operation determines which bit changed
  byte changes = lastReadings^readings;
  //based on which bit changed and current readings, this logic determines whether to add or subtract
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

/**
 * The setup function initializes all objects and variables before calling loop()
 * Here, we run initialization code for objects that require it as well as set pin
 * modes for output pins and inputs that require a pull-up resistor. We also set
 * all PID values that are not the default specified in PidControl.h here.
 * Finally, we enable our ISR routine above.
 */
void setup() {
  pinMode(fanPin, OUTPUT);
  esc.attach(fanPin);
  esc.write(10);      //important to write 10 to esc on startup for esc arming sequence
  pinMode(motorRightForwardPin, OUTPUT);
  pinMode(motorRightReversePin, OUTPUT);
  pinMode(motorLeftForwardPin, OUTPUT);
  pinMode(motorLeftReversePin, OUTPUT);
  pinMode(flameAlertPin, OUTPUT);
  pinMode(frontUltraTriggerPin, OUTPUT);
  pinMode(sideUltraTriggerPin, OUTPUT);
  pinMode(rearUltraTriggerPin, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(43, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(leftEncPin1, INPUT);
  pinMode(leftEncPin2, INPUT);
  pinMode(rightEncPin1, INPUT);
  pinMode(rightEncPin2, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  digitalWrite(flameAlertPin, LOW);
  //Serial.begin(115200); Begin serial output for debugging
  lcd.begin(16,2);
  Wire.begin();
  gyro.init();
  gyro.enableDefault();
  /*
   * These optional lines of code make the PidControllers print debugging output
  distController.toPrint = true;
  alignController.toPrint = true;
  turningController.toPrint = true;
  forwardController.toPrint = true;
  flameController.toPrint = true;*/
  
  /**
   * Set the PID constants for the controllers if default does not work
   * All the constants were determined by trial and error
   * Also change the default tolerance if necessary 
   */
  flameController.kp = 0.25;
  flameController.ki = 0;
  flameController.kd = 2;
  flameController.defaultTolerance = 100;
  distController.kp = 25;
  distController.kd = 1;
  alignController.kd = 1;
  forwardController.kp = 25;
  forwardController.ki = -0.00001;
  forwardController.defaultTolerance = 1;
  approachController.ki = 0;
  turningController.kp = 5;
  turningController.ki = 0;
  turningController.kd = 1;
  turningController.defaultTolerance = 2.5;
  /**
   * These two lines of code enable pin change interrupts on pins 50-53 by setting
   * bits in specific Arduino control registers to 1.
   */
  PCMSK0 |= bit(leftEncInt1) | bit (leftEncInt2) | bit(rightEncInt1) | bit(rightEncInt2); //enable bits corresponding to encoder pins in the interrupt enable mask register
  PCICR  |= bit (PCIE0); //enable bit corresponding to pins 10-13 and 50-53 in the interrupt enable register.
}

/**
 * Our main loop. It runs mainUpdate; then, if not in calibration mode, it reads our sensors
 * It also writes the current state, robot heading, and robot position to the lcd
 */
void loop() {
  /**
   * Debugging output
   * Serial.println("here");
   * Serial.println(currentState);
   * Serial.println(digitalRead(buttonPin));
   * Serial.print("flame sensor: ");
   * Serial.println(analogRead(LEFT_FLAME));
   * Serial.print("error: ");
   * Serial.println(flameError);
   */
  mainUpdate();
  if(calibrating)
    return;
  safeTriggerAllUltras();
  updateGyro();
  //if we have printed the final measurements on the lcd, then do not overwrite them
  //otherwise, print the current state, heading, and position
  if(!finalDist){
    lcd.clear();
    lcd.print(currentState);
    lcd.print(" ");
    lcd.print((int)gyroHeading);
    lcd.print(" ");
    lcd.print(pos.x);
    lcd.print(" ");
    lcd.print(pos.y);
  }
  /**
   * More debugging output
   * Serial.print("DISTANCE FRONT: ");
   * Serial.println(frontUltra.getLastDist());
   * Serial.print("DISTANCE SIDE: ");
   * Serial.println(sideUltra.getLastDist());
   * Serial.print("DISTANCE REAR: ");
   * Serial.println(rearUltra.getLastDist());
   */
}

/**
 * This is our main update function. It is called from the main loop and computes all state
 * changes and state-based functionality of the robot and also updates the PidControllers 
 * and writes the motor speed variables to the motors
 */
void mainUpdate(){
  if(checkStateChange()) //determine if the state changes, if it does, begin the new one
  {
    if(calibrating)
      endCalibration();
    beginState();
  }
  continueState();
  updateControllers();
  if(!stopped){         //write the motor speed values to the motors unless the robot is stopped
    motorRightWrite(constrain(motorRightSpeed, REVERSE, FORWARD));
    motorLeftWrite(constrain(motorLeftSpeed, REVERSE, FORWARD));
  } else {
    motorRightWrite(STALL);
    motorLeftWrite(STALL);
  }
}

/**
 * This writes a value 0-254 to the right motor by writing the value to the forward pin
 * and writing 254 minus that value to the reverse pin.
 */
void motorRightWrite(int i){
  analogWrite(motorRightForwardPin, i);
  analogWrite(motorRightReversePin, FORWARD-i);
}

/**
 * This writes a value 0-254 to the left motor by writing the value to the forward pin
 * and writing 254 minus that value to the reverse pin.
 */
void motorLeftWrite(int i){
  analogWrite(motorLeftForwardPin, i);
  analogWrite(motorLeftReversePin, FORWARD-i);
}

/**
 * This function maintains a ratio between the left and right motor speeds
 * while scaling up the higher speed to maximum
 */
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

/**
 * This function updates the gyroHeading variable with a reading from the gyroscope
 * This function is called every loop
 */
void updateGyro(){
  gyro.read();
  gyroHeading += (gyro.g.z - gyroError)*G_gain;
}

/**
 * This function allows us to snap the gyro reading to a wall when the side
 * ultrasonic sensor readings are within 0.5 in of each other. It takes a
 * gyro reading and snaps it to the closest increment of 90 degrees
 * Input: an angle in degrees
 * Output: the closest increment of 90 degrees to the input
 */
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


/**
 * This function resets encoders and is called in order to account for the
 * angle of the robot changing
 */
void clearEncoders(){
  pos.heading = gyroHeading;
  leftEncCount = 0;
  rightEncCount = 0;
}

/**
 * This function updates the odometer with changes in position since the last
 * call of this function; then it resets the encoders
 * This function is only called when the robot is moving forward, not when it
 * is turning or moving in reverse
 */
void positionUpdate(){
  //dist is the amount the robot has moved forward since the last call of clearEncoders()
  float dist = (leftEncCount + rightEncCount)/2/encTicksPerInch;
  forwardInput+=dist; //This updates the process variable for the PidControl object that
                      //controls the robot when it moves a set distance
  /**
   * debugging output
   * if(currentState!=Calibration){
   * Serial.print("dist: ");
   * Serial.println(dist);
   * }
   */
  pos.x+=(dist*cos(pos.heading*M_PI/180));
  pos.y+=(dist*sin(pos.heading*M_PI/180));
  /**
   * debugging output
   * if(currentState!=Calibration){
   * Serial.print("x: ");
   * Serial.println(dist*cos(pos.heading*M_PI/180));
   * Serial.print("y: ");
   * Serial.println(dist*sin(pos.heading*M_PI/180));
   * Serial.print("heading: ");
   * Serial.println(pos.heading);
   * }
   */
  clearEncoders();
}

/**
 * This function does the same thing as positionUpdate, above, but has a negative distance.
 * We did this because our encoders were reading positive values when running in reverse
 * so when the robot runs in reverse we call this function to properly track our position.
 */
void negPositionUpdate(){
  float dist = (leftEncCount + rightEncCount)/2/encTicksPerInch;
  forwardInput-=dist;
  pos.x-=dist*cos(pos.heading*M_PI/180);
  pos.y-=dist*sin(pos.heading*M_PI/180);
  clearEncoders();
}

/**
 * This function updates all PidControl objects
 */
void updateControllers(){
  flameController.updatePid();
  approachController.updatePid();
  distController.updatePid();
  alignController.updatePid();
  turningController.updatePid();
  forwardController.updatePid();
}

/**
 * This function triggers all the ultrasonic sensors so that none of them interfere with
 * each other. This requires a 100 ms delay between each trigger.
 * This function requires at least 300 ms.
 */
void safeTriggerAllUltras(){
  int failCount;
  failCount = 0;
  timer = millis();
  while(millis()<timer+100);
  while(!frontUltra.trigger()){   //The Ultrasonic object will return 0 if it failed to trigger
    failCount++;
    if(failCount>10){
      Ultrasonic::triggeredObj = NULL;  //Usually, if the Ultrasonic object fails to trigger
                                        //several times, it is because a previous trigger call
                                        //never received an echo. Clearing the triggeredObj
                                        //variable solves this problem
      failCount = 0;
    }
  }
  failCount = 0;
  timer = millis();
  while(millis()<timer+100);
  while(!sideUltra.trigger()){
    failCount++;
    if(failCount>10){
      Ultrasonic::triggeredObj = NULL;
      failCount = 0;
    }
  }
  failCount = 0;
  timer = millis();
  while(millis()<timer+100);
  while(!rearUltra.trigger()){
    failCount++;
    if(failCount>10){
      Ultrasonic::triggeredObj = NULL;
      failCount = 0;
    }
  }
}

/**
 * This function processes wall following control. After it is called, the motor speed
 * variables are set to appropriate values for wall following according to the formula:
 * speed_out = half_speed + 3/4 * half_speed * distController.outSpeed + 1/4 * half_speed * alignController.outSpeed
 * With this formula, the speed control incorporates both distance from the wall and alignment
 * without giving too much weight to either
 */
void wallFollowingSequence(){
  positionUpdate();
  //update the process variables for the PidControllers
  distInput = (sideUltra.getLastDist() + rearUltra.getLastDist())/2; //average distance from wall
  alignInput = sideUltra.getLastDist()-rearUltra.getLastDist();      //difference between wall sensors
  //constrain output from distance controller to 3/4 weight
  int distOut = constrain(distController.outSpeed, -48, 48);
  //constrain output from alignment controller to 1/4 weight
  int alignOut = constrain(alignController.outSpeed, -16, 16);
  //if the ultrasonic readings are within 0.5 in of each other, we can snap the gyro reading to the wall
  if(alignController.isInTolerance(0.5))
    gyroHeading = gyroSnapToWall(gyroHeading);
  motorRightSpeed = STALL + 64+distOut+alignOut;
  motorLeftSpeed = STALL + 64-distOut-alignOut;
  scaleToMaxSpeed(); 
}

/**
 * This function controls the robot's forwards movement during the MoveForward state
 */
void forwardSequence(){
  positionUpdate();
  motorRightSpeed = STALL + constrain(forwardController.outSpeed,-STALL,STALL);
  motorLeftSpeed = motorRightSpeed;
}

/**
 * This function controls the robot's movement during any turn to a specified angle
 */
void turningSequence(){
  motorRightSpeed = STALL-(int)constrain(turningController.outSpeed, -STALL, STALL);
  motorLeftSpeed = STALL+(int)constrain(turningController.outSpeed, -STALL, STALL);
}

/**
 * This function controls the robot's movement when it turns toward the flame
 */
void flameTurningSequence(){
  flameInput = analogRead(FLAME_DIFF);
  motorRightSpeed = STALL+(int)constrain(flameController.outSpeed, -STALL, STALL);
  motorLeftSpeed = STALL-(int)constrain(flameController.outSpeed, -STALL, STALL);
}

/**
 * This function controls the robot's movement as it approaches the flame.
 * Similarly to the wallFollowingSequence, this function uses two PidControl objects to determine the output speeds.
 * This is so that, as the robot approaches the flame, it stays pointed toward the flame.
 */
void flameApproachSequence(){
  positionUpdate();
  approachInput = frontUltra.getLastDist();
  flameInput = analogRead(FLAME_DIFF);
  motorRightSpeed = STALL-(int)(0.5*constrain(approachController.outSpeed, -STALL, STALL)) + (int)(0.5*constrain(flameController.outSpeed, -STALL, STALL));
  motorLeftSpeed = STALL-(int)(0.5*constrain(approachController.outSpeed, -STALL, STALL)) - (int)(0.5*constrain(flameController.outSpeed, -STALL, STALL));
}

/**
 * This function calibrates the robot for 2 seconds after it powers on
 */
void calibrationSequence(){
  if(millis() < calibrationTimer+2000){
    gyro.read();
    //add to the gyro error value in order to take the average later
    gyroError += gyro.g.z;
    //determine the minimum reading of the flame sensor with no flame
    if(flameError > analogRead(A1) && analogRead(A1)!=0)
      flameError = analogRead(A1);
    //used for the gyroscope average error calculation later
    calibrationCounter++;
  }
}

/**
 * This function reads the op-amp output when left and right flame sensor readings are equal, computes the average gyroscope error,
 * ends the arming sequence of the esc, and ends calibration.
 */
void endCalibration(){
  flameDiffSetpoint = analogRead(FLAME_DIFF); //set the flameDiffSetpoint since the left and right flame sensor readings will be equal here
  gyroError = gyroError/calibrationCounter;
  esc.write(0);
  calibrating = false;
  stopped = false;
}

/**
 * This function completes the goal of the project by doing the final calculations of the flame position and then
 * turning the fan on.
 */
void fanBeginSequence(){
  //stop the robot - it will not move again
  stopped = true;
  //get some last readings for the height calculation
  int leftIn=analogRead(LEFT_FLAME);
  int topIn = analogRead(TOP_FLAME);
  //turn the fan on
  esc.write(170);
  //set the fan timer, it will be on for 5 seconds
  fanTimer = millis();
  frontUltra.getLastDist();
  //calculate and print the final distance values
  finalDist = true;
  lcd.clear();
  lcd.print(" X: ");
  lcd.print((int)(pos.x + (frontUltra.getLastDist()+5) * cos(gyroHeading*M_PI/180)));
  lcd.print(" Y: ");
  lcd.print((int)(pos.y + (frontUltra.getLastDist()+5) * sin(gyroHeading*M_PI/180)));
  lcd.setCursor(0,1);
  lcd.print(" Z: ");
  int flameLeftReading = analogRead(LEFT_FLAME);
  int flameTopReading = analogRead(TOP_FLAME);
  double height;
  /**
   * The difference between flameLeftReading and flameTopReading will be between 0 and 1024.
   * Here, we map the maximum and minimum difference directly to the maximum and minimum candle
   * heights based on which sensor reads lower. 10 in above the field is halfway between the sensors.
   */
  if(flameLeftReading<flameTopReading){ //if the flame is closer to the left sensor, then candle is below 10 in
    height = constrain(map(flameTopReading-flameLeftReading, 0, 1024, 9, 4), 4, 9);
  } else {                              //if the flame is closer to the top sensor, then candle is above 10 in
    height = constrain(map(flameLeftReading-flameTopReading, 0, 1024, 10, 12), 10, 12);
  }
  lcd.print(height);
}

/**
 * Based on the current state, this function calls another function to process that state's operation.
 */
void continueState(){
  switch (currentState)
    {
      case Calibration:
        calibrationSequence();
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
      case TurnToFlame:
        flameTurningSequence();
      break;
      case ApproachFlame:
        flameApproachSequence();
      break;
      case FanOn:
        //Nothing to do for this state
      break;
      case MoveForward:
        forwardSequence();
      break;
      case MoveToWall:
        forwardSequence();
      break;
      case Reverse:
        forwardSequence();
      break;
      case Finish:
        //Nothing to do for this state
      break;
    }
}

/**
 * Check if the state should change, if it should, change the state and return true, otherwise return false
 * Returns: true if the state changed, false otherwise
 */
bool checkStateChange(){
  if(currentState!=Calibration && !digitalRead(lightSensorPin)){ //if a cliff is detected
    currentState = Reverse;
    return true;
  }
  if (currentState!=Calibration && currentState!=ApproachFlame && currentState!=FanOn && currentState!=Finish){
    //if a flame is detected, enter the TurnToFlame state. Do not do this in any of the states listed in the above if statement
    if(analogRead(LEFT_FLAME)<flameError - 10){
      currentState = TurnToFlame;
      return true;
    }
  }
  switch (currentState)
    {
      case Calibration:
      //calibration of the flame sensor and gyroscope only lasts 2 seconds; however, the esc for the fan requires
      //at least 5 seconds to complete its arming sequence
        if(digitalRead(buttonPin)==0 && millis()>calibrationTimer+5000){
          currentState = WallFollowing; //start in the wall following state
          return true;
        }
      break;
      case WallFollowing:
        if(sideUltra.getLastDist()>hallWidth){  //if there is no wall on the right, go forward
          currentState = MoveForward;
          return true;
        }
        if(frontUltra.getLastDist()<wallDist+1){//if there is a wall in front, turn left
          currentState = TurnLeft;
          return true;
        }
      break;
      case TurnRight:
        if(turningController.isInTolerance()){  //only change the state once the robot has turned 90 degrees right plus or minus 2.5 degrees
          if(sideUltra.getLastDist()<wallDist){ //if there is a wall on the right, start wall following
            currentState = WallFollowing;
            rightTurnCount = 0;
          }
          else if(rightTurnCount >= 2){         //if we have turned right at least twice, go forward to a wall
            currentState = MoveToWall;
            rightTurnCount = 0;
          }
          else{
            currentState = MoveForward;         //otherwise go forward a set distance
          }
          return true;
        }
      break;
      case TurnLeft:
        if(turningController.isInTolerance()){  //change state once the robot has turned 90 degrees left plus or minus 2.5 degrees
          if(sideUltra.getLastDist()>hallWidth/2) //if there is no wall on the right, move to a wall
            currentState = MoveToWall;
          currentState = WallFollowing;           //if there is a wall on the right, start wall following
          return true;
        }
      break;
      case TurnToFlame:
        if(flameController.isInTolerance()){      //if the amplified difference between the left and right sensors is within a tolerance, approach the flame
          currentState = ApproachFlame;
          return true;
        }
      break;
      case ApproachFlame:
        if(frontUltra.getLastDist()<8){           //if the robot is within 8 inches of the flame, turn the fan on
          currentState = FanOn;
          return true;
        }
      break;
      case FanOn:
        if(millis()>fanTimer+5000){               //if the fan has run for 5 seconds, finish the program
          currentState = Finish;
          return true;
        }
      break;
      case MoveForward:
        if(frontUltra.getLastDist()<wallDist+1){  //if there is a wall in front, turn right
          currentState = TurnRight;
          return true;
        }
        if(sideUltra.getLastDist()<hallWidth/2 && rearUltra.getLastDist()<hallWidth/2){ //if there is a wall on the right, start wall following
          currentState = WallFollowing;
          return true;
        }
        if(forwardController.isInTolerance()){    //once the robot has gone the set distance, turn right
          currentState = TurnRight;
          return true;
        }
      break;
      case MoveToWall:
        if(frontUltra.getLastDist()<wallDist+1){  //if there is a wall in front, turn left
          currentState = TurnLeft;
          return true;
        }
        if(sideUltra.getLastDist()<hallWidth/2 && rearUltra.getLastDist()<hallWidth/2){ //if there is a wall on the right, start wall following
          currentState = WallFollowing;
          return true;
        }
      break;
      case Reverse:
        if(forwardController.isInTolerance()){    //once the robot has gone the set distance, turn left
          currentState = TurnLeft;
          return true;
        }
      break;
      case Finish:

      break;
    }
  return false;
}

/**
 * Begins a new state after the state has been changed in checkStateChange()
 * Usually this means setting a goal for one or more PidController objects related to the state.
 * Setting a goal for a PidController clears the error integral in the object, so no previous
 * updates of the PidController will effect it with the new setpoint. This also allows us to
 * update all the PidControllers at once without regard for which are currently active.
 * If the state is one that tracks robot position, then clear the encoders here in order to
 * avoid adding to the robot position with encoder values from turning or reverse states.
 */
void beginState(){
  switch (currentState)
    {
      case Calibration:
        calibrationTimer = millis();                //set the calibration timer
      break;
      case WallFollowing:
        clearEncoders();
        distController.setGoal(wallDist);           //set the PidControllers for wall following
        alignController.setGoal(0);
      break;
      case TurnRight:
        if(gyroHeading>270 && gyroHeading<360)      //if the heading is above 270, we do not want to turn to 0 degrees: 270 degrees to the left
          turningController.setGoal(360);
        else
          turningController.setGoal(gyroSnapToWall(gyroHeading+85)); //turn right to the closest of the following headings: 0, 90, 180, or 270
        rightTurnCount++;
      break;
      case TurnLeft:
        turningController.setGoal(gyroHeading-90);  //turn left 90 degrees
      break;
      case TurnToFlame:
        digitalWrite(flameAlertPin, HIGH);          //when the flame is detected, turn on the flame alert LED
        flameController.setGoal(flameDiffSetpoint); //turn so the left and right flame sensors are equal. That is, where the amplified difference
                                                    //between the two is what was measured during calibration
      break;
      case ApproachFlame:
        clearEncoders();
        approachController.setGoal(6);              //move forward until 6 inches from the candle holder
      break;
      case FanOn:
        fanBeginSequence();                         //This case has a lot of code, so it is placed in a helper function, above.
      break;
      case MoveForward:
        clearEncoders();
        forwardInput = 0;
        forwardController.setGoal(hallWidth);       //Move forward a set distance in order to avoid crashing into walls when turning right
      break;
      case MoveToWall:
        forwardInput = 0;
        forwardController.setGoal(200);             //Move forward a very large distance (the robot will never reach this distance, this state just moves forward until it encounters a wall)
      break;
      case Reverse:
        forwardInput = 0;
        forwardController.setGoal(-6.0);            //Move back 6 inches to avoid a cliff
      break;
      case Finish:
        esc.write(0);                               //Stop the fan
        digitalWrite(flameAlertPin, LOW);           //Turn off the LED to show the flame has been extinguished
        stopped = true;                             //Stop the robot - it should already be stopped
      break;
    }
}

