#include "PidControl.h"
#include "Ultrasonic.h"

#define INPUT_PIN A0

const int motorRightForwardPin = 4;
const int motorRightReversePin = 5;
const int motorLeftForwardPin = 6;
const int motorLeftReversePin = 7;
const int FORWARD = 255;
const int STALL = 127;
const int REVERSE = 0;

float flameInput =0;
PidControl flameController(&flameInput, 0);

void setup() {
  // put your setup code here, to run once:
  flameController.toPrint=true;
}

void loop() {
  // put your main code here, to run repeatedly:
  flameInput = analogRead(A0);
  flameController.updatePid();
  motorRightWrite(STALL+constrain(flameController.outSpeed, -STALL, STALL+1));
  motorLeftWrite(STALL-constrain(flameController.outSpeed, -STALL, STALL+1));
}

void motorRightWrite(int i){
  analogWrite(motorRightForwardPin, i);
  analogWrite(motorRightReversePin, FORWARD-i);
}

void motorLeftWrite(int i){
  analogWrite(motorLeftForwardPin, i);
  analogWrite(motorLeftReversePin, FORWARD-i);
}
