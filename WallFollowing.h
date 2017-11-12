#include "Servo.h"
#include "PidControl.h"
#include "Ultrasonic.h"
#include "Arduino.h"

namespace WallFollowing {
  int wallDistance = 4;
  const int scalingFactor = 1000;
  long scaledUltraReading = 0;
  int motorDiff = 0;
  Servo *leftServo;
  Servo *rightServo;
  Ultrasonic *ultra;
  PidControl *control;
  void setup();
  void update();
}

