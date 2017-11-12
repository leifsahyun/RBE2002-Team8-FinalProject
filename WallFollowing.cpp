#include "WallFollowing.h"

void WallFollowing::setup(){
  control = new PidControl(&scaledUltraReading, &motorDiff, (long)(wallDistance*scalingFactor));
}

void WallFollowing::update(){
  ultra->trigger();
  scaledUltraReading = scalingFactor*(ultra->getLastDist());
  control->updatePid();
  leftServo->write(150+motorDiff);
  rightServo->write(150-motorDiff);
}

