#include "WallFollowing.h"
#include "OdometryProject.h"
enum State
  {Calibration,
  WallFollowing,
  TurnRight,
  TurnLeft,
  TurnToFlame,
  ApproachFlame,
  FanOn,
  TurnToStart,
  MoveForward,
  TurnToWall,
  WallFollowingStage2,
  TurnLeftStage2,
  Finish};

State currentState = Calibration;
State lastState = Calibration;

void setup() {
  
}

void loop() {
  // switch block for one-time code on entering each state:
  if(lastState != currentState){
    switch (currentState)
    {
      case Calibration:

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
  //switch block for continuous code running during each state:
  switch (currentState)
    {
      case Calibration:

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
      case TurnToWall:

      break;
      case WallFollowingStage2:

      break;
      case TurnLeftStage2:

      break;
      case Finish:

      break;
    }
    lastState = currentState;
    currentState++;
}
