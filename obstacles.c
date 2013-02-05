
#include "obstacles.h"
//left is A, right is B

/*
task main()
{
  StartTask(avoid_obstacles);
  while(true){}
}
*/

task avoid_obstacles()
{
  int obstacleState;

  while (true)
  {

    //hit
    //state 1
    if(SensorValue(leftBumper) || SensorValue(rightBumper))
    {
      if (obstacleState != 1)
      {
        stopCurrentObstacleTask(obstacleState);
      }
      obstacleState = 1;
      StartTask(reverseCorner);
    }
    //base
    //state 0
	  else
	  {
	    if (obstacleState != 0)
      {
        stopCurrentObstacleTask(obstacleState);
      }
	    obstacleState = 0;
	    StartTask(moveForward);
	  }

  }
}

void stopCurrentObstacleTask(int t)
{
  switch(t){
    case 1:
      StopTask(reverseCorner);
    case 0:
      StopTask(moveForward);
  }

}

task moveForward()
{
  int speed = 30;
  nMotorPIDSpeedCtrl[motorA] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
  motor[motorA] = speed;
  motor[motorB] = speed;
}

task reverseCorner()
{
  int left = SensorValue(leftBumper);
  int speed = -50;
  int ratio = 30;
  if (left)
  {
    nSyncedMotors = synchBA;
    nSyncedTurnRatio = ratio;
    motor[motorB] = speed;
  } else {
    nSyncedMotors = synchAB;
    nSyncedTurnRatio = ratio;
    motor[motorA] = speed;
  }
}
