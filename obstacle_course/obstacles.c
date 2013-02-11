#pragma config(Sensor, S1,     leftBumper,          sensorTouch)
#pragma config(Sensor, S2,     rightBumper,         sensorTouch)
#pragma config(Sensor, S3,     leftLight,           sensorLightInactive)
#pragma config(Sensor, S4,     rightLight,          sensorLightInactive)

#include "obstacles.h"
//left is A, right is B

/*
task main()
{
  while(true){}
}
*/

int obstacleState = -1;
bool evading = false;


task avoid_obstacles()
{
  while (true)
  {

    //hit
    //state 1
    if(SensorValue(leftBumper) || SensorValue(rightBumper))
    {
      newObstacleTask(1);
    }
    //base
    //state 0
	  else
	  {
	    newObstacleTask(0);
	  }

  }
}

void newObstacleTask(int newT)
{
  if (obstacleState != newT)
  {
    if (obstacleState != -1)
      if(evading) {return;}
      stopCurrentObstacleTask(obstacleState);

    obstacleState = newT;
	  switch(newT)
	  {
	    case 1:
	      StartTask(reverseCorner);
	      break;
	    case 0:
	      StartTask(moveForward);
	      break;
    }
  }
}

void stopCurrentObstacleTask(int t)
{
  switch(t)
  {
    case 1:
      StopTask(reverseCorner);
      break;
    case 0:
      StopTask(moveForward);
      break;
  }
}

task moveForward()
{
  int speed = 10;
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = 100;
  motor[motorA] = speed;
}

task reverseCorner()
{
  int left = SensorValue(leftBumper);
  int speed = -20;
  int ratio = 40;
  evading = true;
  if (left)
  {
    nSyncedMotors = synchAB;
    nSyncedTurnRatio = ratio;
    motor[motorA] = speed;
  }
  else
  {
    nSyncedMotors = synchBA;
    nSyncedTurnRatio = ratio;
    motor[motorB] = speed;
  }
  wait1Msec(2500);
  evading = false;
}
