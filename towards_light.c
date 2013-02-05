
#include "towards_light.h"
//left is A, right is B

const float bright_light = 30;
int lightState = 0;

/*
task main()
{
  StartTask(towards_light);
  while(true){}
}
*/

task towards_light()
{
  StartTask(seek);
  while (true)
  {

    //light found
    //state 1
    if((SensorValue(leftLight) > bright_light) || (SensorValue(rightLight) > bright_light))
    {
      newLightTask(1);
    }
    //base
    //state 0
	  else
	  {
	    newLightTask(0);
	  }

  }
}

void newLightTask(int newT)
{
  if (lightState != newT)
  {
    stopCurrentLightTask(lightState);
    lightState = newT;
	  switch(newT){
    case 1:
      StartTask(approach_light);
      break;
    case 0:
      StartTask(seek);
      break;
    }
  }
}

void stopCurrentLightTask(int oldT)
{
  switch(oldT){
    case 1:
      StopTask(approach_light);
      break;
    case 0:
      StopTask(seek);
      break;
  }
}

task seek()
{
  int speed = 10;
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = -100;
  motor[motorA] = speed;
}

task approach_light()
{
  while(true)
  {
	  float left = SensorValue(leftLight);
	  float right = SensorValue(rightLight);
	  float leftness = left - right;
	  float powerConstant = 1;
	  nSyncedMotors = synchAB;
	  nSyncedTurnRatio = -100;
	  motor[motorA] = -powerConstant*leftness;
	}
}
