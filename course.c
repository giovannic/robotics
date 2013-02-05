#pragma config(Sensor, S1,     leftBumper,          sensorTouch)
#pragma config(Sensor, S2,     rightBumper,         sensorTouch)
#pragma config(Sensor, S3,     leftLight,           sensorLightInactive)
#pragma config(Sensor, S4,     rightLight,          sensorLightInactive)

#include "course.h"
#include "towards_light.c"
#include "obstacles.c"
//left is A, right is B
int courseState = -1;


task main()
{
  StartTask(course);
  while(true){}
}

task course()
{
  while (true)
  {

    //light found
    //state 1
    //go forward
    if(centred() || bumping())
    {
      newCourseTask(1);
    }
    //base - look for light
    //state 0
	  else
	  {
	    newCourseTask(0);
	  }

  }
}

void newCourseTask(int newT)
{
  if (courseState != newT)
  {
    if (evading) {return;}
    //if (courseState != -1)
      stopCurrentCourseTask(courseState);

    courseState = newT;
	  switch(newT){
    case 1:
      StartTask(avoid_obstacles);
      break;
    case 0:
      StartTask(towards_light);
      break;
    }
  }
}

void stopCurrentCourseTask(int oldT)
{
  switch(oldT){
    case 1:
      StopTask(avoid_obstacles);
      stopCurrentObstacleTask(obstacleState);
      obstacleState = -1;
      break;
    case 0:
      StopTask(towards_light);
      stopCurrentLightTask(lightState);
      lightState = -1;
      break;
  }
}

bool centred()
{
	return (abs(SensorValue(leftLight) - SensorValue(rightLight)) < centre_threshold) && (SensorValue(leftLight) > bright_light);
}

bool bumping()
{

  return ((SensorValue(leftBumper) == 1) || (SensorValue(rightBumper)) == 1);
}
